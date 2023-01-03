
import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import math 
from sensor_msgs.msg import LaserScan
# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images
import tf
from geometry_msgs.msg import Twist

# Initialize socketio server and Flask application 
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

rospy.init_node("base_scan")
pub = rospy.Publisher('/base_scan', LaserScan, queue_size = 10)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying 
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying 
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of naviagation
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        self.nav_angles = [] # Angles of navigable terrain pixels
        self.nav_dists = [] # Distances of navigable terrain pixels
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.mode = 'forward' # Current mode (can be forward or stop)
        self.throttle_set = 1 # Throttle setting when accelerating
        self.brake_set = 0.2 # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 40 # Threshold to initiate stopping
        self.angle_forward = 40 # Threshold angle to go forward again
        self.can_go_forward = True # tracks clearance ahead for moving forward
        # pixel distance threshold for how close to a wall before turning around
        self.mim_wall_distance = 40
        # pitch angle for when the rover is considered to have climbed a wall
        # self.pitch_cutoff =1.3
        self.max_vel = 1.1 # Maximum velocity (meters/second)
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float)
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float)
        self.sample_angles = None  # Angles of sample pixels
        self.sample_dists = None  # Distances of sample pixels
        self.sample_detected = False # set to True when sample found in image
        self.sample_stop_forward = 7  # Threshold to initiate stopping
        self.samples_pos = None # To store the actual sample positions
        self.samples_found = 0 # To count the number of samples found
        self.near_sample = 0 # Will be set to telemetry value data["near_sample"]
        self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False # Set to True to trigger rock pickup
 
        # keeps track of the turn direction for on the spot rotations
        self.turn_dir = 'none'
# Initialize our rover 
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
fps = None

# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):
    scann = LaserScan()

    global frame_counter, second_counter, fps
    frame_counter+=1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
    #print("Current FPS: {}".format(fps))

    if data:
        global Rover
        global point_cloud
        distances=[]
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)

        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state
            # Execute the perception and decision steps to update the Rover's state
            Rover,point_cloud,distances= perception_step(Rover)
            #print(point_cloud)
            point_cloud = np.array(point_cloud)
            if(len(point_cloud)!=0):
                angles = np.arctan2(point_cloud[:,1], point_cloud[:, 0])
                min_angle, max_angle = np.min(angles), np.max(angles)
                scann.angle_increment = (max_angle - min_angle)/ len(point_cloud)
            else:
                min_angle, max_angle =0,0
            
            current_time = rospy.Time.now()
            scann.header.stamp = current_time
            scann.header.frame_id = 'laser'
            scann.angle_min = min_angle
            scann.angle_max = max_angle
            # scann.angle_increment = (max_angle - min_angle)/ len(point_cloud)
            scann.time_increment = len(point_cloud)/15
            scann.range_min = 0.00999999977648
            scann.range_max = 32.0 # To Test for later
            scann.ranges = distances
            pub.publish(scann)
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.header.stamp = rospy.Time.now()
               
            odom.pose.pose.position.x = Rover.pos[0] # Replace with the current pose
            odom.pose.pose.position.y = Rover.pos[1]
            odom.pose.pose.position.z =0
            # Create a Twist message to represent the velocity
            velocity = Twist()
            velocity.linear.x = Rover.vel  # Replace with the linear x velocity
            velocity.linear.y = 0  # Set the linear y velocity to 0
            velocity.linear.z = 0  # Set the linear z velocity to 0
            velocity.angular.x = 0  # Set the angular x velocity to 0
            velocity.angular.y = 0  # Set the angular y velocity to 0
            velocity.angular.z = 0  # Set the angular z velocity to 0

            # Assign the Twist message to the odom.twist.twist field
            odom.twist.twist = velocity
            odom_pub.publish(odom) # Publish the message
            br = tf.TransformBroadcaster()
            translation = (Rover.pos[0]/50,Rover.pos[1]/50, 0)
            quat = tf.transformations.quaternion_from_euler(math.radians(Rover.roll),math.radians(Rover.pitch),math.radians(Rover.yaw))
            timestamp = rospy.Time.now()

            # Send the transform to be broadcasted
            br.sendTransform(translation, quat, timestamp,"base_link","odom" )
            # br.sendTransform(translation, quat, timestamp,"base_link","odom" )
            br2 = tf.TransformBroadcaster()
            translation2 = (0,0, 0)
            quat2 =(0,0,0,1)
            timestamp2 = rospy.Time.now()
            br2.sendTransform(translation2, quat2, timestamp2,"laser","base_link")
            Rover = decision_step(Rover)


            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!
            commands = (Rover.throttle, Rover.brake, Rover.steer)
            send_control(commands, out_image_string1, out_image_string2)
 
            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
        # In case of invalid telemetry, send null commands
        else:

            # Send zeros for throttle, brake and steer and empty images
            send_control((0, 0, 0), '', '')

        # If you want to save camera images from autonomous driving specify a path
        # Example: $ python drive_rover.py image_folder_path
        # Conditional to save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))

    else:
        sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)

def send_control(commands, image_string1, image_string2):
    # Define commands to be sent to the rover
    data={
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image1': image_string1,
        'inset_image2': image_string2,
        }
    # Send commands via socketIO server
    sio.emit(
        "data",
        data,
        skip_sid=True)

# Define a function to send the "pickup" command 
def send_pickup():
    print("Picking up")
    pickup = {}
    sio.emit(
        "pickup",
        pickup,
        skip_sid=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()
    
    os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("Recording this run ...")
    else:
        print("NOT recording this run ...")
    
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
