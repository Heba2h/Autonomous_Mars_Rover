import numpy as np


class PID:
    
    def __init__(self,kp,ki,kd) :

        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.eSum=0
        self.e=0
       

    def get_longitudinal_control(self,currentState,desired_throttle,dt):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        currentState: np.array of floats, shape=6
        Current State Data    [x, y, theta, throttle ]
        
        dt: float
            Delta time since last time the function was called
        
        desired_speed: float 
            Desired speed
        
        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''

        self.eSum=0 
        e_past=0
        
        current_vel=currentState[3]


        e = desired_throttle - current_vel

        self.eSum = self.eSum + e  
        dedt=(e-e_past)/dt 
        action = self.kp*e + self.ki * self.eSum + self.kd * dedt

        print(action)

        if(action<0):
            action=0
        elif (action>1):
            action=1
        # throttle_output = np.tanh(action)
        e_past=e
        return action

        
pid=PID(3,0.25,0.01)


def forward(Rover, speed , desired_thottle,steer, can_move):

    longitudunal_Control=0 
    # moves the rover forward
    if can_move:
        if Rover.vel < Rover.max_vel:
            # Set throttle value to controlled throttle setting
            longitudunal_Control = pid.get_longitudinal_control(np.array([Rover.pos[0],Rover.pos[1],Rover.nav_angles,speed]),desired_thottle,(1/25))
            Rover.throttle = longitudunal_Control
        else: 
            Rover.throttle = 0
            
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.clip(steer, -15, 15)
    else:
        # objects in the way so turn around
        Rover.mode ='turn_around'

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # check which state the rover is in and calles the functions to handle that state
    try:
    # only perform state updates if the rover is not picking up a sample rock
        if Rover.picking_up == 0 and Rover.send_pickup is False:

            if Rover.mode == 'sample': # state in sample found
                if Rover.sample_angles.any():
                    # moves towards the found sample and picks it up when close
                    # check that there is sample data available
                    if len(Rover.sample_dists) > 0:
                        distance = np.mean(Rover.sample_dists)
                    else:
                        distance = 30
                    if len(Rover.sample_angles) > 0:
                        steer =np.mean(Rover.sample_angles * 180 / np.pi)
                    else:
                        steer = 0

                    # check if the sample can be picked up
                    if Rover.near_sample > 0:
                        if Rover.vel > 0.2:
                            # travelling to fast so stop
                            # slow the rover down when near objects
                            Rover.throttle = 0
                            Rover.brake = 0.2
                            Rover.steer = np.clip(0.8*steer, -15, 15)
                        elif Rover.vel <= 0.1:
                            # the sample can be picked up so pick it up

                            # stop the movement of the rover
                            # If we're in stop mode but still moving keep braking
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0

                            # once stop set the next state
                            if Rover.vel == 0:
                                if Rover.can_go_forward:
                                    Rover.mode = 'forward'
                                else:
                                    Rover.mode = 'turn_around'
                                    
                            Rover.send_pickup = True
                            # Rover.mode = 'turn_around'

                    elif distance < 60:          #ana edit here #####loooooooooooook
                        # slow the rover down if it is close to the sample
                        if Rover.vel > 0.2:
                            # slow the rover down when near objects
                            Rover.throttle = 0
                            Rover.brake = 0.1
                            Rover.steer = np.clip(0.8*steer, -15, 15)
                        else:
                            # the rover stop not close enough to pick up the sample
                            forward(Rover, Rover.vel,0.2, 0.8*steer, True)
                    elif Rover.sample_angles is not None:
                        # sample is not close so move towards it
                        forward(Rover, Rover.vel, Rover.throttle_set ,steer*0.8, True)
                    else:
                        Rover.mode = 'turn_around'
                else:
                    Rover.mode = 'turn_around'
    
            elif Rover.mode == 'turn_around':
                    # turns the rover around until the path is clear
                Rover.throttle = 0
                Rover.brake = Rover.brake_set

                # check if the path is clear
                if Rover.can_go_forward:
                    Rover.mode = 'forward'
                    Rover.turn_dir = 'none'
                    Rover.brake = 0
                    
                elif Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = 0.1
                    Rover.steer = 0
                else:
                    # check which direction the rover last rotated about and
                    # continue in that direction.
                    if Rover.turn_dir == 'none':
                        # first movement will turn to the side of the best chance
                        # of a clear path or if none will turn right
                        if len(Rover.nav_angles) > 0:
                            if (Rover.nav_angles[int(len(Rover.nav_angles)/3)] * 180/np.pi) > 0:
                                Rover.turn_dir = 'left'
                                Rover.steer = 15
                            else:
                                Rover.turn_dir = 'right'
                                Rover.steer = -15
                        else:
                            Rover.turn_dir = 'right'
                            Rover.steer = -15
                    elif Rover.turn_dir == 'left':
                        Rover.steer = 15
                    elif Rover.turn_dir == 'right':
                        Rover.steer = -15
                    else:
                        Rover.turn_dir = 'none'
                        Rover.steer = -15

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0

            elif Rover.mode == 'forward' and Rover.can_go_forward:
                # Check the extent of navigable terrain
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    if len(Rover.nav_angles) > 0:
                        steer = np.clip(Rover.nav_angles[int(len(Rover.nav_angles)/3)] * 180/np.pi, -15, 15)
                    else:
                        steer = 0
                    forward(Rover, Rover.vel,Rover.throttle_set,steer*0.9, Rover.can_go_forward)
                else:
                    Rover.mode = 'turn_around'
            else:
                Rover.mode = 'turn_around'
    except:
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = -15 # Could be more clever here about which way to turn


    return Rover
