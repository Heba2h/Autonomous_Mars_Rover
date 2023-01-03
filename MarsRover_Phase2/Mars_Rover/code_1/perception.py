import numpy as np
import cv2
# from scipy import curve_fit
# from scipy.optimize import curve_fit

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

#trying to implement curve fit region (not complete)
    #     new_x,new_y=cv2.approxPolyDP()
    #     cv2.approxPolyDP(count, epsilon, True)
    #     cv2.drawContours(imageread, [approximations], 0, (0), 3)
    # #the name of the detected shapes are written on the image
    #     new_x, new_y = approximations[0][0]

    # load input variables from a file
    # x_values =x_pixel 
    # y_values = y_pixel
    # fit curve
    #popt, _ = curve_fit(objective, x_values, y_values)
    # define new input values
    #x_new = arange(min(x), max(x), 1)
    # unpack optima parameters for the objective function
    #a, b, c = popt
    # use optimal parameters to calculate new values
    #y_new = objective(x_new, a, b, c)
#end region

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)

    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask=cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped ,mask
    
def find_rocks(img,rgb_thresh=(110,110,50)):
    rock_pixel = np.zeros_like(img[:,:,0])
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    rock_pixel[above_thresh] = 1
    return rock_pixel
def rover_and_curve_par(threshed):
    global Rover
    threshed=threshed[35:,:]   
    xpix, ypix = rover_coords(threshed)
    x_rover=xpix.astype(int)
    y_rover=ypix.astype(int)
    yaw=0
    try:
        coefficients = np.polyfit(x_rover, y_rover, 2)
        f_x=np.poly1d(coefficients)
        # print( f_x)

        x_new = np.linspace(min(x_rover), max(x_rover), len(x_rover))
        y_pred=f_x(x_new)

        points,angles=to_polar_coords(x_new,y_pred)
    except:
        x_new=[]
        y_pred=[]
        angles=-15   
        print("saaaaaaaaaaaaaaaaaaaaaaaad")
    # print("-----------------------------------")
    # print(len(angles))
    # print("-----------------------------------")
    # dist, angles = to_polar_coords(xpix,ypix)
    # mean_dir = angles[2]
    return angles
def nearst_distance(myangles,angle):
    minimum=100000 
    index=-1
    for my_angle in myangles:
            if(np.abs(my_angle-angle)<=minimum):
                    minimum=np.abs(my_angle-angle)
                    index=myangles.index(my_angle)
    return index,minimum


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    dst_size = 8
    bottom_offset = 6
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped ,mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed=color_thresh(warped)
    obs_map=np.absolute(np.float32((threshed)-1))*mask 
    rock_map=find_rocks(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0]=obs_map *255

    Rover.vision_image[:,:,2]=threshed*255
    free_pixel=Rover.vision_image[:,:,2]>0
    Rover.vision_image[free_pixel,0]=0
        
        

    # 5) Convert map image pixel values to rover-centric coords
    x_pixel,y_pixel=rover_coords(threshed)
    obs_x_pixel,obs_y_pixel=rover_coords(obs_map)
    world_size=Rover.worldmap.shape[0]
    scale=2*dst_size

    x_pos=Rover.pos[0]

    y_pos=Rover.pos[1]
    
    yaw=Rover.yaw

    
            ################ heba check here #####################
            # dist, angles = to_polar_coords(xpix_navigable, ypix_navigable)
            # Rover.nav_dists = dist
            # Rover.nav_angles = angles
    
    # For can_go_forward flag calculations 
    
    Rover.nav_dists,Rover.nav_angles= to_polar_coords(x_pixel, y_pixel)
    Rover.nav_angles=rover_and_curve_par(threshed)



    nav_mean_dist = np.mean(Rover.nav_dists)
    # nav_mean_angle = np.mean(Rover.nav_angles)
    try:
         nav_mean_angle = np.clip(Rover.nav_angles[int(len(Rover.nav_angles)/3)] * 180/np.pi, -15, 15)
    except:
        nav_mean_angle=-15

    
    Rover.can_go_forward = (nav_mean_angle > -1 * Rover.angle_forward) and (nav_mean_angle < Rover.angle_forward) and (nav_mean_dist > Rover.mim_wall_distance)
    
    print ("dist:",nav_mean_dist)
    print ("angle mean:",nav_mean_angle)


    if rock_map.any():
        
        rock_x_pixel,rock_y_pixel=rover_coords(rock_map)
        rock_dis,rock_ang=to_polar_coords(rock_x_pixel, rock_y_pixel)
        rock_idx=np.argmin(rock_dis)
        rock_x_world, rock_y_world=pix_to_world(rock_x_pixel[rock_idx],rock_y_pixel[rock_idx],x_pos,y_pos,yaw,world_size,scale)
 
        Rover.sample_dists = rock_dis
        Rover.sample_angles = rock_ang
        Rover.sample_detected = True
        Rover.mode = 'sample'
        Rover.turn_dir = 'none'
            
    elif Rover.can_go_forward:
                # lost the sample and no objects in the path
                Rover.sample_detected = False
                Rover.mode = 'forward'
    else:
                # lost sample and there are objects / wall in the way
                Rover.vision_image[:,:,1]=0
                Rover.sample_detected = False
                Rover.mode = 'turn_around'

    # print (Rover.sample_dists)
    


    # 6) Convert rover-centric pixel values to world coordinates
    navigable_x_world,navigable_y_world,=pix_to_world(x_pixel,y_pixel,x_pos,y_pos,yaw,world_size,scale)
    obstacle_x_world, obstacle_y_world=pix_to_world(obs_x_pixel,obs_y_pixel,x_pos,y_pos,yaw,world_size,scale)

    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if(Rover.pitch<0.3 or Rover.pitch>359.7) and (Rover.roll<0.3 or Rover.roll>359.7):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] = 255
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] =255
        if rock_map.any():
            Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
        

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    #if not rock_map.any():

    kernel=np.ones((3,3),np.uint8)
    opening=cv2.morphologyEx(threshed, cv2.MORPH_OPEN, kernel)
    closing=cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    dialation=cv2.dilate(closing,kernel,iterations=1)
    boundry=dialation-closing
    
    erosion_mask=cv2.erode(mask,kernel,iterations=1)
    point_cloud=np.logical_and(boundry,erosion_mask)
    point_cloud=point_cloud[100:,:]
    point_cloud=point_cloud.astype(int)
    point_cloud = np.argwhere(point_cloud==1)
    
    my_distance=[]
    wanted_distance=[]
    my_angles=[]
    
    threshold_of_testing=0.01
    # print(len(cropped))
    try:
        for x in point_cloud:
            
                my_distance.append(np.sqrt(x[0]*x[0]+x[1]*x[1]))
                my_angles.append(np.arctan2(x[1],x[0]))

    # print(max(my_angles),min(my_angles))
    
        angles_list=np.arange(start=min(my_angles),stop=max(my_angles),step=(max(my_angles) - min(my_angles))/ len(my_distance))
        for angle in angles_list:
            index,diff=nearst_distance(my_angles,angle)
            if(index==-1):
                wanted_distance.append(35)
            elif(diff<threshold_of_testing):

                    # wanted_distance.append(distance[index])
                    wanted_distance.append(my_distance.pop(index)/50)
                    my_angles.pop(index)
            else:
                    wanted_distance.append(35)
    except:
        wanted_distance=[]

        wanted_distance=np.array(wanted_distance)
    
    #################################################
    # if Rover.start_pos is None:
    #     Rover.start_pos = (Rover.pos[0], Rover.pos[1])
    #     print('STARTING POSITION IS: ', Rover.start_pos)
    ########################################################
    return Rover,point_cloud,wanted_distance