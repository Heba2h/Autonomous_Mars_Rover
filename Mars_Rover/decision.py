import numpy as np

class PID:
    
    def __init__(self,kp,ki,kd) :

        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.eSum=0
        self.e=0

    def get_longitudinal_control(self,currentState,desired_speed,dt):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        currentState: np.array of floats, shape=6
        Current State Data    [x, y, theta, speed, beta (slip angle), theta_dot]
        
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
        v_current=currentState[3]
        e = desired_speed - v_current 
        self.eSum = self.eSum + e  
        dedt=(e-e_past)/dt 
        action = self.kp*e + self.ki * self.eSum + self.kd * dedt
        throttle_output = np.tanh(action)
        e_past=e
        return throttle_output
        
pid=PID(10,0.7,0.01)


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    longitudunal_Control = pid.get_longitudinal_control(np.array([Rover.pos[0],Rover.pos[1],Rover.nav_angles,Rover.vel]),Rover.max_vel,(1/25))
                    Rover.throttle = longitudunal_Control
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = longitudunal_Control
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = longitudunal_Control
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

