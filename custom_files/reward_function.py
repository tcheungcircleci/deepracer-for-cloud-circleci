import math

def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']

    # Give higher reward if the car is closer to center line and vice versa
    # +1 at center, -1 at edge
    reward = math.cos( 2 * (distance_from_center/track_width) * math.pi)

    # Higher reward for higher speed
    reward += (speed/4)-0.25

    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 15

    # Penalize reward if the car is steering too much
    if steering > ABS_STEERING_THRESHOLD:
        reward -= 0.5

    if progress in [25,50,75,100]:
        reward += progress

    if not all_wheels_on_track:
        reward = -100

    return float(reward)
