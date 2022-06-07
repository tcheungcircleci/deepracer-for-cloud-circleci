import math

class PARAMS:
    prev_speed = None
    prev_steering_angle = None 
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None
def reward_function(params):
    
    # Read input parameters
    heading = params['heading']
    distance_from_center = params['distance_from_center']
    steps = params['steps']
    steering_angle = params['steering_angle']
    speed = params['speed']
# Reinitialize previous parameters if it is a new episode
    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None
        PARAMS.prev_direction_diff = None
        PARAMS.prev_normalized_distance_from_route = None
    #Check if the speed has dropped
    has_speed_dropped = False
    if PARAMS.prev_speed is not None:
        if PARAMS.prev_speed > speed:
            has_speed_dropped = True
    #Penalize slowing down without good reason on straight portions
    if has_speed_dropped and not is_turn_upcoming: 
        speed_maintain_bonus = min( speed / PARAMS.prev_speed, 1 )
    #Penalize making the heading direction worse
    heading_decrease_bonus = 0
    if PARAMS.prev_direction_diff is not None:
        if is_heading_in_right_direction:
            if abs( PARAMS.prev_direction_diff / direction_diff ) > 1:
                heading_decrease_bonus = min(10, abs( PARAMS.prev_direction_diff / direction_diff ))
    #has the steering angle changed
    has_steering_angle_changed = False
    if PARAMS.prev_steering_angle is not None:
        if not(math.isclose(PARAMS.prev_steering_angle,steering_angle)):
            has_steering_angle_changed = True
    steering_angle_maintain_bonus = 1 
    #Not changing the steering angle is a good thing if heading in the right direction
    if is_heading_in_right_direction and not has_steering_angle_changed:
        if abs(direction_diff) < 10:
            steering_angle_maintain_bonus *= 2
        if abs(direction_diff) < 5:
            steering_angle_maintain_bonus *= 2
        if PARAMS.prev_direction_diff is not None and abs(PARAMS.prev_direction_diff) > abs(direction_diff):
            steering_angle_maintain_bonus *= 2
    #Reward reducing distance to the race line
    distance_reduction_bonus = 1
    if PARAMS.prev_normalized_distance_from_route is not None and PARAMS.prev_normalized_distance_from_route > normalized_distance_from_route:
        if abs(normalized_distance_from_route) > 0:
            distance_reduction_bonus = min( abs( PARAMS.prev_normalized_distance_from_route / normalized_distance_from_route ), 2)
    # Before returning reward, update the variables
    PARAMS.prev_speed = speed
    PARAMS.prev_steering_angle = steering_angle
    PARAMS.prev_direction_diff = direction_diff
    PARAMS.prev_steps = steps
    PARAMS.prev_normalized_distance_from_route = normalized_distance_from_route


    #Calculate the speed reward
    MIN_SPEED = 2.0
    MAX_SPEED = 4.0
    optimal_speed = 0
    #chosen such that 6 standard deviations covers the entire range
    sigma_speed = abs(MAX_SPEED - MIN_SPEED)/6.0
    i = closest_route_point
    nsteps = 5
    if i+nsteps < len(PARAMS.waypoints):
        optimal_speed = min( PARAMS.optimal_speed[i:(i+nsteps)%len(PARAMS.waypoints)] )
    else:
        optimal_speed = min( min(PARAMS.optimal_speed[i:]), min(PARAMS.optimal_speed[:(i+nsteps)%len(PARAMS.waypoints)+1]) )
        
    #PARAMS.optimal speed is unbounded and hence must be bounded from above to reflect action space realities
    optimal_speed = min(MAX_SPEED,optimal_speed)
    speed_reward = math.exp(-0.5*abs(speed-optimal_speed)**2 / sigma_speed**2)


    #distance reward is value of the standard normal scaled back to 1. #Hence the 1/2*pi*sigma term is cancelled out
    distance_reward = 0
    if "center" in bearing: #i.e. on the route line
        distance_from_route = 0
        distance_reward = 1
    elif "right" in bearing: #i.e. on right side of the route line
        sigma=abs(normalized_route_distance_from_inner_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
    elif "left" in bearing: #i.e. on left side of the route line
        sigma=abs(normalized_route_distance_from_outer_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)


    heading = params['heading']
    vehicle_x = params['x']
    vehicle_y = params['y']
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians between target and current vehicle position
    route_direction = math.atan2(next_route_point_y - vehicle_y, next_route_point_x - vehicle_x) 
    # Convert to degree
    route_direction = math.degrees(route_direction)
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = route_direction - heading
    #Check that the direction_diff is in valid range
    #Then compute the heading reward
    heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 10
    if abs(direction_diff) <= 20:
        heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 4


    # Reward for making steady progress
    progress_reward = 10 * progress / steps
    if steps <= 5:
        progress_reward = 1 #ignore progress in the first 5 steps
    # Bonus that the agent gets for completing every 10 percent of track
    # Is exponential in the progress / steps. 
    # exponent increases with an increase in fraction of lap completed
    intermediate_progress_bonus = 0
    pi = int(progress//10)
    if pi != 0 and PARAMS.intermediate_progress[ pi ] == 0:
        if pi==10: # 100% track completion
            intermediate_progress_bonus = progress_reward ** 14
        else:
            intermediate_progress_bonus = progress_reward ** (5+0.75*pi)
    PARAMS.intermediate_progress[ pi ] = intermediate_progress_bonus




    #heading component of reward
    HC = ( 10 * heading_reward * steering_angle_maintain_bonus )
    #distance component of reward
    DC = ( 10 * distance_reward * distance_reduction_bonus )
    #speed component of reward
    SC = ( 5 * speed_reward * speed_maintain_bonus )
    #Immediate component of reward
    IC = ( HC + DC + SC ) ** 2 + ( HC * DC * SC ) 
    #If an unpardonable action is taken, then the immediate reward is 0
    if PARAMS.unpardonable_action:
        IC = 1e-3
    #Long term component of reward
    LC = ( curve_bonus + intermediate_progress_bonus + straight_section_bonus )
    return max(IC + LC,1e-3)