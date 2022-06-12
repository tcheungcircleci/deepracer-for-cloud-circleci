import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # # Gives back indexes that lie between start and end index of a cyclical list 
        # # (start index is included, end index is not)
        # def indexes_cyclical(start, end, array_len):

        #     if end < start:
        #         end += array_len

        #     return [index % array_len for index in range(start, end)]

        # # Calculate how long car would take for entire lap, if it continued like it did until now
        # def projected_time(first_index, closest_index, step_count, times_list):

        #     # Calculate how much time has passed since start
        #     current_actual_time = (step_count-1) / 15

        #     # Calculate which indexes were already passed
        #     indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

        #     # Calculate how much time should have passed if car would have followed optimals
        #     current_expected_time = sum([times_list[i] for i in indexes_traveled])

        #     # Calculate how long one entire lap takes if car follows optimals
        #     total_expected_time = sum(times_list)

        #     # Calculate how long car would take for entire lap, if it continued like it did until now
        #     try:
        #         projected_time = (current_actual_time/current_expected_time) * total_expected_time
        #     except:
        #         projected_time = 9999

        #     return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the June 2022 Pro track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[8.53338, 3.14127, 3.0, 0.06329],
                        [8.34033, 3.15951, 3.0, 0.06463],
                        [8.14379, 3.17191, 3.0, 0.06565],
                        [7.94477, 3.18089, 3.0, 0.06641],
                        [7.74543, 3.18768, 3.0, 0.06648],
                        [7.54702, 3.19265, 3.0, 0.06616],
                        [7.349, 3.1962, 3.0, 0.06601],
                        [7.15117, 3.19862, 3.0, 0.06595],
                        [6.95339, 3.20015, 3.0, 0.06593],
                        [6.7556, 3.20098, 3.0, 0.06593],
                        [6.55778, 3.20125, 3.0, 0.06594],
                        [6.35993, 3.20114, 3.0, 0.06595],
                        [6.16204, 3.2008, 3.0, 0.06596],
                        [5.96413, 3.20029, 3.0, 0.06597],
                        [5.7662, 3.19968, 3.0, 0.06598],
                        [5.56823, 3.19891, 3.0, 0.06599],
                        [5.37022, 3.19798, 3.0, 0.066],
                        [5.17217, 3.19688, 3.0, 0.06602],
                        [4.97405, 3.19562, 3.0, 0.06604],
                        [4.77319, 3.19436, 3.0, 0.06696],
                        [4.5723, 3.19321, 3.0, 0.06696],
                        [4.37139, 3.19222, 3.0, 0.06697],
                        [4.17045, 3.1914, 3.0, 0.06698],
                        [3.9695, 3.19079, 3.0, 0.06699],
                        [3.76853, 3.1904, 3.0, 0.06699],
                        [3.56755, 3.19023, 3.0, 0.06699],
                        [3.36656, 3.19032, 3.0, 0.067],
                        [3.16557, 3.19081, 3.0, 0.067],
                        [2.96459, 3.19189, 3.0, 0.067],
                        [2.76363, 3.19367, 3.0, 0.06699],
                        [2.56273, 3.19639, 3.0, 0.06697],
                        [2.36192, 3.20026, 2.94145, 0.06828],
                        [2.16127, 3.20567, 2.40188, 0.08357],
                        [1.96089, 3.21303, 2.0598, 0.09735],
                        [1.7609, 3.22294, 1.79287, 0.11168],
                        [1.5649, 3.23586, 1.54969, 0.12675],
                        [1.37891, 3.24178, 1.54969, 0.12008],
                        [1.20067, 3.23606, 1.54969, 0.11507],
                        [1.03035, 3.2148, 1.54969, 0.11076],
                        [0.86944, 3.17458, 1.54969, 0.10703],
                        [0.72026, 3.11154, 1.54969, 0.10451],
                        [0.5874, 3.01976, 1.69987, 0.095],
                        [0.46992, 2.90447, 1.81362, 0.09076],
                        [0.36846, 2.76832, 1.91939, 0.08846],
                        [0.28462, 2.61416, 1.99741, 0.08786],
                        [0.22084, 2.44623, 1.88587, 0.09525],
                        [0.17945, 2.27026, 1.75752, 0.10285],
                        [0.16108, 2.09216, 1.61388, 0.11094],
                        [0.165, 1.91641, 1.61388, 0.10893],
                        [0.18971, 1.74592, 1.61388, 0.10674],
                        [0.23526, 1.58322, 1.61388, 0.10469],
                        [0.30254, 1.43099, 1.61388, 0.10313],
                        [0.39318, 1.29246, 1.61388, 0.10258],
                        [0.51035, 1.17254, 1.92754, 0.08698],
                        [0.64606, 1.06758, 2.16356, 0.07929],
                        [0.79691, 0.97603, 2.39804, 0.07359],
                        [0.96058, 0.89678, 2.72588, 0.06671],
                        [1.13458, 0.82818, 3.0, 0.06234],
                        [1.31677, 0.76857, 3.0, 0.0639],
                        [1.50582, 0.71688, 3.0, 0.06533],
                        [1.69997, 0.67243, 3.0, 0.06639],
                        [1.89587, 0.63484, 3.0, 0.06649],
                        [2.09246, 0.60321, 3.0, 0.06637],
                        [2.28942, 0.57667, 3.0, 0.06624],
                        [2.48656, 0.55442, 3.0, 0.06613],
                        [2.68378, 0.53586, 3.0, 0.06603],
                        [2.88099, 0.52053, 3.0, 0.06593],
                        [3.0781, 0.50803, 3.0, 0.06584],
                        [3.27504, 0.49797, 3.0, 0.06573],
                        [3.47168, 0.49012, 3.0, 0.0656],
                        [3.66772, 0.48435, 3.0, 0.06538],
                        [3.86251, 0.48063, 3.0, 0.06494],
                        [4.05505, 0.47899, 3.0, 0.06418],
                        [4.24527, 0.47966, 3.0, 0.06341],
                        [4.43358, 0.48299, 3.0, 0.06278],
                        [4.62026, 0.48946, 3.0, 0.06226],
                        [4.80542, 0.49965, 3.0, 0.06181],
                        [4.9891, 0.51427, 3.0, 0.06142],
                        [5.17122, 0.53446, 3.0, 0.06108],
                        [5.35167, 0.56158, 3.0, 0.06083],
                        [5.53019, 0.59811, 3.0, 0.06074],
                        [5.70782, 0.63928, 3.0, 0.06078],
                        [5.8861, 0.67713, 2.9111, 0.0626],
                        [6.06458, 0.71103, 2.58108, 0.07039],
                        [6.24319, 0.74016, 2.58108, 0.07011],
                        [6.4218, 0.76344, 2.58108, 0.06978],
                        [6.60024, 0.7793, 2.58108, 0.06941],
                        [6.77824, 0.78596, 2.58108, 0.06901],
                        [6.95541, 0.78131, 2.58108, 0.06867],
                        [7.13111, 0.76246, 2.85984, 0.06179],
                        [7.3053, 0.73211, 3.0, 0.05893],
                        [7.47795, 0.69185, 3.0, 0.0591],
                        [7.64892, 0.64196, 3.0, 0.05937],
                        [7.81818, 0.58354, 2.94473, 0.06081],
                        [7.98555, 0.51701, 2.66927, 0.06747],
                        [8.15053, 0.44263, 2.40401, 0.07528],
                        [8.31081, 0.37892, 2.16281, 0.07975],
                        [8.46993, 0.32402, 1.94649, 0.08647],
                        [8.62775, 0.27812, 1.72818, 0.09511],
                        [8.78391, 0.24219, 1.52983, 0.10474],
                        [8.93782, 0.21753, 1.34891, 0.11556],
                        [9.08873, 0.20571, 1.2, 0.12614],
                        [9.23563, 0.20845, 1.2, 0.12244],
                        [9.37722, 0.22764, 1.2, 0.11907],
                        [9.51155, 0.26604, 1.2, 0.11643],
                        [9.63565, 0.32737, 1.2, 0.11536],
                        [9.74428, 0.41752, 1.2, 0.11764],
                        [9.82597, 0.54545, 1.76643, 0.08593],
                        [9.89058, 0.69145, 1.98874, 0.08028],
                        [9.93872, 0.85225, 2.23207, 0.0752],
                        [9.97127, 1.02504, 2.48697, 0.0707],
                        [9.98922, 1.20743, 2.48567, 0.07373],
                        [9.9929, 1.39774, 2.32966, 0.0817],
                        [9.98305, 1.5941, 2.17009, 0.0906],
                        [9.959, 1.79339, 1.91364, 0.1049],
                        [9.92117, 1.98494, 1.69783, 0.115],
                        [9.86953, 2.16664, 1.69783, 0.11126],
                        [9.80375, 2.33713, 1.69783, 0.10763],
                        [9.72364, 2.49526, 1.69783, 0.10441],
                        [9.6287, 2.63963, 1.69783, 0.10177],
                        [9.51649, 2.76663, 1.69783, 0.09982],
                        [9.38415, 2.87124, 1.85763, 0.09081],
                        [9.23575, 2.9561, 2.06926, 0.08261],
                        [9.07443, 3.0237, 2.28704, 0.07648],
                        [8.90229, 3.07579, 2.56234, 0.07019],
                        [8.72132, 3.11427, 2.91841, 0.0634]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']
        is_reversed = params['is_reversed']

        """
        "all_wheels_on_track": Boolean,        # flag to indicate if the agent is on the track
        "x": float,                            # agent's x-coordinate in meters
        "y": float,                            # agent's y-coordinate in meters
        "closest_objects": [int, int],         # zero-based indices of the two closest objects to the agent's current position of (x, y).
        "closest_waypoints": [int, int],       # indices of the two nearest waypoints.
        "distance_from_center": float,         # distance in meters from the track center 
        "is_crashed": Boolean,                 # Boolean flag to indicate whether the agent has crashed.
        "is_left_of_center": Boolean,          # Flag to indicate if the agent is on the left side to the track center or not. 
        "is_offtrack": Boolean,                # Boolean flag to indicate whether the agent has gone off track.
        "is_reversed": Boolean,                # flag to indicate if the agent is driving clockwise (True) or counter clockwise (False).
        "heading": float,                      # agent's yaw in degrees
        "objects_distance": [float, ],         # list of the objects' distances in meters between 0 and track_length in relation to the starting line.
        "objects_heading": [float, ],          # list of the objects' headings in degrees between -180 and 180.
        "objects_left_of_center": [Boolean, ], # list of Boolean flags indicating whether elements' objects are left of the center (True) or not (False).
        "objects_location": [(float, float),], # list of object locations [(x,y), ...].
        "objects_speed": [float, ],            # list of the objects' speeds in meters per second.
        "progress": float,                     # percentage of track completed
        "speed": float,                        # agent's speed in meters per second (m/s)
        "steering_angle": float,               # agent's steering angle in degrees
        "steps": int,                          # number steps completed
        "track_length": float,                 # track length in meters.
        "track_width": float,                  # width of the track
        "waypoints": [(float, float), ]        # list of (x,y) as milestones along the track center
        """

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # # Save first racingpoint of episode for later
        # if self.verbose == True:
        #     self.first_racingpoint_index = 0 # this is just for testing purposes
        # if steps == 1:
        #     self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1.0
        SPEED_MULTIPLE = 1
        speed_diff = optimals[2]-speed
        if -1.0 < speed_diff <= 1.0:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = 1 - ( speed_diff )**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # # Reward if less steps
        # REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        # STANDARD_TIME = 37
        # FASTEST_TIME = 27
        # times_list = [row[3] for row in racing_track]
        # projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        # try:
        #     steps_prediction = projected_time * 15 + 1
        #     reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
        #                                    (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
        #     steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        # except:
        #     steps_reward = 0
        # reward += steps_reward

        # Reward based on how close car is to optimal heading
        direction_diff = racing_direction_diff( optimals[0:2], optimals_second[0:2], [x, y], heading )
        heading_reward = 0
        if direction_diff <= 10:
            heading_reward = 1.0
        elif 10 < direction_diff <= 30:
            heading_reward = (-1/20)*direction_diff + 1.5
        reward += heading_reward
            
        # Zero reward if obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 1.0:
            reward = 1e-3

        # progress reward
        progress_reward = 0
        if progress in [20,40,60,80,100]:
            progress_reward = progress/steps * 10
        reward += progress_reward
            
        # ## Incentive for finishing the lap in less steps ##
        # REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        # STANDARD_TIME = 37  # seconds (time that is easily done by model)
        # FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        # if progress == 100:
        #     finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
        #               (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        # else:
        #     finish_reward = 0
        # reward += finish_reward
        
        # Zero reward if obviously wrong direction (e.g. spin)
        if direction_diff > 30 or is_reversed == True:
            reward = 1e-3

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            # print("Direction difference: %f" % direction_diff)
            # print("Predicted time: %f" % projected_time)
            # print("=== Steps reward: %f ===" % steps_reward)
            # print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)



# add parameter verbose=True to get noisy output for testing
reward_object = Reward(
    # verbose=True
    )

def reward_function(params):
    return reward_object.reward_function(params)
