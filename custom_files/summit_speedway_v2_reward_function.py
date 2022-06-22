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
        racing_track = [[8.49719, 3.08704, 2.94134, 0.06168],
                        [8.31474, 3.12087, 3.27119, 0.05672],
                        [8.12722, 3.1475, 3.70449, 0.05113],
                        [7.93558, 3.16819, 4.0, 0.04819],
                        [7.7406, 3.18389, 4.0, 0.0489],
                        [7.54329, 3.19542, 4.0, 0.04941],
                        [7.34536, 3.20354, 4.0, 0.04952],
                        [7.14791, 3.20891, 4.0, 0.04938],
                        [6.95082, 3.21211, 4.0, 0.04928],
                        [6.75388, 3.21362, 4.0, 0.04924],
                        [6.55695, 3.21382, 4.0, 0.04923],
                        [6.35992, 3.21302, 4.0, 0.04926],
                        [6.16278, 3.2115, 4.0, 0.04929],
                        [5.96553, 3.20953, 4.0, 0.04932],
                        [5.7682, 3.20733, 4.0, 0.04933],
                        [5.5708, 3.20494, 4.0, 0.04935],
                        [5.37184, 3.20252, 4.0, 0.04975],
                        [5.17209, 3.20027, 4.0, 0.04994],
                        [4.97193, 3.19819, 4.0, 0.05004],
                        [4.77154, 3.19628, 4.0, 0.0501],
                        [4.57103, 3.1945, 4.0, 0.05013],
                        [4.3704, 3.19293, 4.0, 0.05016],
                        [4.1697, 3.19155, 4.0, 0.05018],
                        [3.96894, 3.1904, 4.0, 0.05019],
                        [3.76814, 3.18946, 4.0, 0.0502],
                        [3.5673, 3.18876, 4.0, 0.05021],
                        [3.36641, 3.18844, 4.0, 0.05022],
                        [3.16547, 3.18863, 4.0, 0.05023],
                        [2.96449, 3.18948, 4.0, 0.05024],
                        [2.7635, 3.19121, 3.48156, 0.05773],
                        [2.56255, 3.19408, 2.95241, 0.06807],
                        [2.36172, 3.19839, 2.57501, 0.07801],
                        [2.16111, 3.20454, 2.25244, 0.0891],
                        [1.96692, 3.20825, 2.0236, 0.09598],
                        [1.77765, 3.20732, 1.7332, 0.10921],
                        [1.5943, 3.19961, 1.41369, 0.1298],
                        [1.41753, 3.18331, 1.41369, 0.12557],
                        [1.24795, 3.15673, 1.41369, 0.12142],
                        [1.0865, 3.11796, 1.41369, 0.11745],
                        [0.9341, 3.0654, 1.41369, 0.11403],
                        [0.79326, 2.99564, 1.41369, 0.11118],
                        [0.67012, 2.90238, 1.51275, 0.10211],
                        [0.56407, 2.7908, 1.58386, 0.09719],
                        [0.47559, 2.66471, 1.66178, 0.09269],
                        [0.40481, 2.52786, 1.68149, 0.09162],
                        [0.35226, 2.38333, 1.56479, 0.09828],
                        [0.31798, 2.23399, 1.41865, 0.10801],
                        [0.30177, 2.0823, 1.41865, 0.10754],
                        [0.30376, 1.93037, 1.41865, 0.10711],
                        [0.32412, 1.78018, 1.41865, 0.10683],
                        [0.36369, 1.63383, 1.41865, 0.10687],
                        [0.42502, 1.49422, 1.41865, 0.10749],
                        [0.51231, 1.36618, 1.56961, 0.09873],
                        [0.62068, 1.25029, 1.73807, 0.09129],
                        [0.74662, 1.14629, 1.925, 0.08485],
                        [0.88736, 1.05362, 2.13097, 0.07908],
                        [1.04054, 0.97163, 2.36335, 0.07351],
                        [1.20396, 0.89957, 2.61881, 0.0682],
                        [1.37558, 0.83657, 2.89113, 0.06324],
                        [1.55363, 0.78177, 3.17262, 0.05872],
                        [1.73659, 0.73434, 3.45651, 0.05468],
                        [1.92318, 0.69354, 3.78948, 0.0504],
                        [2.11232, 0.65852, 4.0, 0.04809],
                        [2.30323, 0.62873, 4.0, 0.04831],
                        [2.49529, 0.6037, 4.0, 0.04842],
                        [2.68798, 0.58303, 4.0, 0.04845],
                        [2.88089, 0.56649, 4.0, 0.0484],
                        [3.07368, 0.55364, 4.0, 0.0483],
                        [3.26612, 0.54424, 4.0, 0.04817],
                        [3.45803, 0.53807, 4.0, 0.048],
                        [3.6493, 0.53489, 4.0, 0.04782],
                        [3.83987, 0.53446, 4.0, 0.04764],
                        [4.0297, 0.53663, 4.0, 0.04746],
                        [4.21876, 0.54124, 4.0, 0.04728],
                        [4.40707, 0.5481, 4.0, 0.04711],
                        [4.59466, 0.55701, 4.0, 0.04695],
                        [4.78161, 0.56774, 4.0, 0.04681],
                        [4.96799, 0.58001, 4.0, 0.0467],
                        [5.15392, 0.59347, 4.0, 0.04661],
                        [5.33951, 0.60786, 4.0, 0.04654],
                        [5.52002, 0.62255, 4.0, 0.04528],
                        [5.69979, 0.63576, 4.0, 0.04506],
                        [5.87878, 0.64696, 3.8055, 0.04713],
                        [6.05702, 0.65561, 3.44947, 0.05173],
                        [6.23464, 0.6615, 3.44947, 0.05152],
                        [6.41167, 0.6642, 3.44947, 0.05133],
                        [6.58812, 0.66328, 3.44947, 0.05115],
                        [6.76395, 0.65818, 3.44947, 0.05099],
                        [6.93901, 0.64798, 3.44947, 0.05084],
                        [7.11312, 0.63162, 3.84234, 0.04551],
                        [7.2864, 0.61033, 4.0, 0.04365],
                        [7.45889, 0.58492, 4.0, 0.04359],
                        [7.63046, 0.55619, 4.0, 0.04349],
                        [7.80076, 0.52477, 3.19508, 0.0542],
                        [7.96924, 0.49132, 2.65299, 0.06475],
                        [8.13529, 0.4564, 2.29283, 0.074],
                        [8.29845, 0.42057, 2.00118, 0.08347],
                        [8.45101, 0.38814, 1.75631, 0.08881],
                        [8.60247, 0.35916, 1.52778, 0.10093],
                        [8.75198, 0.3361, 1.34067, 0.11284],
                        [8.89882, 0.3211, 1.18476, 0.12458],
                        [9.04223, 0.31607, 1.1, 0.13046],
                        [9.18133, 0.32304, 1.1, 0.12661],
                        [9.31494, 0.34425, 1.1, 0.12298],
                        [9.44121, 0.3827, 1.1, 0.12],
                        [9.55743, 0.442, 1.1, 0.11862],
                        [9.65896, 0.5272, 1.1, 0.12049],
                        [9.73833, 0.64332, 1.54165, 0.09123],
                        [9.8029, 0.77584, 1.68236, 0.08763],
                        [9.85227, 0.92255, 1.83618, 0.0843],
                        [9.88621, 1.08082, 1.98213, 0.08167],
                        [9.90465, 1.24767, 2.09798, 0.08001],
                        [9.90757, 1.41982, 2.06368, 0.08343],
                        [9.89551, 1.5939, 1.93902, 0.08999],
                        [9.86911, 1.76703, 1.77359, 0.09874],
                        [9.82921, 1.93691, 1.59946, 0.1091],
                        [9.77574, 2.10137, 1.59946, 0.10812],
                        [9.70881, 2.25844, 1.59946, 0.10674],
                        [9.62832, 2.40619, 1.59946, 0.10519],
                        [9.53383, 2.54255, 1.59946, 0.10372],
                        [9.42403, 2.66461, 1.59946, 0.10265],
                        [9.29679, 2.76822, 1.78799, 0.09177],
                        [9.15607, 2.85623, 1.97886, 0.08387],
                        [9.00422, 2.93064, 2.19209, 0.07714],
                        [8.84302, 2.99314, 2.39845, 0.07209],
                        [8.67364, 3.04489, 2.63985, 0.06709]]

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

        # ## Reward if speed is close to optimal speed ##
        # SPEED_DIFF_NO_REWARD = 1.0
        # SPEED_MULTIPLE = 1
        # speed_diff = optimals[2]-speed
        # if -1.0 < speed_diff <= 1.0:
        #     # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
        #     # so, we do not punish small deviations from optimal speed
        #     speed_reward = 1 - ( speed_diff )**2
        # else:
        #     speed_reward = 0
        # reward += speed_reward * SPEED_MULTIPLE

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
            
        # # Zero reward if obviously too slow
        # speed_diff_zero = optimals[2]-speed
        # if speed_diff_zero > 1.0:
        #     reward = 1e-3

        # # progress reward
        # progress_reward = 0
        # if progress in [20,40,60,80,100]:
        #     progress_reward = progress/steps * 10
        # reward += progress_reward

        if progress == 25 and steps <= 74: #53:
            progress_25_reward = progress/steps * 74 * 5
            reward += progress_25_reward

        if progress == 50 and steps <= 134: #103:
            progress_50_reward = progress/steps * 134 * 5
            reward += progress_50_reward

        if progress == 75 and steps <= 172: #151:
            progress_75_reward = progress/steps * 172 * 5
            reward += progress_75_reward

        if progress == 100 and steps <= 221: #201:
            progress_100_reward = progress/steps * 221 * 10
            reward += progress_100_reward
            
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
            # print("Speed difference: %f" % speed_diff)
            # print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
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
