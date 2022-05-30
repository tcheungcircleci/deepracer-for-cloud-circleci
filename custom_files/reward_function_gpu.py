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

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [
            [5.05151, 0.86354, 4.0, 0.07544],
            [5.05149, 1.16529, 4.0, 0.07544],
            [5.05144, 1.46704, 3.30188, 0.09139],
            [5.05144, 1.7688, 1.9977, 0.15105],
            [5.05157, 2.07055, 1.65646, 0.18217],
            [5.0514, 2.3723, 1.52786, 0.1975],
            [5.0514, 2.67405, 1.5008, 0.20106],
            [5.05182, 2.97582, 1.5008, 0.20107],
            [5.04218, 3.27569, 1.5008, 0.19991],
            [5.00573, 3.57052, 1.5008, 0.19795],
            [4.93186, 3.85515, 1.5008, 0.19594],
            [4.81616, 4.12382, 1.5008, 0.19491],
            [4.65939, 4.37129, 1.52327, 0.19231],
            [4.46528, 4.59323, 1.57213, 0.18755],
            [4.23913, 4.78625, 1.63301, 0.18207],
            [3.98705, 4.94801, 1.69403, 0.17681],
            [3.71532, 5.07707, 1.77792, 0.1692],
            [3.43023, 5.174, 1.90083, 0.15841],
            [3.13691, 5.24159, 2.08805, 0.14415],
            [2.83904, 5.28436, 2.38581, 0.12613],
            [2.53888, 5.30794, 2.86371, 0.10514],
            [2.23766, 5.31812, 3.80644, 0.07918],
            [1.93603, 5.32068, 4.0, 0.07541],
            [1.63429, 5.321, 4.0, 0.07544],
            [1.33253, 5.32102, 4.0, 0.07544],
            [1.03078, 5.32108, 4.0, 0.07544],
            [0.72903, 5.32112, 4.0, 0.07544],
            [0.42728, 5.32116, 2.85931, 0.10553],
            [0.12553, 5.32121, 2.39952, 0.12576],
            [-0.17622, 5.32124, 2.21424, 0.13628],
            [-0.47798, 5.32133, 2.14807, 0.14048],
            [-0.77973, 5.32135, 2.11629, 0.14258],
            [-1.03846, 5.31755, 2.10422, 0.12297],
            [-1.26231, 5.30622, 2.08863, 0.10731],
            [-1.48637, 5.2842, 2.07289, 0.10862],
            [-1.72664, 5.24641, 2.03095, 0.11976],
            [-1.98706, 5.18732, 1.91476, 0.13946],
            [-2.26085, 5.10316, 1.74287, 0.16435],
            [-2.53685, 4.99349, 1.64411, 0.18064],
            [-2.8061, 4.8596, 1.60157, 0.18776],
            [-3.06361, 4.70293, 1.56025, 0.19319],
            [-3.30636, 4.52394, 1.50867, 0.19992],
            [-3.52984, 4.3218, 1.45084, 0.2077],
            [-3.72681, 4.09501, 1.36401, 0.22023],
            [-3.89073, 3.84506, 1.32039, 0.22638],
            [-4.01729, 3.57556, 1.31156, 0.22701],
            [-4.10253, 3.29035, 1.31156, 0.22696],
            [-4.1416, 2.99386, 1.31156, 0.22801],
            [-4.12891, 2.69335, 1.31156, 0.22933],
            [-4.0584, 2.40301, 1.31156, 0.22781],
            [-3.93116, 2.13654, 1.31156, 0.22515],
            [-3.75291, 1.9038, 1.34199, 0.21845],
            [-3.53223, 1.71059, 1.41748, 0.20692],
            [-3.27918, 1.55855, 1.53239, 0.19265],
            [-3.00355, 1.44604, 1.70712, 0.1744],
            [-2.71412, 1.36812, 1.96382, 0.15263],
            [-2.41746, 1.31769, 2.34953, 0.12808],
            [-2.11777, 1.28697, 3.01961, 0.09977],
            [-1.81678, 1.26829, 2.08421, 0.14469],
            [-1.51536, 1.25527, 1.72832, 0.17457],
            [-1.21383, 1.24642, 1.56446, 0.19282],
            [-0.91223, 1.24042, 1.47426, 0.20462],
            [-0.61058, 1.23622, 1.41285, 0.21352],
            [-0.31042, 1.22275, 1.38774, 0.21651],
            [-0.01452, 1.1845, 1.37278, 0.21734],
            [0.27241, 1.11139, 1.37278, 0.21569],
            [0.54425, 0.99767, 1.37278, 0.21465],
            [0.79373, 0.84091, 1.37278, 0.21463],
            [1.01204, 0.64121, 1.37278, 0.21552],
            [1.19035, 0.40297, 1.34133, 0.22186],
            [1.32026, 0.1341, 1.30774, 0.22834],
            [1.39614, -0.15374, 1.30774, 0.22763],
            [1.41603, -0.44912, 1.30774, 0.22638],
            [1.38004, -0.74357, 1.30774, 0.22684],
            [1.28773, -1.0282, 1.30774, 0.22881],
            [1.13963, -1.28915, 1.30774, 0.22944],
            [0.94088, -1.51164, 1.3199, 0.22603],
            [0.70247, -1.68701, 1.37811, 0.21476],
            [0.43639, -1.81391, 1.50179, 0.19629],
            [0.15294, -1.89725, 1.71131, 0.17265],
            [-0.14049, -1.94549, 2.04357, 0.14551],
            [-0.43913, -1.96827, 2.6637, 0.11244],
            [-0.74016, -1.97572, 4.0, 0.07528],
            [-1.04191, -1.97743, 4.0, 0.07544],
            [-1.34366, -1.9793, 4.0, 0.07544],
            [-1.6454, -1.98113, 2.43239, 0.12406],
            [-1.94715, -1.98298, 1.80396, 0.16727],
            [-2.2489, -1.98481, 1.57036, 0.19215],
            [-2.55064, -1.98667, 1.46401, 0.20611],
            [-2.85239, -1.98844, 1.42016, 0.21248],
            [-3.15412, -1.99077, 1.40642, 0.21454],
            [-3.45221, -2.01148, 1.40642, 0.21246],
            [-3.74241, -2.06436, 1.40642, 0.20974],
            [-4.0189, -2.15747, 1.40642, 0.20744],
            [-4.27532, -2.29425, 1.40642, 0.20663],
            [-4.50489, -2.47449, 1.40642, 0.20753],
            [-4.70032, -2.69514, 1.41289, 0.20862],
            [-4.85447, -2.94976, 1.34572, 0.22118],
            [-4.96171, -3.22878, 1.3, 0.22994],
            [-5.01882, -3.52185, 1.3, 0.22968],
            [-5.02567, -3.81982, 1.3, 0.22927],
            [-4.98218, -4.11661, 1.3, 0.23074],
            [-4.88831, -4.40273, 1.3, 0.23163],
            [-4.74001, -4.66257, 1.3, 0.23014],
            [-4.542, -4.88254, 1.30722, 0.22641],
            [-4.30448, -5.05502, 1.36258, 0.21543],
            [-4.03873, -5.17879, 1.46978, 0.19946],
            [-3.75471, -5.25752, 1.66813, 0.17668],
            [-3.46038, -5.29934, 2.05589, 0.1446],
            [-3.16122, -5.31592, 2.83321, 0.10575],
            [-2.86019, -5.31891, 4.0, 0.07526],
            [-2.55845, -5.31861, 4.0, 0.07544],
            [-2.2567, -5.31833, 4.0, 0.07544],
            [-1.95496, -5.31789, 4.0, 0.07544],
            [-1.6532, -5.31725, 4.0, 0.07544],
            [-1.35145, -5.3167, 4.0, 0.07544],
            [-1.0497, -5.31614, 4.0, 0.07544],
            [-0.74795, -5.31556, 4.0, 0.07544],
            [-0.4462, -5.31499, 4.0, 0.07544],
            [-0.14445, -5.31441, 4.0, 0.07544],
            [0.1573, -5.31384, 4.0, 0.07544],
            [0.45906, -5.31327, 4.0, 0.07544],
            [0.76081, -5.3127, 4.0, 0.07544],
            [1.06256, -5.31212, 4.0, 0.07544],
            [1.36431, -5.31153, 2.49695, 0.12085],
            [1.66606, -5.31104, 2.03046, 0.14861],
            [1.96781, -5.31033, 1.81386, 0.16636],
            [2.26955, -5.30933, 1.65225, 0.18262],
            [2.5713, -5.30891, 1.51783, 0.19881],
            [2.8731, -5.3115, 1.43598, 0.21018],
            [3.17341, -5.29643, 1.40856, 0.21347],
            [3.46974, -5.25512, 1.40856, 0.21241],
            [3.7586, -5.18174, 1.40856, 0.21159],
            [4.03495, -5.07091, 1.40856, 0.21138],
            [4.29097, -4.91791, 1.40856, 0.21175],
            [4.5166, -4.72213, 1.40856, 0.21208],
            [4.70257, -4.4889, 1.442, 0.20686],
            [4.84485, -4.22783, 1.52109, 0.19547],
            [4.94469, -3.9481, 1.65606, 0.17935],
            [5.00707, -3.65692, 1.8763, 0.15871],
            [5.03933, -3.35943, 2.26623, 0.13204],
            [5.05047, -3.05896, 3.25163, 0.09247],
            [5.05122, -2.75747, 4.0, 0.07537],
            [5.05137, -2.45572, 4.0, 0.07544],
            [5.05171, -2.15398, 4.0, 0.07544],
            [5.05175, -1.85223, 4.0, 0.07544],
            [5.05166, -1.55047, 4.0, 0.07544],
            [5.05166, -1.24872, 4.0, 0.07544],
            [5.05165, -0.94697, 4.0, 0.07544],
            [5.05162, -0.64522, 4.0, 0.07544],
            [5.0516, -0.34347, 4.0, 0.07544],
            [5.05157, -0.04171, 4.0, 0.07544],
            [5.05155, 0.26004, 4.0, 0.07544],
            [5.05153, 0.56179, 4.0, 0.07544]
            ]

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

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
