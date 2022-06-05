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
        racing_track = [[3.14664, 0.94321, 4.0, 0.07523],
                        [2.86674, 0.83269, 4.0, 0.07523],
                        [2.58673, 0.72245, 4.0, 0.07523],
                        [2.30663, 0.61243, 4.0, 0.07523],
                        [2.02657, 0.50231, 4.0, 0.07523],
                        [1.74659, 0.392, 4.0, 0.07523],
                        [1.46672, 0.28141, 4.0, 0.07523],
                        [1.18696, 0.17053, 4.0, 0.07523],
                        [0.90731, 0.05938, 4.0, 0.07523],
                        [0.62781, -0.05217, 4.0, 0.07523],
                        [0.34881, -0.1649, 4.0, 0.07523],
                        [0.07075, -0.27989, 4.0, 0.07522],
                        [-0.20605, -0.39775, 4.0, 0.07521],
                        [-0.48153, -0.51855, 4.0, 0.0752],
                        [-0.75587, -0.64189, 4.0, 0.0752],
                        [-1.02931, -0.76721, 3.69046, 0.08151],
                        [-1.30195, -0.89411, 2.36437, 0.12719],
                        [-1.57687, -1.01586, 1.84172, 0.16325],
                        [-1.85505, -1.12992, 1.6641, 0.18067],
                        [-2.13728, -1.23378, 1.63235, 0.18423],
                        [-2.42405, -1.32471, 1.63235, 0.1843],
                        [-2.71543, -1.39982, 1.63235, 0.18434],
                        [-3.00718, -1.43505, 1.63235, 0.18003],
                        [-3.28362, -1.41032, 1.63235, 0.17003],
                        [-3.5336, -1.32396, 1.63235, 0.16202],
                        [-3.7518, -1.18172, 1.68685, 0.15441],
                        [-3.93514, -0.99013, 1.79938, 0.14737],
                        [-4.08075, -0.75565, 1.95288, 0.14133],
                        [-4.18563, -0.48658, 2.15539, 0.13399],
                        [-4.2491, -0.19587, 2.38127, 0.12496],
                        [-4.27431, 0.10348, 2.65826, 0.11301],
                        [-4.26782, 0.40421, 2.803, 0.10731],
                        [-4.23283, 0.70291, 1.92672, 0.15609],
                        [-4.17244, 0.9974, 1.56428, 0.19218],
                        [-4.08994, 1.2864, 1.438, 0.209],
                        [-3.9885, 1.56931, 1.4364, 0.20924],
                        [-3.87124, 1.84615, 1.4364, 0.20931],
                        [-3.73898, 2.11482, 1.4364, 0.20848],
                        [-3.57022, 2.33639, 1.4364, 0.1939],
                        [-3.36287, 2.49276, 1.4364, 0.1808],
                        [-3.12697, 2.57595, 1.4364, 0.17414],
                        [-2.87543, 2.58504, 1.51161, 0.16652],
                        [-2.62141, 2.52192, 1.38736, 0.18866],
                        [-2.37737, 2.39426, 1.3, 0.21185],
                        [-2.15184, 2.21543, 1.3, 0.22141],
                        [-1.94561, 2.00315, 1.3, 0.22766],
                        [-1.75809, 1.78116, 1.3, 0.22353],
                        [-1.54252, 1.6264, 1.3, 0.20413],
                        [-1.30595, 1.55558, 1.3, 0.18996],
                        [-1.06586, 1.57218, 1.32218, 0.18202],
                        [-0.84144, 1.67122, 1.39962, 0.17526],
                        [-0.65076, 1.84364, 1.52137, 0.16898],
                        [-0.50843, 2.07515, 1.66801, 0.16293],
                        [-0.42337, 2.34844, 1.80631, 0.15846],
                        [-0.3998, 2.64522, 1.9562, 0.15219],
                        [-0.43388, 2.94189, 2.04516, 0.14601],
                        [-0.51818, 3.22391, 2.17944, 0.13506],
                        [-0.64385, 3.48814, 2.36875, 0.12352],
                        [-0.8032, 3.73476, 2.64884, 0.11085],
                        [-0.989, 3.96547, 2.37797, 0.12457],
                        [-1.1944, 4.18277, 2.22754, 0.13423],
                        [-1.41262, 4.38958, 2.22455, 0.13515],
                        [-1.63724, 4.58982, 2.22455, 0.13527],
                        [-1.86847, 4.78095, 2.22455, 0.13486],
                        [-2.11307, 4.94743, 2.22455, 0.13301],
                        [-2.373, 5.07902, 2.22455, 0.13097],
                        [-2.64698, 5.17086, 2.22455, 0.1299],
                        [-2.93184, 5.2222, 2.31591, 0.12498],
                        [-3.22365, 5.23495, 2.46024, 0.11872],
                        [-3.51838, 5.21226, 2.6626, 0.11102],
                        [-3.81212, 5.15853, 2.65904, 0.1123],
                        [-4.10167, 5.0789, 2.39908, 0.12517],
                        [-4.38531, 4.97869, 2.33818, 0.12866],
                        [-4.66279, 4.86232, 2.33818, 0.12869],
                        [-4.93336, 4.73068, 2.33818, 0.12869],
                        [-5.19456, 4.58175, 2.33818, 0.12859],
                        [-5.43717, 4.4073, 2.33818, 0.1278],
                        [-5.65307, 4.20551, 2.33818, 0.12639],
                        [-5.83868, 3.97854, 2.42123, 0.12109],
                        [-5.99466, 3.73047, 2.63271, 0.1113],
                        [-6.12444, 3.46593, 2.9879, 0.09862],
                        [-6.23292, 3.18941, 3.55968, 0.08344],
                        [-6.32574, 2.905, 4.0, 0.07479],
                        [-6.40836, 2.61606, 4.0, 0.07513],
                        [-6.48574, 2.32525, 4.0, 0.07523],
                        [-6.56163, 2.03404, 4.0, 0.07523],
                        [-6.63646, 1.74257, 4.0, 0.07523],
                        [-6.71046, 1.45088, 4.0, 0.07523],
                        [-6.78373, 1.15901, 4.0, 0.07523],
                        [-6.8567, 0.86706, 4.0, 0.07523],
                        [-6.92976, 0.57516, 4.0, 0.07523],
                        [-7.00646, 0.2843, 3.34048, 0.09005],
                        [-7.08748, -0.00531, 2.62403, 0.11461],
                        [-7.17197, -0.29391, 2.32835, 0.12915],
                        [-7.25917, -0.58172, 2.21552, 0.13574],
                        [-7.34927, -0.86866, 2.20244, 0.13656],
                        [-7.43501, -1.15604, 2.20244, 0.13616],
                        [-7.50117, -1.44725, 2.20244, 0.13559],
                        [-7.53556, -1.74231, 2.20244, 0.13488],
                        [-7.52972, -2.03806, 2.20244, 0.13431],
                        [-7.47996, -2.32914, 2.20244, 0.13408],
                        [-7.38681, -2.6097, 2.25492, 0.1311],
                        [-7.25376, -2.87477, 2.36548, 0.12538],
                        [-7.08628, -3.1211, 2.52494, 0.11797],
                        [-6.89056, -3.34727, 2.73625, 0.10931],
                        [-6.6727, -3.55357, 2.99312, 0.10024],
                        [-6.43798, -3.74141, 3.32566, 0.0904],
                        [-6.19095, -3.91313, 3.78402, 0.07951],
                        [-5.93527, -4.07176, 2.81816, 0.10677],
                        [-5.67392, -4.22088, 2.05371, 0.14652],
                        [-5.40925, -4.36405, 1.88291, 0.15981],
                        [-5.14319, -4.50456, 1.87543, 0.16044],
                        [-4.87889, -4.64828, 1.87543, 0.16041],
                        [-4.61672, -4.79578, 1.87543, 0.1604],
                        [-4.44937, -4.87386, 1.87543, 0.09847],
                        [-4.24731, -4.94074, 1.87543, 0.11349],
                        [-4.01564, -4.97879, 1.87543, 0.12518],
                        [-3.76339, -4.97647, 1.9991, 0.12619],
                        [-3.50032, -4.93116, 2.2184, 0.12033],
                        [-3.23387, -4.84626, 2.56256, 0.10913],
                        [-2.96865, -4.72905, 3.13294, 0.09255],
                        [-2.70662, -4.58917, 4.0, 0.07426],
                        [-2.44764, -4.43695, 4.0, 0.0751],
                        [-2.18981, -4.28176, 4.0, 0.07523],
                        [-1.9324, -4.12587, 4.0, 0.07523],
                        [-1.67521, -3.96964, 4.0, 0.07523],
                        [-1.41807, -3.81332, 4.0, 0.07523],
                        [-1.16077, -3.65725, 4.0, 0.07523],
                        [-0.90357, -3.50103, 4.0, 0.07523],
                        [-0.64624, -3.34501, 4.0, 0.07523],
                        [-0.38867, -3.18939, 3.01455, 0.09983],
                        [-0.13064, -3.03454, 2.48648, 0.12103],
                        [0.12808, -2.88084, 2.14736, 0.14014],
                        [0.38771, -2.72869, 1.82417, 0.16497],
                        [0.64855, -2.57866, 1.66456, 0.18078],
                        [0.91494, -2.44081, 1.60608, 0.18675],
                        [1.19022, -2.32577, 1.60608, 0.18576],
                        [1.47583, -2.24478, 1.60608, 0.18484],
                        [1.77018, -2.21032, 1.60608, 0.18452],
                        [2.06465, -2.2413, 1.60608, 0.18436],
                        [2.33709, -2.34663, 1.60608, 0.18187],
                        [2.56363, -2.51773, 1.64287, 0.1728],
                        [2.73552, -2.73712, 1.76274, 0.15811],
                        [2.85499, -2.99016, 1.73509, 0.16127],
                        [2.92834, -3.266, 1.63467, 0.17461],
                        [2.96541, -3.5559, 1.63467, 0.17879],
                        [2.98088, -3.85247, 1.63467, 0.18168],
                        [2.99187, -4.14938, 1.63467, 0.18176],
                        [3.04893, -4.42855, 1.63467, 0.17431],
                        [3.16276, -4.67566, 1.63467, 0.16643],
                        [3.33048, -4.88216, 1.66648, 0.15964],
                        [3.54405, -5.04433, 1.79942, 0.14902],
                        [3.79363, -5.162, 2.01993, 0.1366],
                        [4.06903, -5.2381, 2.31601, 0.12337],
                        [4.36066, -5.27781, 2.67959, 0.10984],
                        [4.66011, -5.2876, 2.84694, 0.10524],
                        [4.96003, -5.27408, 2.74929, 0.1092],
                        [5.25537, -5.23389, 2.46119, 0.12111],
                        [5.5446, -5.1673, 2.33481, 0.12712],
                        [5.82666, -5.07551, 2.33009, 0.1273],
                        [6.10043, -4.95917, 2.33009, 0.12766],
                        [6.36401, -4.81789, 2.33009, 0.12835],
                        [6.61302, -4.65103, 2.33009, 0.12864],
                        [6.83705, -4.45751, 2.33009, 0.12705],
                        [7.03076, -4.23856, 2.33009, 0.12546],
                        [7.19291, -3.99686, 2.39584, 0.12149],
                        [7.32393, -3.73555, 2.51601, 0.11618],
                        [7.42503, -3.45816, 2.68885, 0.1098],
                        [7.49816, -3.16888, 2.88975, 0.10325],
                        [7.54557, -2.87265, 3.11533, 0.0963],
                        [7.57016, -2.57316, 3.37621, 0.089],
                        [7.57511, -2.27242, 3.72166, 0.08082],
                        [7.56385, -1.9718, 3.9096, 0.07695],
                        [7.53792, -1.67207, 4.0, 0.07521],
                        [7.49877, -1.37375, 4.0, 0.07522],
                        [7.44779, -1.07721, 4.0, 0.07522],
                        [7.38629, -0.78264, 3.57298, 0.08422],
                        [7.31549, -0.49018, 2.64377, 0.11382],
                        [7.23586, -0.19999, 2.07036, 0.14534],
                        [7.14713, 0.08754, 1.82869, 0.16455],
                        [7.04864, 0.37188, 1.74301, 0.17264],
                        [6.93861, 0.65195, 1.74301, 0.17264],
                        [6.81242, 0.92504, 1.74301, 0.1726],
                        [6.65815, 1.18266, 1.74301, 0.17227],
                        [6.4638, 1.40752, 1.74301, 0.17052],
                        [6.2298, 1.58145, 1.74301, 0.16727],
                        [5.9659, 1.69379, 1.77548, 0.16154],
                        [5.6847, 1.744, 1.90178, 0.1502],
                        [5.39679, 1.73812, 2.16269, 0.13316],
                        [5.1091, 1.68743, 2.58409, 0.11305],
                        [4.82471, 1.60448, 3.30386, 0.08966],
                        [4.54387, 1.50149, 4.0, 0.07478],
                        [4.26489, 1.38868, 4.0, 0.07523],
                        [3.98559, 1.27665, 4.0, 0.07523],
                        [3.70609, 1.16513, 4.0, 0.07523],
                        [3.42642, 1.05402, 4.0, 0.07523]]

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

        DISTANCE_MULTIPLE = 1
        # distance_reward = math.cos( 2 * (distance_from_center/track_width) * math.pi)
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



# add parameter verbose=True to get noisy output for testing
reward_object = Reward(
    # verbose=True
    )

def reward_function(params):
    return reward_object.reward_function(params)
