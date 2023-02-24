import numpy as np

def heading_angle_generator(optimal_x_y_path, theta_init):
    

    # optimal_x_y_path has unit of mm, convert to m first
    x_path_optimal = optimal_x_y_path[:,0]
    y_path_optimal = optimal_x_y_path[:,1]

    theta_path_optimal = [theta_init] # Initial Heading Angle 

    for i in range(np.size(x_path_optimal) - 2):
        delta_y = y_path_optimal[i + 2] - y_path_optimal[i + 1]
        delta_x = x_path_optimal[i + 2] - x_path_optimal[i + 1]
        theta_path_optimal.append(np.arctan(delta_y/delta_x))

    # Terminal State
    theta_f = theta_path_optimal(-1) # The Terminal Heading Angle Euqals to the Angle at n-1 step

    theta_path_optimal.append(theta_f) # Construct Optimal Heading Angle

    optimal_path = optimal_x_y_path.append(theta_path_optimal, axis = 1) # Contruct Optimal Path which Includes Heading Angle

    return optimal_path