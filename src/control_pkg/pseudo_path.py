import numpy as np

def heading_angle_generator(optimal_x_y_path, theta_init):

    x_path_optimal = optimal_x_y_path()
    y_path_optimal = optimal_x_y_path()

    t = np.shape()

    theta_path_optimal = [theta_init] # Initial Heading Angle 

    for i in range(np.size(t) - 2):
        delta_y = y_path_optimal[i + 2] - y_path_optimal[i + 1]
        delta_x = x_path_optimal[i + 2] - x_path_optimal[i + 1]
        theta_path_optimal.append(np.arctan(delta_y/delta_x))

    # Terminal State
    theta_f = theta_path_optimal(-1) # The Terminal Heading Angle Euqals to the Angle at n-1 step

    theta_path_optimal.append(theta_f) # Construct Optimal Heading Angle

    optimal_path = optimal_x_y_path.append() # Contruct Optimal Path which Includes Heading Angle

    return optimal_path