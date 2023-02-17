import numpy as np
import cv2

from vision_pkg.state.initialize import calib_frame
from vision_pkg.state import state_estimation
from vision_pkg.state.state_estimation import get_state
from vision_pkg.detect_object.Camera import generate_map


# Measure the board size [length, height] in mm
# length is the distance from top left to top right (tag 0 to tag 3)
# height is the distance from top left to bottom left (tag 0 to tag 1)
board_size = [320, 440]

# speed to PWM ratio (PWM/speed)
ratio = 1/0.94

cap = cv2.VideoCapture(0)
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

# Initialize (find obstacle map, find path)
# 1st while: generate obstacle map
while True:
    ret, img = cap.read()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)
    cv2.imshow("not calibrated", img)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    if len(corners) > 3:
        ids = ids.flatten()
        if 0 in ids and 1 in ids and 2 in ids and 3 in ids:
            calibrated_img, homograph_matrix = calib_frame(img, corners, ids, board_size)
            # cv2.imshow("calibrated", calibrated_img)
            # cv2.waitKey(1000)
            break
cv2.destroyAllWindows()
print('obtained calibrated img')
# NOTE: row correspond to y, col correspond to x
A_star_map = generate_map(calibrated_img)

# 2nd while: find start and end position:
while True:
    ret, img = cap.read()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)

    if len(corners) >= 2:
        ids = ids.flatten()
        can_see_corners = 0 in ids and 1 in ids and 2 in ids and 3 in ids
        if can_see_corners and 4 in ids and 5 in ids:
            start_state = state_estimation.get_state_old(corners, ids, board_size)
            start_pos = start_state[0:2]
            end_pos = state_estimation.get_desired_pos_old(corners, ids, board_size)
            break

print('start and end position found')
# Now have A_star_map, start_pos, end_pos
# TODO: feed A_star_map and start_pos, end_pos to generate path


# Run MPC
current_state = start_state
while True:
    ret, img = cap.read()

    calibrated_frame = cv2.warpPerspective(img, homograph_matrix, np.array(board_size))
    cv2.imshow("calibrated", cv2.resize(calibrated_frame, (1000, int(1000/board_size[1]*board_size[0]))))
    cv2.waitKey(1)

    (calib_corners, calib_ids, calib_rej) = cv2.aruco.detectMarkers(calibrated_frame, arucoDict,
                                                                    parameters=arucoParams)
    if len(calib_corners) > 0:
        calib_ids.flatten()
        if 4 in calib_ids:
            current_state = get_state(calib_corners, calib_ids, board_size)
            print(current_state)

    # TODO: call MPC
