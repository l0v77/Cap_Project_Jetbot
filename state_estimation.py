import cv2
import numpy as np
import math

# Initial parameters
board_size = [1000, 1000]  # [length, height] of board in mm


def get_uncalibrated_param(corners, ids):
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # print(ids)
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
        # extract the marker corners (which are always returned
        # in top-left, top-right, bottom-right, and bottom-left
        # order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # compute the center (x, y)-coordinates of the ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        if markerID == 0:
            board_tl = (cX, cY)
        if markerID == 1:
            board_bl = (cX, cY)
        if markerID == 2:
            board_br = (cX, cY)
        if markerID == 3:
            board_tr = (cX, cY)
        if markerID == 4:
            car_pos = (cX, cY)  # Position of JetBot on uncalibrated image
            dx = topRight[0] - topLeft[0]  # dx and dy forms the orientation vector
            dy = topRight[1] - topLeft[1]

    src_pts = np.array([board_tl, board_tr, board_bl, board_br]).reshape(-1, 1, 2)

    return src_pts, car_pos, [dx, dy]

def get_state(src_pts, board_size, pos_uncalib, orient_vector_uncalib):
    # src_pts are the location of the corners of the board: np.array([tl, tr, bl, br]).reshape(-1, 1, 2)
    # board_size is a list of [length, height]
    # pos_uncalib: [x, y] is the uncalibrated pixel coordinate of JetBot on the captured video
    # orient_vector_uncalib is the vector [dx, dy] that represents the orientation of the JetBot
    # returns calibrated position (size(2,) np array) and orientation (float) of JetBot
    src_pts = src_pts.reshape(-1, 1, 2)

    # Construct dst_pts, the destination coordinate of where the src_pts are after the Homography transformation
    dst_tl = [0, 0]
    dst_tr = [board_size[0], 0]
    dst_bl = [0, board_size[1]]
    dst_br = [board_size[0], board_size[1]]
    dst_pts = np.array([dst_tl, dst_tr, dst_bl, dst_br]).reshape(-1, 1, 2)

    # Finds the homography transformation matrix
    H, status = cv2.findHomography(src_pts, dst_pts)

    # Calculate calibrated car position
    pos_uncalib = np.array(pos_uncalib)
    pos_uncalib = np.vstack((pos_uncalib.reshape((2, 1)), [1]))
    car_pos_calib = H @ pos_uncalib

    # Calculate calibrated car orientation
    ox = orient_vector_uncalib[0]
    oy = orient_vector_uncalib[1]
    dist = np.linalg.norm(orient_vector_uncalib)
    orient_vector_uncalib = np.array([[ox / dist], [oy / dist], [0]])
    vector_calib = H @ orient_vector_uncalib
    theta_calib = math.atan2(vector_calib[1], vector_calib[0])

    return car_pos_calib[0:2].reshape((2,)), theta_calib


# Capture video
cap = cv2.VideoCapture(0)

# Load Aruco dictionary and create parameters to detect aruco tags
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

while True:
    # Read video, detect ArUco Markers
    ret, frame = cap.read()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
                                                       parameters=arucoParams)

    if len(corners) > 4:
        src_pts, pos_uncalib, orient_vector_uncalib = get_uncalibrated_param(corners, ids)
        pos, orient = get_state(src_pts, board_size, pos_uncalib, orient_vector_uncalib)
        # print('pos:', pos)
        print('orientation:', orient)

    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
