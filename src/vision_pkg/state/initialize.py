import cv2
import numpy as np


def calib_matrix(corners, ids, board_size):
    # MAKE SURE THAT WHEN CALLING THIS FUNCTION, ALL 4 CORNERS ARE DETECTED
    # corners and ids are from ArUco tags detection
    # board_size is a list of [length, height] of the map
    # returns the calibration matrix H

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
        # topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
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

    src_pts = np.array([board_tl, board_tr, board_bl, board_br]).reshape(-1, 1, 2)

    dst_tl = [0, 0]
    dst_tr = [board_size[0], 0]
    dst_bl = [0, board_size[1]]
    dst_br = [board_size[0], board_size[1]]
    dst_pts = np.array([dst_tl, dst_tr, dst_bl, dst_br]).reshape(-1, 1, 2)

    # Finds the homography transformation matrix
    H, status = cv2.findHomography(src_pts, dst_pts)
    return H


def calib_frame(frame, corner, mark_id, board_size):
    # MAKE SURE THAT WHEN CALLING THIS FUNCTION, ALL 4 CORNERS ARE DETECTED
    # img is the camera captured img (video frame)
    # board_size is a list [length, height] that represent the size of the map
    # returns the calibrated image

    H = calib_matrix(corner, mark_id, board_size)

    # find calibrated img
    board_size = np.array([board_size[0], board_size[1]])
    calib_img = cv2.warpPerspective(frame, H, board_size.reshape((2,)))

    return calib_img, H


######################################################################
# # Test code
# board_size = [500, 500]
# cap = cv2.VideoCapture(0)
# arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
# arucoParams = cv2.aruco.DetectorParameters_create()
#
# while True:
#     ret, img = cap.read()
#     (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
#                                                        parameters=arucoParams)
#     cv2.imshow("not calibrated", img)
#     key = cv2.waitKey(1) & 0xFF
#     # if the `q` key was pressed, break from the loop
#     if key == ord("q"):
#         break
#     if len(corners) > 3:
#         ids = ids.flatten()
#         if 0 in ids and 1 in ids and 2 in ids and 3 in ids:
#             calibrated_img = calib_frame(img, corners, ids, board_size)
#             # cv2.imshow("calibrated", calibrated_img)
#             # cv2.waitKey(2000)
#             break
#
# # cv2.destroyAllWindows()
# cap.release()
