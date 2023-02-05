import cv2
# from imutils.video import VideoStream
# import imutils
# import cv2.aruco as aruco
import numpy as np
import math
from calibration_test import calibrate

# Capture video
cap = cv2.VideoCapture(0)

# Load Aruco dictionary and create parameters to detect aruco tags
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

while True:
    # Read video, detect ArUco Markers
    ret, frame = cap.read()
    # frame = imutils.resize(frame, width=1000)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
                                                       parameters=arucoParams)

    # Get corners of the ArUco markers and Plot
    if len(corners) >= 5:
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

            # cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            # cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            # cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            # cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            # compute and draw the center (x, y)-coordinates of the ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            # cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # # draw the ArUco marker ID on the frame
            # cv2.putText(frame, str(markerID),
            #             (topLeft[0], topLeft[1] - 15),
            #             cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (0, 255, 0), 2)

            if markerID == 0:
                board_tl = (cX, cY)
            if markerID == 1:
                board_bl = (cX, cY)
            if markerID == 2:
                board_br = (cX, cY)
            if markerID == 3:
                board_tr = (cX, cY)
            if markerID == 4:
                car_pos = (cX, cY)
                dx = topRight[0] - topLeft[0]
                dy = topRight[1] - topLeft[1]
                dist = np.linalg.norm(np.array(topRight) - np.array(topLeft))
                cv2.circle(frame, car_pos, 4, (0, 0, 255), -1)
                cv2.line(frame, car_pos, (car_pos[0]+dx, car_pos[1]+dy), (255, 0, 0), 2)

        src_pts = np.array([board_tl, board_tr, board_bl, board_br]).reshape(-1, 1, 2)

        # The destination coordinates (mm) for the corners (need to change according to the game board size)
        dst_tl = [0, 0]
        dst_tr = [500, 0]
        dst_bl = [0, 700]
        dst_br = [500, 700]
        dst_pts = np.array([dst_tl, dst_tr, dst_bl, dst_br]).reshape(-1, 1, 2)
        # print(src_pts)

        # test code
        H, status = cv2.findHomography(src_pts, dst_pts)
        # print(H)
        # The size of the calibrated img (same as the coord of the br point)
        size = dst_pts[-1, :]
        dst_img = cv2.warpPerspective(frame, H, size.reshape((2,)))
        cv2.imshow("Homography", dst_img)

        car_coord, car_orient = calibrate(frame, src_pts, dst_pts, np.array(car_pos), dx, dy)

        # Draw the board
        cv2.line(frame, board_tl, board_tr, (0, 255, 0), 2)
        cv2.line(frame, board_tr, board_br, (0, 255, 0), 2)
        cv2.line(frame, board_br, board_bl, (0, 255, 0), 2)
        cv2.line(frame, board_bl, board_tl, (0, 255, 0), 2)

    # show the output frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
cap.release()