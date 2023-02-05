import numpy as np
import cv2
import math


def calibrate(img, src_pts, dst_pts, car_pos, dx, dy):

    # Finds the homography transformation matrix
    H, status = cv2.findHomography(src_pts, dst_pts)
    # print(H)
    # The size of the calibrated img (same as the coord of the br point)
    size = dst_pts[-1, :]
    dst_img = cv2.warpPerspective(img, H, size.reshape((2,)))
    cv2.imshow("Homography", dst_img)

    # Calculate calibrated car position
    car_pos = car_pos.reshape((2, 1))
    car_pos = np.vstack((car_pos, [1]))
    car_coord = H@car_pos

    # Calculate calibrated car orientation
    dist = np.linalg.norm([dx, dy])
    orient_uncalibrated = np.array([[dx/dist], [dy/dist], [0]])
    orient = H@orient_uncalibrated
    theta = math.atan2(orient[1], orient[0])

    # print(car_coord)
    print(theta)
    return car_coord, theta
