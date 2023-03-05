import cv2
import numpy as np
from PIL import Image as im
from matplotlib import pyplot as plt

# cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop)
# ret,img = cap.read() # return a single frame in variable `frame`


def generate_map(img):
    # Need to input the calibrated image of the map with obstacles on it
    # Need to input the board_size: list of [length, height]

    board_size = [img.shape[1], img.shape[0]]

    # #################################### New Code, use HSV
    # HSV parameters
    # pink sheet (dark light)
    hsv_low_bound = np.array([151, 83, 176])
    hsv_up_bound = np.array([180, 255, 255])

    # # pink phone case in dark lighting
    # hsv_low_bound = np.array([0, 23, 180])
    # hsv_up_bound = np.array([13, 169, 221])

    # # pink phone case
    # hsv_low_bound = np.array([0, 19, 23])
    # hsv_up_bound = np.array([13, 83, 255])

    # # dark blue file pkg
    # hsv_low_bound = np.array([82, 68, 146])
    # hsv_up_bound = np.array([135, 150, 210])

    # # No constraint
    # hsv_low_bound = np.array([0, 0, 0])
    # hsv_up_bound = np.array([0, 0, 0])
    blur = cv2.GaussianBlur(img, (3, 3), 3)
    img_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, hsv_low_bound, hsv_up_bound)
    cv2.imshow('mask', mask)
    cv2.waitKey(1000)

    # Turn img into matrix
    # A_star = np.asarray(mask)
    # A_star = A_star/255
    # A_star.astype(np.uint8)
    # print(type(A_star))
    # print(A_star.shape)
    # print(A_star[0:20, 0:20])
    ############################################################

    A_star = np.zeros((board_size[1], board_size[0]), dtype=np.uint8)
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cont_pts = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        extend = 10  # 12 cm of distance added around the contour
        x_min = x - extend
        y_min = y - extend
        x_max = x_min + w + 2*extend
        y_max = y_min + h + 2*extend

        # make sure no index out of bounds
        x_min = np.clip(x_min, 0, board_size[0])
        y_min = np.clip(y_min, 0, board_size[1])
        x_max = np.clip(x_max, 0, board_size[0])
        y_max = np.clip(y_max, 0, board_size[1])

        area_threshold = 500
        area = (x_max - x_min)*(y_max - y_min)
        # plt.imshow(cv2.rectangle(th1, (x, y), (x+w, y+h), (100, 100, 255), 10))
        # plt.title('Bounding Rectangle')
        # plt.show()
        # print((x, y), (x, y+h), (x+w, y), (x+w, y+h))
        if area > area_threshold:
            A_star[y_min:y_max, x_min:x_max] = np.ones((y_max - y_min, x_max - x_min), dtype=int)
            # print('area', area)
            cont_pts.append([x_min, x_max, y_min, y_max])


    # ############################### Old Code, using contours
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GREY)
    # ret, th1 = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    # A_star = np.ones((board_size[1], board_size[0]), dtype=int)
    # contours, hierarchy = cv2.findContours(th1.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)
    #     # plt.imshow(cv2.rectangle(th1, (x, y), (x+w, y+h), (100, 100, 255), 10))
    #     # plt.title('Bounding Rectangle')
    #     # plt.show()
    #     # print((x, y), (x, y+h), (x+w, y), (x+w, y+h))
    #     A_star[y:y+h, x:x+w] = np.zeros((h, w), dtype=int)
    ####################################################

    # flip the matrix upside down to make it in the right coordinate
    A_star = np.flipud(A_star)
    # print(A_star)
    # A_star = A_star.astype('uint8')
    # test_output = im.fromarray(A_star*255)
    # test_output.save('test_output.png')

    return A_star, cont_pts
