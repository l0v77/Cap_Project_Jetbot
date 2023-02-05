import cv2
import numpy as np
from PIL import Image as im
from matplotlib import pyplot as plt

# cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop)
# ret,img = cap.read() # return a single frame in variable `frame`


def generate_map(img):
    # Need to input the calibrated image of the map with obstacles on it
    # Need to input the board_size: list of [length, height]

    # TODO: cv2.inRange (1st BGR2HSV, find the range of color that is obstacle, then call the inRange function)

    board_size = [img.shape[1], img.shape[0]]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, th1 = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    A_star = np.ones((board_size[1], board_size[0]), dtype=int)
    contours, hierarchy = cv2.findContours(th1.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        # plt.imshow(cv2.rectangle(th1, (x, y), (x+w, y+h), (100, 100, 255), 10))
        # plt.title('Bounding Rectangle')
        # plt.show()
        # print((x, y), (x, y+h), (x+w, y), (x+w, y+h))
        A_star[y:y+h, x:x+w] = np.zeros((h, w), dtype=int)

    # flip the matrix upside down to make it in the right coordinate
    A_star = np.flipud(A_star)
    # print(A_star)
    # A_star = A_star.astype('uint8')
    # test_output = im.fromarray(A_star*255)
    # test_output.save('test_output.png')
    return A_star
