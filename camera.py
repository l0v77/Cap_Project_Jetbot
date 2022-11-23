import numpy as np
import cv2 as cv
import time
import matplotlib.pyplot as plt
from matplotlib import pyplot as plt


# def process(img):
#     orig_image = img.copy()
#     img = cv.medianBlur(img, 5)
#     ret, th1 = cv.threshold(img, 127, 255, cv.THRESH_BINARY)
#     # plt.imshow(th1,'gray')
#     # plt.xticks([]),plt.yticks([])
#     # plt.show()
#     im2, contours, hierarchy = cv.findContours(th1.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
#     plt.imshow(th1, cmap='gray')
#     print(len(contours))
#     for c in contours:
#         x, y, w, h = cv.boundingRect(c)
#         # plt.imshow(cv.rectangle(th1, (x, y), (x + w, y + h), (100, 100, 255), 10))
#         # plt.title('Bounding Rectangle');
#         # plt.show()
#     return (x, y), (x, y + h), (x + w, y), (x + w, y + h)


def main(cap):
    print("0")

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        print("1")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame

        # process(frame)
        cv.imshow('frame', gray)
        if cv.waitKey(1) == ord('q'):
            break
        # When everything done, release the capture
        time.sleep(1)


if __name__ == '__main__':
    try:
        print("Initializing...")
        cap = cv.VideoCapture(0)
        main(cap)
    finally:
        cap.release()
        cv.destroyAllWindows()
