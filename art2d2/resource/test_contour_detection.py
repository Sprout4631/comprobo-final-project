#!/usr/bin/env Python3
import numpy as np
import cv2 as cv
im = cv.imread('rectangle.jpg')

assert im is not None, "file could not be read, check with os.path.exists()"

imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

print("Contours:")
print(contours)

print("----------")
print("Hierarchy:")
print(hierarchy)

labeled_image = cv.drawContours(im, contours, -1, (0,255,0), 3)
cv.imshow("Detected contours", im)

timeout_wait = 0
k = cv.waitKey(timeout_wait)