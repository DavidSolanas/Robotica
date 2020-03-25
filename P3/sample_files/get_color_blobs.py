#!/usr/bin/python
# -*- coding: UTF-8 -*-


###########################################
#   Autores: Daniel Cay (741066)          #
#            Javier Fañanás (737987)      #
#            David Solanas (738630)       #
#                                         #
#   Fichero: get_color_blobs.py           #
#   Robótica - Práctica 3                 #
###########################################

# Standard imports
import cv2
import numpy as np;

# Read image
img_BGR = cv2.imread("a.jpg")
#img_BGR = cv2.imread("many.jpg")

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
params.minArea = 200
params.maxArea = 10000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Color
params.filterByColor = False
# not directly color, but intensity on the channel input
#params.blobColor = 0
params.filterByConvexity = False
params.filterByInertia = False


# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else :
	detector = cv2.SimpleBlobDetector_create(params)

# filter certain RED COLOR channels

# Pixels with 0 <= H <= 10, 170 <= H <= 180, 
# 70 <= S <= 255, 50 <= V <= 255 will be considered red.

img_HSV = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

# BY DEFAULT, opencv IMAGES have BGR format
redMin1 = (0, 70, 50)
redMax1 = (10, 255, 255)
redMin2 = (170, 70, 50)
redMax2 = (180, 255, 255)

# Create RED mask in HSV colorspace
mask_red1=cv2.inRange(img_HSV, redMin1, redMax1)
mask_red2 = cv2.inRange(img_HSV, redMin2, redMax2)
mask_red = mask_red1 + mask_red2


# apply the mask
red = cv2.bitwise_and(img_HSV, img_BGR, mask = mask_red)
# show resulting filtered image next to the original one
cv2.imshow("Red regions", np.hstack([img_BGR, red]))

# detector finds "dark" blobs by default
keypoints_red = detector.detect(mask_red)

# documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
print(keypoints_red)
for kp in keypoints_red:
	print(kp.pt[0], kp.pt[1], kp.size)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints_red, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show mask and blobs found
cv2.imshow("Keypoints on RED", im_with_keypoints)
cv2.waitKey(0)

