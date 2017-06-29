import cv2
import numpy as np

blue = cv2.cvtColor(np.uint8([[[255, 0, 0]]]), cv2.COLOR_BGR2HSV)
green = cv2.cvtColor(np.uint8([[[0, 255, 0]]]), cv2.COLOR_BGR2HSV)
red = cv2.cvtColor(np.uint8([[[0, 0, 255]]]), cv2.COLOR_BGR2HSV)

print blue
print green
print red

frame = cv2.imread('light6.jpg')
# Convering BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

lower_red = np.array([0, 120, 120])
upper_red = np.array([10, 255, 255])

lower_green = np.array([60, 120, 120])
upper_green = np.array([90, 255, 255])


# Creating a mask where the values in the range are made 255 others are made 0
mask_red = cv2.inRange(hsv, lower_red, upper_red)
mask_red = cv2.GaussianBlur(mask_red, (5, 5), 0)
cv2.imshow('Mask', mask_red)
res_red = cv2.bitwise_and(frame, frame, mask=mask_red)
# im2, contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# # cv2.drawContours(res_red, contours_red, 3, (255, 0, 0), 3)
# red_cnt = contours_red[0]
# red_area = cv2.contourArea(red_cnt)
# print red_area
# print red_cnt[1]

mask_green = cv2.inRange(hsv, lower_green, upper_green)
res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

im2, contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# cv2.drawContours(res_green, contours_green, 3, (255, 0, 0), 3)

# smooth = cv2.filter2D(res, -1, kernel)
# blur = cv2.GaussianBlur(res, (5, 5), 0)
# median = cv2.medianBlur(res, 15)

cv2.imshow('Frame', frame)
# cv2.imshow('Smooth', mask)
cv2.imshow('Red', res_red)
cv2.imshow('Green', res_green)
# cv2.imshow('Gaussian', blur)
# cv2.imshow('Median', median)

cv2.waitKey(0)
cv2.destroyAllWindows()
