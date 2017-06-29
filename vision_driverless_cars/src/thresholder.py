import cv2
import numpy as np


class Thresholder:
    def __init__(self):
        self.sobel_kernel = 9

        self.thresh_dir_min = 0.5
        self.thresh_dir_max = 1.2

        self.thresh_mag_min = 80
        self.thresh_mag_max = 255

    def dir_thresh(self, sobelx, sobely):
        abs_sobelx = np.abs(sobelx)
        abs_sobely = np.abs(sobely)
        scaled_sobel = np.arctan2(abs_sobely, abs_sobelx)
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= self.thresh_dir_min) & (scaled_sobel <= self.thresh_dir_max)] = 255
        # cv2.imshow('Direction', sxbinary)
        return sxbinary

    def mag_thresh(self, sobelx, sobely):
        gradmag = np.sqrt(sobelx ** 2 + sobely ** 2)
        scale_factor = np.max(gradmag) / 255
        gradmag = (gradmag / scale_factor).astype(np.uint8)
        binary_output = np.zeros_like(gradmag)
        binary_output[(gradmag >= self.thresh_mag_min) & (gradmag <= self.thresh_mag_max)] = 255
        # cv2.imshow('Magnitude', binary_output)
        return binary_output

    def color_thresh(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        yellow_min = np.array([60, 100, 140], np.uint8)
        yellow_max = np.array([120, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(img, yellow_min, yellow_max)

        white_min = np.array([0, 0, 220], np.uint8)
        white_max = np.array([255, 40, 255], np.uint8)
        white_mask = cv2.inRange(img, white_min, white_max)

        binary_output = np.zeros_like(img[:, :, 0])
        binary_output[((yellow_mask != 0) | (white_mask != 0))] = 255

        filtered = img
        filtered[((yellow_mask == 0) & (white_mask == 0))] = 0
        # cv2.imshow('Color Threshold', filtered)
        return binary_output

    def threshold(self, img):
        sobelx = cv2.Sobel(img[:, :, 2], cv2.CV_64F, 1, 0, ksize=self.sobel_kernel)
        sobely = cv2.Sobel(img[:, :, 2], cv2.CV_64F, 0, 1, ksize=self.sobel_kernel)

        direc = self.dir_thresh(sobelx, sobely)
        mag = self.mag_thresh(sobelx, sobely)
        color = self.color_thresh(img)

        combined = np.zeros_like(direc)
        combined[((color == 255) & ((mag == 255) | (direc == 255)))] = 255
        return combined


if __name__ == '__main__':
    cap = cv2.VideoCapture('challenge.mp4')
    threshold_object = Thresholder()

    while(cap.isOpened()):
        ret, color = cap.read()
        th = threshold_object.threshold(color)
        cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Original', color)
        cv2.namedWindow('Combined', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Combined', threshold_object.threshold(color))
        if cv2.waitKey(1) & 0xFF == 27:
            break

cv2.destroyAllWindows()
