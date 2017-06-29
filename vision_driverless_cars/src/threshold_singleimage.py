import cv2
import numpy as np
from Region import Region_of_interest


class Thresholder:
    def __init__(self):
        self.sobel_kernel = 15

        self.thresh_dir_min = 0.4
        self.thresh_dir_max = 1.6

        self.thresh_mag_min = 20
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

        white_min = np.array([0, 0, 180], np.uint8)
        white_max = np.array([255, 255, 255], np.uint8)
        white_mask = cv2.inRange(img, white_min, white_max)

        binary_output = np.zeros_like(img[:, :, 0])
        binary_output[((white_mask != 0))] = 255

        filtered = img
        filtered[((white_mask == 0))] = 0
        # cv2.imshow('Color Threshold', filtered)
        return binary_output

    def threshold(self, img):
        sobelx = cv2.Sobel(img[:, :, 2], cv2.CV_64F, 1, 0, ksize=self.sobel_kernel)
        sobely = cv2.Sobel(img[:, :, 2], cv2.CV_64F, 0, 1, ksize=self.sobel_kernel)

        direc = self.dir_thresh(sobelx, sobely)
        mag = self.mag_thresh(sobelx, sobely)
        color = self.color_thresh(img)

        combined = np.zeros_like(direc)
        combined[((color == 255) | ((mag == 255) & (direc == 255)))] = 255
        kernel_size = 3
        combined = cv2.GaussianBlur(combined, (kernel_size, kernel_size), 0)
        return combined


if __name__ == '__main__':
    color = cv2.imread('image.jpg')
    threshold_object = Thresholder()
    roi = Region_of_interest()
    # color = roi.roi_mask(color)
    img = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Gray', img)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl1 = clahe.apply(img)
    cv2.imshow('CLAHE', cl1)
    th = threshold_object.threshold(color)
    # cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Original', color)
    cv2.namedWindow('Combined', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Combined', threshold_object.threshold(color))

    cv2.waitKey(0)
    cv2.destroyAllWindows()
