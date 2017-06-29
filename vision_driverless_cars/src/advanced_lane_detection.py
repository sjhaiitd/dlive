import cv2
import numpy as np
from thresholder import Thresholder
from warper import Warper
from polyfitter import Polyfitter
from polydrawer import Polydrawer


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    threshold_object = Thresholder()
    warp_object = Warper()
    fitt_object = Polyfitter()
    draw_object = Polydrawer()

    while(cap.isOpened()):
        ret, color = cap.read()
        print color.shape
        th = threshold_object.threshold(color)
        # cv2.imshow('Combined', threshold_object.threshold(color))
        # cv2.imshow('Warped', warp_object.warp(th))
        img = warp_object.warp(th)
        left_fit, right_fit = fitt_object.polyfit(img)
        print left_fit, right_fit
        img = draw_object.draw(color, left_fit, right_fit, warp_object.Minv)
        cv2.imshow('Lane Detection', img)
        if cv2.waitKey(1) & 0xFF == 27:
            break

cv2.destroyAllWindows()
