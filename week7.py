import cv2
import math
import Camera
import numpy as np
import time
#size of the images
size = (640, 480)
#class for finding the blocks
class FindColor(object):
    #initialize
    def __init__(self, targcolor, img):
        self.targetcolor = targcolor
        self.image = img
        self.imageForProcessing = []
    #function to augment the image for processing
    def AugmentImage(self):
        #copy the image
        img_copy = self.image.copy()
        img_h, img_w = self.image.shape[:2]
        #resize the image
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        #blur the image
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #convert image for processing
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        #store image for processing
        self.imageForProcessing = frame_lab
    #function to find the blocks
    def findcolor(self):
        #get image for processing
        img = self.imageForProcessing
        #get the not augmented image for showing to the user
        imgcopy = self.image
        #default values
        botrange = [0, 0, 0]
        toprange = [255, 255, 255]
        #different ranges to pick up each color
        if self.targetcolor == 'green':
            botrange = np.array([0, 0, 0])
            toprange = np.array([255, 115, 255])
        elif self.targetcolor == 'red':
            botrange = np.array([0, 151, 100])
            toprange = np.array([255, 255, 255])
        elif self.targetcolor == 'blue':
            botrange = np.array([0, 0, 0])
            toprange = np.array([255, 255, 110])
        else:
            print("Color not recognized. Valid color choices are red, green, or blue.")
        #make a mask for the color being detected
        frame_mask = cv2.inRange(img, botrange, toprange)
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = 0
        #go through the mask and find the largest area detected
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c
        #if an area is detected, label it
        #the if is necessary because the camera takes a few seconds to connect
        if contours:
            rect = cv2.minAreaRect(area_max_contour)
            box = np.int0(cv2.boxPoints(rect))
            if self.targetcolor == 'green':
                cv2.drawContours(imgcopy, [box], -1, [0, 255, 0])
            if self.targetcolor == 'red':
                cv2.drawContours(imgcopy, [box], -1, [0, 0, 255])
            if self.targetcolor == 'blue':
                cv2.drawContours(imgcopy, [box], -1, [255, 0, 0])
        cv2.imshow("img", imgcopy)

#run the code
if __name__ == '__main__':
    starttime = time.time()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            #looking for each block for 6 seconds to approximate colorsorting.py
            if time.time() - starttime < 6:
                a = FindColor('red', frame)
                a.AugmentImage()
                a.findcolor()
            if 6 < time.time() - starttime < 12:
                a = FindColor('green', frame)
                a.AugmentImage()
                a.findcolor()
            if 12 < time.time() - starttime < 18:
                a = FindColor('blue', frame)
                a.AugmentImage()
                a.findcolor()
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()


