import sys

sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
#set costants
AK = ArmIK()
size = (640, 480)
servo1 = 500
#this project makes the arm roll a singular die and count the number rolled
#this class contains the functions to complete the project
class RollDice(object):
    def __init__(self, img):
        self.img = img
        self.imgtoshow = []
        self.coords = []
        self.rect = []
        self.bestcontour = []
        self.roi = []
        self.stable = False

    def FindDice(self):
        #get image
        img = self.img
        #make  copy to draw on
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        #color ranges to segment the die from the background
        botrange = np.array([170, 160, 130])
        toprange = np.array([255, 255, 255])
        #get masks from this range
        frame_mask = cv2.inRange(img, botrange, toprange)
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = [1]
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                # print(contour_area_temp)
                if contour_area_temp > 40:
                    area_max_contour = c
        #select the largest blob that is in the color range as the die
        if contours:
            self.bestcontour = area_max_contour
            if len(area_max_contour) > 1:
                rect = cv2.minAreaRect(area_max_contour)
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(img_copy, [box], -1, [0, 255, 0])
                self.rect = rect
    #get the region of interest from the largest contour
    def findroi(self):
        if len(self.bestcontour) > 1:
            self.rect = cv2.minAreaRect(self.bestcontour)
            box = np.int0(cv2.boxPoints(self.rect))
            roi = getROI(box)
            self.roi = roi
    #make sure the die is not moving
    def StableCheck(self):
        ystable = False
        xstable = False
        a = RollDice(self.img)
        # check the die location 5 times over 1 seconds
        if len(self.roi) > 0:
            a.FindDice()
            a.findroi()
            #get the center
            img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)
            c1 = [img_centerx, img_centery]
            time.sleep(.2)
            a.FindDice()
            a.findroi()
            img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)
            c2 = [img_centerx, img_centery]
            time.sleep(.2)
            a.FindDice()
            a.findroi()
            img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)
            c3 = [img_centerx, img_centery]
            time.sleep(.2)
            a.FindDice()
            a.findroi()
            img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)
            c4 = [img_centerx, img_centery]
            time.sleep(.2)
            a.FindDice()
            a.findroi()
            img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)
            c5 = [img_centerx, img_centery]
            axc = c1[0] + c2[0] + c3[0] + c4[0] + c5[0]
            axc = axc / 5
            ayc = c1[1] + c2[1] + c3[1] + c4[1] + c5[1]
            ayc = ayc / 5
            #check that none of the centers are far from the average
            if axc - 10 < c1[0] < axc + 10 and axc - 10 < c2[0] < axc + 10 and axc - 10 < c3[
                0] < axc + 10 and axc - 10 < c4[0] < axc + 10 and axc - 10 < c5[0] < axc + 10:
                xstable = True
            if ayc - 10 < c1[1] < ayc + 10 and ayc - 10 < c2[1] < ayc + 10 and ayc - 10 < c3[
                1] < ayc + 10 and ayc - 10 < c4[1] < ayc + 10 and ayc - 10 < c5[1] < ayc + 10:
                ystable = True
            self.rect = a.rect
        if ystable and xstable:
            self.stable = True
            #get the coordinates of the die
            self.coords = [axc, ayc]
    #send the arm back to starting position
    def GoHome(self):
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
    #pick up the die
    def GrabDice(self):
        # open gripper
        Board.setBusServoPulse(1, servo1 - 280, 500)
        coords = [0, 0]
        if len(self.coords) > 0:
            #adjust the coordinates to account for the error of the camera
            coords[0] = self.coords[0] - 70
            coords[1] = self.coords[1] + 80
        if coords[0] > 0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            rotation_angle = self.rect[2]
            #straighten gripper
            Board.setBusServoPulse(2, 500, 500)
            time.sleep(0.8)
            #move arm to coordinates
            AK.setPitchRangeMoving((world_x, world_y, 2), -90, -90, 0, 1000)
            time.sleep(1.5)
            #tighten gripper
            Board.setBusServoPulse(1, servo1 + 40, 500)
            time.sleep(.5)
    #roll the dice
    def Roll(self):
        #keep the grasper clasped
        Board.setBusServoPulse(1, 540, 500)
        time.sleep(0.8)
        #go toa  set height and location
        AK.setPitchRangeMoving((0, 20, 5), -90, -90, 0, 1000)
        time.sleep(1.5)
        #set a random angle to spin the hand
        rollang = np.random.randint(300, 650)
        #roll to this angle
        Board.setBusServoPulse(2, rollang, 500)
        time.sleep(.2)
        #drop the dice
        Board.setBusServoPulse(1, 200, 500)
        time.sleep(.3)
    #count the rolled number
    def Count(self):
        #get image
        img = self.img
        #if there is a die
        if len(self.coords) > 0:
            #adjust calculated center
            x = int(self.coords[0]) - 20
            y = int(self.coords[1]) - 20
            #crop the image down to the die
            xtop = x + 30
            xbot = x - 30
            ytop = y + 30
            ybot = y - 30
            #can't crop outside of the image
            if xtop > 639:
                xtop = 639
            if xbot < 0:
                xbot = 0
            if ytop > 479:
                ytop = 479
            if ybot < 0:
                ybot = 0
            #crop the image
            img = img[ybot:ytop, xbot:xtop]
            #make copy to draw on
            imgdraw = img.copy()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #get Hough circles
            circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 1,
                                       param1=5, param2=10, minRadius=1, maxRadius=6)
            #function to delete overlapping circles
            def overlap(circ):
                circ = circ[0]
                count = 0
                #get the center and radius of the circles
                rad = circ[:, 2]
                centx = circ[:, 1]
                centy = circ[:, 0]
                #compare each circle to all the cicrles that follow it
                for i in range(len(circ) - 1):
                    rad1 = rad[i]
                    cx1 = centx[i]
                    cy1 = centy[i]
                    for j in range(i + 1, len(circ)):
                        rad2 = rad[j]
                        cx2 = centx[j]
                        cy2 = centy[j]
                        dist = rad1 + rad2
                        if cx1 - dist < cx2 < cx1 + dist and cy1 - dist < cy2 < cy1 + dist:
                            #if the circles overlap, set one of them equal to 0
                            centx[j] = 0
                            centy[j] = 0
                            rad[j] = 0
                            count = count + 1
                #array to store more circles
                newcircles = np.zeros([3, len(centy)])
                for q in range(len(centy)):
                    newcircles[0, q] = centy[q]
                    newcircles[1, q] = centx[q]
                    newcircles[2, q] = rad[q]
                count = len(circ) - count
                return newcircles, count
            if circles is not None:
                #get the final circles
                circles = np.uint16(np.around(circles))
                circles, numcirc = overlap(circles)
                print("count: ", numcirc)
                for i in range(len(circles[0, :])):
                    # draw the outer circle
                    cv2.circle(imgdraw, (int(circles[0, i]), int(circles[1, i])), int(circles[2, i]), (0, 255, 0), 2)
                    # draw the center of the circle
                    cv2.circle(imgdraw, (int(circles[0, i]), int(circles[1, i])), 2, (0, 0, 255), 3)
                self.imgtoshow = imgdraw
                #show the die with circles drawn
                cv2.imshow("img",imgdraw)


if __name__ == '__main__':
    my_camera = Camera.Camera()
    my_camera.camera_open()
    time.sleep(6)
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            a = RollDice(frame)
            a.FindDice()
            time.sleep(1.5)
            a.findroi()
            time.sleep(1.5)
            a.StableCheck()
            time.sleep(1.5)
            a.Count()
            a.GrabDice()
            time.sleep(1.5)
            a.Roll()
            time.sleep(2)
            a.GoHome()
            cv2.imshow("img", a.imgtoshow)
            time.sleep(4)
        key = cv2.waitKey(1)
        if key == 27:
            break
    my_camera.camera_close()
    cv2.destroyAllWindows()

