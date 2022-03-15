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
AK = ArmIK()
size = (640, 480)
servo1 = 500
#class for rolling a die
class RollDice(object):
    def __init__(self, img):
        self.img = img
        self.imgtoshow = img
        self.coords = []
        self.rect = []
        self.bestcontour = []
        self.roi = []
        self.stable = False
        self.count = []
#function to find the die
    def FindDice(self):
        img = self.img
        #copy to draw on
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        #range that segments die
        botrange = np.array([170, 160, 130])
        toprange = np.array([255, 255, 255])
        #get mask of range
        frame_mask = cv2.inRange(img, botrange, toprange)
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = [1]
        #get contous from the mask
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                # print(contour_area_temp)
                if contour_area_temp > 40:
                    area_max_contour = c
        #get biggest contour
        if contours:
            self.bestcontour = area_max_contour
            if len(area_max_contour) > 1:
                rect = cv2.minAreaRect(area_max_contour)
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(img_copy, [box], -1, [0, 255, 0])
                self.rect = rect
    #get roi from contour
    def findroi(self):
        if len(self.bestcontour) > 1:
            self.rect = cv2.minAreaRect(self.bestcontour)
            box = np.int0(cv2.boxPoints(self.rect))
            roi = getROI(box)
            self.roi = roi
    #check that the die isnt moving
    def StableCheck(self):
        ystable = False
        xstable = False
        a = RollDice(self.img)
        # check 5 times over 1 second
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
            #check that all 5 centers are close to the average
            if axc - 10 < c1[0] < axc + 10 and axc - 10 < c2[0] < axc + 10 and axc - 10 < c3[0] < axc + 10 and axc - 10 < c4[0] < axc + 10 and axc - 10 < c5[0] < axc + 10:
                xstable = True
            if ayc - 10 < c1[1] < ayc + 10 and ayc - 10 < c2[1] < ayc + 10 and ayc - 10 < c3[1] < ayc + 10 and ayc - 10 < c4[1] < ayc + 10 and ayc - 10 < c5[1] < ayc + 10:
                ystable = True
            self.rect = a.rect
            # print(self.rect,self.roi)
        if ystable and xstable:
            self.stable = True
            box = np.int0(cv2.boxPoints(self.rect))
            #get the real center of the rectangle
            cx = box[0][0] + box[1][0] + box[2][0] + box[3][0]
            cx = cx / 4
            cy = box[0][1] + box[1][1] + box[2][1] + box[3][1]
            cy = cy / 4
            #set as coordinates
            self.coords = [cx, cy]
    #send the arm to home position
    def GoHome(self):
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
    #function for grabbing the die
    def GrabDice(self):
        # open gripper
        Board.setBusServoPulse(1, servo1 - 280, 500)
        coords = [0, 0]
        #adjust coordinates for the camera being off
        if len(self.coords) > 0:
            coords[0] = self.coords[0] - 40
            coords[1] = self.coords[1] + 100
        #garb die at this location
        if coords[0] > 0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            rotation_angle = self.rect[2]
            servo2_angle = getAngle(world_x, world_y, rotation_angle)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.8)
            AK.setPitchRangeMoving((world_x, world_y, 2), -90, -90, 0, 1000)
            time.sleep(1.5)
            Board.setBusServoPulse(1, servo1 + 40, 500)
            time.sleep(.5)
    #function to roll the die
    def Roll(self):
        #stay gripped tightly
        Board.setBusServoPulse(1, 540, 500)
        time.sleep(0.8)
        #go to the middle of the mat
        AK.setPitchRangeMoving((0, 20, 6), -90, -90, 0, 1000)
        time.sleep(1.5)
        #random angle to rotate to
        rollang = np.random.randint(300, 650)
        #rotate to this angle
        Board.setBusServoPulse(2, rollang, 500)
        time.sleep(.2)
        #drop the die
        Board.setBusServoPulse(1, 200, 500)
        time.sleep(.3)
    #function to count the die
    def Count(self):
        #get image
        img=self.img
        #if a die has been detected
        if len(self.coords)>0:
            #get the coordinates
            x=int(self.coords[0])
            y=int(self.coords[1])
            #values to zoom to
            xtop=x+30
            xbot=x-30
            ytop=y+30
            ybot=y-30
            #can't crop outside of the image
            if xtop>639:
                xtop=639
            if xbot<0:
                xbot=0
            if ytop>479:
                ytop=479
            if ybot<0:
                ybot=0
            #crop to the die
            img=img[ybot:ytop, xbot:xtop]
            #copy of image to draw on
            imgdraw=img.copy()
            #convert image to gray to use with hough transforms
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,1,param1=6,param2=11,minRadius=1,maxRadius=6)
            #get rid of overlapping circles
            def overlap(circ):
                #get the circles
                circ=circ[0]
                count=0
                #get the radii and center coordinates
                rad=circ[:,2]
                centx=circ[:,1]
                centy=circ[:,0]
                #compare each circle to the following circles
                for i in range(len(circ)-1):
                    #get the circle's info
                    rad1=rad[i]
                    cx1=centx[i]
                    cy1=centy[i]
                    #compare to other circles
                    for j in range(i+1,len(circ)):
                        rad2=int(rad[j])
                        cx2=int(centx[j])
                        cy2=int(centy[j])
                        dist=int(rad1+rad2)
                        if cx1-dist<cx2<cx1+dist and cy1-dist<cy2<cy1+dist:
                            #get rid of overlapping circles
                            centx[j]=0
                            centy[j]=0
                            rad[j]=0
                            count=count+1
                #start an array to store the final circles in
                newcircles=np.zeros([3,len(centy)])
                for q in range(len(centy)):
                    newcircles[0,q]=centy[q]
                    newcircles[1,q]=centx[q]
                    newcircles[2,q]=rad[q]
                #calculate the number of circles
                count=len(circ)-count
                return newcircles, count
            #if circles were found
            if circles is not None:
                circles = np.uint16(np.around(circles))
                #get rid of overlaps and count the circles
                circles, numcirc=overlap(circles)
                #print the count
                print("count: ", numcirc)
                self.count=numcirc
                #draw the circles
                for i in range(len(circles[0,:])):
                    # draw the outer circle
                    cv2.circle(imgdraw,(int(circles[0,i]),int(circles[1,i])),int(circles[2,i]),(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(imgdraw,(int(circles[0,i]),int(circles[1,i])),2,(0,0,255),3)
                #write the number of circles on the image
                font  = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (5,50)
                fontScale   = 2
                fontColor   = (255,0,255)
                thickness    = 3
                lineType     = 2
                cv2.putText(imgdraw,str(self.count), bottomLeftCornerOfText, font, fontScale,fontColor,thickness,lineType)
                #save the image to show
                self.imgtoshow=imgdraw
                cv2.imshow("test",imgdraw)

if __name__ == '__main__':
    my_camera = Camera.Camera()
    my_camera.camera_open()
    time.sleep(4)
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            a = RollDice(frame)
            #find the die
            a.FindDice()
            time.sleep(1.5)
            a.findroi()
            time.sleep(1.5)
            #check if it's moving
            a.StableCheck()
            time.sleep(1.5)
            #count the number displayed
            a.Count()
            #try to show another image. I don't understand why, but this made it so that the relevant image was displayed in a timely manner
            cv2.imshow("img", a.imgtoshow)
            time.sleep(8)
            #pick up the die
            a.GrabDice()
            time.sleep(1.5)
            #roll the die
            a.Roll()
            time.sleep(2)
            #go to home position
            a.GoHome()
            time.sleep(1.5)
            cv2.destroyAllWindows()
            time.sleep(1)
        key = cv2.waitKey(1)
        if key == 27:
            break
    my_camera.camera_close()
    cv2.destroyAllWindows()




