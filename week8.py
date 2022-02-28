import week7
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
class MoveArm(object):
    def __init__(self,targetcolor,img):
        self.targetcolor=targetcolor
        self.img=img
        self.stable=False
        self.coords=[]
        self.rect=[]
    def boxnotmoving(self):
        ystable=False
        xstable=False
        #check 5 times over 2 seconds
        a=week7.FindColor(self.targetcolor,self.img)
        #find the roi
        a.AugmentImage()
        a.findcolor()
        a.findroi()
        if len(a.roi)>0:
            img_centerx, img_centery = getCenter(a.rect, a.roi, size, square_length)
            c1=[img_centerx,img_centery]
            time.sleep(.4)
            a.findroi()
            img_centerx, img_centery = getCenter(a.rect, a.roi, size, square_length)
            c2=[img_centerx,img_centery]
            time.sleep(.4)
            a.findroi()
            img_centerx, img_centery = getCenter(a.rect, a.roi, size, square_length)
            c3 =[img_centerx,img_centery]
            time.sleep(.4)
            a.findroi()
            img_centerx, img_centery = getCenter(a.rect, a.roi, size, square_length)
            c4 =[img_centerx,img_centery]
            time.sleep(.4)
            a.findroi()
            img_centerx, img_centery = getCenter(a.rect, a.roi, size, square_length)
            c5 =[img_centerx,img_centery]
            axc=c1[0]+c2[0]+c3[0]+c4[0]+c5[0]
            axc=axc/5
            ayc=c1[1]+c2[1]+c3[1]+c4[1]+c5[1]
            ayc=ayc/5
            if axc-10<c1[0]<axc+10 and axc-10<c2[0]<axc+10 and axc-10<c3[0]<axc+10 and axc-10<c4[0]<axc+10 and axc-10<c5[0]<axc+10:
                xstable=True
            if ayc-10<c1[1]<ayc+10 and ayc-10<c2[1]<ayc+10 and ayc-10<c3[1]<ayc+10 and ayc-10<c4[1]<ayc+10 and ayc-10<c5[1]<ayc+10:
                ystable=True
            self.rect=a.rect
        if ystable and xstable:
            self.stable=True
            self.coords=[axc,ayc]
    def movetoblock(self):
        coords=self.coords
        if len(coords)>0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            AK.setPitchRangeMoving((world_x+2.5, world_x-2 , 5), -90, -90, 0)
            time.sleep(1.5)
    def grasp(self):
        #open gripper
        Board.setBusServoPulse(1, servo1 - 280, 500)
        coords=self.coords
        if len(coords)>0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            rotation_angle = self.rect[2]
            servo2_angle = getAngle(world_x, world_y, rotation_angle)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.8)
            AK.setPitchRangeMoving((world_x, world_y, 2), -90, -90, 0, 1000)
            time.sleep(2)
            Board.setBusServoPulse(1, servo1, 500)
            time.sleep(1)
    def placeblock(self):
        coords = self.coords
        if len(coords) > 0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            Board.setBusServoPulse(2, 500, 500)
            AK.setPitchRangeMoving((world_x, world_x, 12), -90, -90, 0, 1000)
            time.sleep(1)
            if self.targetcolor == "red":
                coordinate = [-14.5, 11.5]
            if self.targetcolor == "green":
                coordinate = [-14.5, 5.5]
            if self.targetcolor == "blue":
                coordinate = [-14.5, -.5]
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 12), -90, -90, 0)
            servo2_angle = getAngle(coordinate[0], coordinate[1], -90)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.5)
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 4.5), -90, -90, 0, 500)
            time.sleep(0.5)
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 1.5), -90, -90, 0, 1000)
            time.sleep(0.8)
            Board.setBusServoPulse(1, servo1 - 200, 500)
            time.sleep(.5)

    def placeblockpal(self):
        coordinate = [-14, -7.5, 1.5]
        coords = self.coords
        if len(coords) > 0:
            world_x, world_y = convertCoordinate(coords[0], coords[1], size)
            Board.setBusServoPulse(2, 500, 500)
            AK.setPitchRangeMoving((world_x, world_x, 12), -90, -90, 0, 1000)
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 12), -90, -90, 0)
            servo2_angle = getAngle(coordinate[0], coordinate[1], -90)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.5)
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 4.5), -90, -90, 0, 500)
            time.sleep(0.5)
            AK.setPitchRangeMoving((coordinate[0], coordinate[1], 1.5), -90, -90, 0, 1000)
            time.sleep(0.8)
            Board.setBusServoPulse(1, servo1 - 200, 500)
            time.sleep(.5)

if __name__ == '__main__':
    my_camera = Camera.Camera()
    my_camera.camera_open()
    #choose which script to emulate
    choice = input("tracking, sorting, or palletizing?(T,S,or P)")
    if choice == "T":
        frame = []
        a = MoveArm("red", frame)
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(2)
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                a.img = frame
                a.boxnotmoving()
                a.movetoblock()
                time.sleep(1.5)
                a.grasp()
                time.sleep(1)
                a.placeblock()
                time.sleep(2)
            key = cv2.waitKey(1)
        if key == 27:
            break
    if choice=='P':
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        frame=[]
        a=MoveArm("red",frame)
        b=MoveArm("green",frame)
        c=MoveArm("blue",frame)
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(2)
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                a.img=frame
                a.boxnotmoving()
                a.movetoblock()
                time.sleep(1.5)
                a.grasp()
                time.sleep(1)
                a.placeblockpal()
                time.sleep(2)
            img = my_camera.frame
            if img is not None:
                Board.setBusServoPulse(1, servo1 - 50, 300)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                b.img=frame
                b.boxnotmoving()
                b.movetoblock()
                time.sleep(1.5)
                b.grasp()
                time.sleep(1)
                b.placeblockpal()
                time.sleep(2)
            img = my_camera.frame
            if img is not None:
                Board.setBusServoPulse(1, servo1 - 50, 300)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                c.img=frame
                c.boxnotmoving()
                c.movetoblock()
                time.sleep(1.5)
                c.grasp()
                time.sleep(1)
                c.placeblockpal()
                time.sleep(2)
            key = cv2.waitKey(1)
            if key == 27:
                break
    if choice=='S':
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        frame=[]
        a=MoveArm("red",frame)
        b=MoveArm("green",frame)
        c=MoveArm("blue",frame)
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(2)
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                a.img=frame
                a.boxnotmoving()
                a.movetoblock()
                time.sleep(1.5)
                a.grasp()
                time.sleep(1)
                a.placeblock()
                time.sleep(2)
            img = my_camera.frame
            if img is not None:
                Board.setBusServoPulse(1, servo1 - 50, 300)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                b.img=frame
                b.boxnotmoving()
                b.movetoblock()
                time.sleep(1.5)
                b.grasp()
                time.sleep(1)
                b.placeblock()
                time.sleep(2)
            img = my_camera.frame
            if img is not None:
                Board.setBusServoPulse(1, servo1 - 50, 300)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                c.img=frame
                c.boxnotmoving()
                c.movetoblock()
                time.sleep(1.5)
                c.grasp()
                time.sleep(1)
                c.placeblock()
                time.sleep(2)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()



