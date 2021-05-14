try:
    import epics
    import multi_sample12IDB as bl
    QR_PV = epics.PV(bl.QR_PV)
except:
    pass
from PyQt5.QtCore import (pyqtSignal, QObject)
from PyQt5.QtWidgets import QWidget

import time
import numpy
# conda install -c anaconda freetype
# conda install -c conda-forge ez_setup
beamlinePV = '12idc:'
beamlinePV2 = "12idb:"
trans_motor_PV = 17
vert_motor_PV = 18
magazine_motor_PV = 20
fingerPV = "9440:1:bo_0"

class RobotException(Exception):
    pass

# UR3
import os
import sys
sys.path.append(r"python-urx")

import math3d as m3d
import urx
import logging
import math
import time
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from urx.urdashboard import dashboard

def ind2sub(ind, array_shape):
    rows = int(ind / array_shape[1])
    cols = (int(ind) % array_shape[1]) # or numpy.mod(ind.astype('int'), array_shape[1])
    return (rows, cols)

def sub2ind(rows, cols, array_shape):
    return rows*array_shape[1] + cols

class UR3(QObject):
    # unit of position vector : meter.
    sigFinger = pyqtSignal(str)
    sigMoving = pyqtSignal(bool)
    sigFingerPosition = pyqtSignal(str)
    sigObject_onFinger = pyqtSignal(bool)
    sigRobotCommand = pyqtSignal(str)
    sigRobotPosition = pyqtSignal(numpy.ndarray)
    #Waypointmagup_q=[5.192646026611328, -0.6902674001506348, 0.557411019002096, -1.440483884220459, -1.5736210981952112, -1.0272372404681605]
    Waypointmagup_q = [3.915731906890869, -1.0986860555461426, 1.227795426045553, -1.702268739739889, -1.5722954908954065, 0.7979311943054199]
    Waypoint_QR_p = [ 2.06744440e-01,  2.35091449e-01,  1.53515533e-01,  2.88421769e+00, -1.24519515e+00, -3.20548384e-05]
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]

    def __init__(self, name = 'UR3'):
        super(UR3, self).__init__()
# definition of Cartesian Axis of UR3 at 12idb.
# X : positive - Out board
# X : negative - In board
# Y : positive - Along X-ray
# Y : negative - Again X-ray
        if name == 'UR3':
            IP="164.54.122.96"
        if name == 'UR5':
            IP = 'UR5-12IDC.xray.aps.anl.gov'
        self.logger = logging.getLogger(name)
        try:
            self.robot = urx.Robot(IP)
        except TimeoutError:
            self.logger.info("UR3 comm time out error")
            exit
        self.robot.IP = IP
        self.robot.set_tcp(self.tcp)
        self.robot.set_payload(0.0, (0,0,0))
        self.finger = Robotiq_Two_Finger_Gripper(self.robot)
        self.dashboard = dashboard(self.robot)
        #self.finger.gripper_activate()
        self.name = name
        self.ini_name = '%s.ini'%name
        self._path_ini_name = '%spath.ini'%name
        self.isSampleOnStage = False
        self.currentFrame = 0
        self.readini()

    def terminate(self):
        self.robot.close()
        self.udpServer.disconnect() # shutoff network

    def get_status(self):
        return self.dashboard.get_status()

    def unlock_stop(self):
        self.dashboard.unlock()

    def activate_gripper(self):
        self.finger.gripper_activate()
    def is_protective_stopped(self):
        return self.robot.is_protective_stopped()
        
    def init_waypoints(self):
        Waypointmagdn_p = self.pos_sample1
        Waypointmagup_p = [Waypointmagdn_p[0],Waypointmagdn_p[1],Waypointmagdn_p[2],
                Waypointmagdn_p[3],Waypointmagdn_p[4],Waypointmagdn_p[5]]
        Waypointmagup_p[2] = Waypointmagdn_p[2] + self.vert_magZ/1000
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000
        Waypoint_middle_p = [Waypointup_p[0],Waypointup_p[1],Waypointup_p[2],
                Waypointup_p[3],Waypointup_p[4],Waypointup_p[5]]
        Waypoint_middle_p[0] = Waypoint_middle_p[0] + self.trans_X/1000
        self.Waypointmagdn_p = self.pos_sample1
        self.Waypointmagup_p = Waypointmagup_p
        self.middl_p = m3d.Transform(Waypoint_middle_p)
        self.samup_p = m3d.Transform(Waypointup_p)
        self.samdn_p = m3d.Transform(Waypointdown_p)
        self.QR_p = m3d.Transform(self.Waypoint_QR_p)
        #self.QR_p = self.middl_p # for now default QR reading position will be here.
        self.set_magazine(0, 0)
        #self.magdn_p = magdn_p
        #self.magup_p = magup_p

    def getj(self):
        joints = self.robot.getj()
        return joints

    def rotx(self, val):
        # val in degree
        self.robot.rx = -val/180*math.pi

    def roty(self, val):
        # val in degree
        self.robot.ry = -val/180*math.pi

    def rotz(self, val):
        # val in degree
        self.robot.rz = -val/180*math.pi

    def rotate(self, axis='x', val=0.0):
        # val in degree
        if axis == 'x':
            self.rotx(val)
        if axis == 'y':
            self.roty(val)
        if axis == 'z':
            self.rotz(val)

    def set_current_as_sampledownXYonly(self):
        #self.samdn_p = self.robot.get_pose()
        newp = self.robot.getl()
        for i in range(len(newp)):
            if i==2:
                continue
            self.pos_samplestage[i] = newp[i]
#        self.pos_samplestage = self.samdn_p
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000
        self.samdn_p = m3d.Transform(Waypointdown_p)
        self.samup_p = m3d.Transform(Waypointup_p)

    def set_current_as_sampledown(self):
        self.samdn_p = self.robot.get_pose()
        self.pos_samplestage = self.robot.getl()
#        self.pos_samplestage = self.samdn_p
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000
        self.samup_p = m3d.Transform(Waypointup_p)

    def set_current_as_magazinedown(self):
        self.magdn_p = self.robot.get_pose()
        self.pos_sample1 = self.robot.getl()
        Waypointmagdn_p = self.pos_sample1
        Waypointmagup_p = [Waypointmagdn_p[0],Waypointmagdn_p[1],Waypointmagdn_p[2],
                Waypointmagdn_p[3],Waypointmagdn_p[4],Waypointmagdn_p[5]]
        Waypointmagup_p[2] = Waypointmagdn_p[2] + self.vert_magZ/1000
        #self.logger.debug("Position: %d", self.magup_p)
        self.magup_p = m3d.Transform(Waypointmagup_p)
        #self.logger.debug("Is changed to %d", self.magup_p)

    def writeini(self):
        w = []
        #print(self.ini_name, " will be overritten.")
        with open(self.ini_name) as f:
            for line in f:
                (desc, val) = line.split(',')
                (key, val) = val.split(':')
                myval = getattr(self, key.strip())
                ismultiple = False
                if isinstance(myval, list):
                    if len(myval) > 1:
                        ismultiple = True
                if ismultiple == False:
                    if isinstance(myval, str):
                        mstr = "%s, %s : %s\n" % (desc.strip(), key.strip(), myval)
                    else:
                        mstr = "%s, %s : %0.4f\n" % (desc.strip(), key.strip(), myval)
                else:
                    s = ""
                    for ind in myval:
                        s = s + "%0.4f "%ind
                    mstr = "%s, %s : %s\n" % (desc.strip(), key.strip(), s)
                #print(mstr)
                w.append(mstr)
        f = open(self.ini_name, "w")
        f.writelines(w)
        f.close()

    def readini(self):
        d = {}
        with open(self.ini_name) as f:
            for line in f:
                if len(line) < 1:
                    break
                (desc, val) = line.split(',')
                (key, val) = val.split(':')
                vallist = val.split()
                if len(vallist) == 1:
                    val = vallist[0].strip()
                    try:
                        val = float(val)
                    except:
                        pass
                else:
                    val = []
                    for i in vallist:
                        val.append(float(i))
                d[key.strip()] = val
        for key in d:
            setattr(self, key, d[key])
        self.init_waypoints()
        self.getCurrentFrameNumber()
        self.read_pathini()
        return d

    def read_pathini(self):
        d = {}
        with open(self._path_ini_name) as f:
            for line in f:
#                print(line)
                if len(line) < 1:
                    break
 #               print(line.split(','))
                (desc, val) = line.split(',')
 #               print(desc)
 #               print(val)
                (key, val) = val.split(':')
                vallist = val.split()
                if len(vallist) == 1:
                    val = float(val)
                else:
                    val = []
                    for i in vallist:
                        val.append(float(i))
                d[key.strip()] = val
        for key in d:
            setattr(self, key, d[key])
        return d

    def get_xyz(self):
        # return the Cartesian position as a numpy array
        # if you want the Cartesian position as a list, use self.robot.getl()
        pose = self.robot.get_pose()
        return pose.get_pose_vector()

    # @property
    # def whereisFinger(self): 
    #     return self._fingerpos 
    # @whereisFinger.setter 
    # def whereisFinger(self, pos): 
    #     self._fingerpos = pos 

    def grab(self):
        self.finger.close_gripper()
        #self.finger.gripper_action(255)

    def release(self):
        #self.finger.open_gripper()
        self.finger.gripper_action(120)

    def loosen(self):
        #self.finger.open_gripper()
        self.finger.gripper_action(190)

    def set_magazine(self, xN, yN):
        d = self.Waypointmagdn_p
        #u = self.Waypointmagdn_p
        newdn = [d[0],d[1],d[2],d[3],d[4],d[5]]
        newup = [d[0],d[1],d[2]+self.vert_magZ/1000, d[3],d[4],d[5]]
        newdn[0] = d[0] + xN*self.magXgap/1000
        newdn[1] = d[1] + yN*self.magYgap/1000
        newup[0] = d[0] + xN*self.magXgap/1000
        newup[1] = d[1] + yN*self.magYgap/1000
#        print(newdn)
#        print(xN, yN)
        self.magdn_p = m3d.Transform(newdn)
        self.magup_p = m3d.Transform(newup)

    def get_magazine(self):
        newdn = self.magdn_p.get_pose_vector()
        xd = (newdn[0]-self.Waypointmagdn_p[0])
        yd = (newdn[1]-self.Waypointmagdn_p[1])
        if self.magXgap is not 0:
            i = round(xd/(self.magXgap/1000), 0)
        else:
            i = 0
        if self.magYgap is not 0:
            j = round(yd/(self.magYgap/1000), 0)
        else:
            j = 0
        return (i, j)

    def transport_from_samplestage_up_to_magazine_up(self):
        #try:
        self.robot.movels([self.path3, self.path2, self.path1],radius=0.01, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=1)
        # inverse kinematics toward the magazine works....
        self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')
        #except ParsingException as ex::

    def transport_from_magazine_up_to_QR(self):
        self.robot.movel(self.QR_p, acc=0.5, vel=1)

    def transport_from_samplestage_up_to_default(self):
        self.robot.movels([self.samup_p, self.path3, self.path2, self.path1],radius=0.01, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=1)

    def transport_from_magazine_up_to_samplestage_up(self):
        self.robot.movej(self.middl_q, acc=0.5, vel=1)
        self.robot.movels([self.path1, self.path2, self.path3, self.samup_p],radius=0.01, acc=0.5, vel=0.5)

    def transport_from_default_to_samplestage_up(self):
#        self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
        self.robot.movels([self.path1, self.path2, self.path3, self.samup_p],radius=0.01, acc=0.5, vel=0.5)

    def picksample(self):
        if self.currentFrame < 0.0:
            return -1
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.transfingertomagazine()
        # # check where the finger is now..
        # wh = self.whereisFinger()
        # if wh == 'samplestage':
        #     self.transport_from_samplestage_up_to_magazine_up()
        # if wh == ('magazine', 'middle'):
        #     self.transport_from_default_to_magazine_up()
        # pos = self.robot.getl()
        # if (pos[1]>-0.03) and (pos[0]>0.0): # finger is at a quadrant where magazine is.
        #     if pos[2] < (self.magup_p.pos[2]-0.01):
        #         self.robot.up(self.magup_p.pos[2]-pos[2], acc=1.4, vel=1.0)
        # else:
        #     self.robot.movej(self.middl_q, acc=0.5, vel=0.5)

        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')

        # going down to pick up sample
        self.sigFinger.emit("Frame %i is picked up.."%self.currentFrame)
        self.pickup()
        # self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
        # self.grab()
        # self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
        # transport to the sample stage via a middle point
        self.sigFinger.emit("Being transported..")
#        self.robot.movej(self.middl_q, acc=0.5, vel=1.0) # inverse_kinematics calculation failed.
        #so I have to find out the joint position of middl.
        self.transport_from_magazine_up_to_samplestage_up()
#        self.robot.movels([self.middl_p, self.samup_p], radius=0.01, acc=0.5, vel=0.5) 
        return 0

    def scan_QR_all(self, mysample=[]):
        sam = []
        if len(mysample) == 0:
            mysample = range(0,self.mag_index)
        for ind in mysample:
            print(ind, " This is the index")
            QR_PV.put("")
            val = self.picksample_forQR(ind)
            if val == 0:
                time.sleep(1)
                qr = QR_PV.get()
#                print(ind, qr)
                sam.append([ind, qr])
                self.returnsample_fromQR()
            else:
#                print(ind)
                sam.append([ind, None])
        self.release()
#        print(sam)
        return sam

    def picksample_forQR(self, number):
        if self.currentFrame < 0.0:
            return -1
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.moveMagazine2FrameN(number)
        self.transfingertomagazine()

        # going down to pick up sample
        self.sigFinger.emit("Frame %i is picked up.."%self.currentFrame)
        self.pickup()
        if self.sample_onFinger:
            # transport to the sample stage via a middle point
            self.sigFinger.emit("Being transported..")
            self.transport_from_magazine_up_to_QR()
    #        self.robot.movels([self.middl_p, self.samup_p], radius=0.01, acc=0.5, vel=0.5) 
            return 0
        else:
            return -1
    def returnsample_fromQR(self):
        self.transfingertomagazine()
        self.putsampledown()

    def getsample(self):
        r = self.picksample()
        if r<0:
            return r
        self.putsampledown()
        self.transport_from_samplestage_up_to_default()
        # wait at the middle position for data acquisition
        self.sigFinger.emit("Frame %i is loaded. Waiting for data acquisition.."%self.currentFrame)
# #        self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
#         self.robot.movel(self.middl_p, acc=0.5, vel=0.5)
#         self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
        try:
            self.sigMoving.emit(False)
        except:
            pass
        self.isSampleOnStage = True
        #self.whereisFinger() = 'samplestage'
        return 0

    def returnsample(self, z=0.15, in_offset = None):
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.transfingertosamplestage()
        # if self.whereisFinger() == 'middle':
        #     self.transport_from_default_to_samplestage_up()
        # if self.whereisFinger() == 'magazine':
        #     self.transport_from_magazine_up_to_samplestage_up()
#        if not self.whereisFinger() in ('samplestage', 'middle'):
#            self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
#            self.robot.movel(self.middl_p, acc=1.4, vel=1.0)
        # pick up sample from the sample stage
        self.sigFinger.emit("Returning the sample first..")
#        self.robot.movels([self.samup_p, self.samdn_p], radius = 0.01, acc=0.5, vel=0.5)
#        #self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
#        self.grab()
#        self.robot.movel(self.sampup_p, acc=0.5, vel=0.5)
        self.pickup()

        # transport to the magazine via the middle point
        self.sigFinger.emit("Being transported..")
        self.transport_from_samplestage_up_to_magazine_up()
        # self.robot.movels([self.samup_p, self.middl_p], radius = 0.01, acc=0.5, vel=0.5)
        # #self.robot.movel(self.middl_p, acc=0.5, vel=0.5)
        # self.robot.movej(self.middl_q, acc=0.5, vel=0.5)

        # # inverse kinematics toward the magazine works....
        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')
        
        #rob.movep(magup_p, acc=0.5, vel=0.5, radius=0.05)

#        self.sigFinger.emit("Returned..")
        val = self.putsampledown(z=0.150, offset=in_offset)
        self.sigFinger.emit("Successfully returned..")
        # self.robot.down((self.vert_magZ-1.5)/1000, acc=1.4, vel=1.0)
        # #self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.release()

        # # move up and stop.
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

        self.isSampleOnStage = False
        try:
            self.sigMoving.emit(False)
        except:
            pass
        return val

    def moveMagazine2NextFrame(self):
        self.currentFrame = self.currentFrame + 1
        self.moveMagazine2FrameN(self.currentFrame)

    def moveMagazine2FrameN(self, number):
        self.currentFrame = number
        newN = self.mag_index[number]
        xN, yN = ind2sub(int(newN), (int(self.numXFrame), int(self.numYFrame)))
        self.set_magazine(xN, yN)
    
    def mountNextFrame(self):
        self.currentFrame = self.currentFrame + 1
        self.mountFrameN(self.currentFrame)

    def mountFrameN(self, number):
        if self.isSampleOnStage is True:
            #self.sigFinger.emit("Sample is being returned.")
            self.returnsample()
        if number > self.numFrame:
            print("%i is larger than total number of frames"%number)
            return -2
        self.sigFinger.emit("Moving the finger over to next frame.")
        self.moveMagazine2FrameN(number)
        #self.sigFinger.emit("Frame %i is being picked up."%self.currentFrame)
        rtn = self.getsample()
#        if rtn == 0:
#            self.sigFinger.emit("Frame %i is loaded successfully."%self.currentFrame)
        return rtn

    def setCurrentasFirstFrame(self):
        self.currentFrame = 1

    def getCurrentFrameNumber(self):
        i, j = self.get_magazine()
        newN = sub2ind(i, j, (int(self.numXFrame), int(self.numYFrame)))
        self.currentFrame = self.mag_index.index(newN)

    def pickuptest(self, gap=0):

        self.release()
        self.movefingerup_totransport()
        #self.transportfinger_to_magazine()
        self.transfingertomagazine()
        # if not self.whereisFinger() == 'magazine':
        #     # go to pick position
        #     self.robot.movej(self.middl_q, acc=0.5, vel=1.0)
        #     #self.robot.movels([self.middl_p, self.magup_p], acc=0.5, vel=0.5, radius=0.05)

        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')

        # going down to pick up sample
        self.pickup()
        # self.sigFinger.emit("Moving the finger down to pick up.")
        # self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.sigFinger.emit("Grabing.")
        # self.grab()
        # self.sigFinger.emit("Moving up to transport.")
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

        time.sleep(5)
        self.sigFinger.emit("Moving down to return..")
        self.putsampledown()
        # self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.sigFinger.emit("Release fingers.")
        # self.release()

        # # move up and stop.
        # self.sigFinger.emit("Moving fingers up.")
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

    def goto_default(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'goto_default'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        pos = self.robot.getl()
        pm = self.magup_p.get_pos()
        ps = self.samup_p.get_pos()
        minzval = min(pm[2], ps[2])
        if pos[2] < minzval:
            pos[2] = minzval
            self.robot.movel(pos, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=0.75)
#        self.whereisFinger()

#    def goto_magazine(self):
#        pos = self.robot.getl()
#        if (pos[1] > -0.2) and (pos[0]>0.35) and (pos[2]>0.15):
#            pass
#        else:
#            self.goto_default()
#        self.robot.set_pose(self.magup_p, acc=0.5, vel=0.75, command="movej")

    def pickup(self):
        run = 0
        if self.whereisFinger() == "nowhere":
            print("current figner position is at nowhere.")
        if self.whereisFinger() == 'samplestage':
            self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
            run = 1
        if self.whereisFinger() == 'magazine':
            self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
            run = 2
        self.grab()
        if run == 1:
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        if run == 2:
            self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
        val = self.finger.get_position()
        if val < 0.5:
            self.sample_onFinger = False
            self.sigObject_onFinger.emit(False)
        else:
            self.sample_onFinger = True
            self.sigObject_onFinger.emit(True)
#        self.whereisFinger()

    def dropofftest(self):
        if not self.whereisFinger() == 'samplestage':
            self.robot.movel(self.middl_p, acc=0.5, vel=0.5)

        # pick up sample from the sample stage
        #self.sigFinger.emit("Sample is pickup after measurement..")
        self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
        self.grab()

        # transport to the magazine via the middle point
        self.sigFinger.emit("Being transported..")
        self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        time.sleep(5)
        self.putsampledown()
        # self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)

        # # release sample
        # self.release()

        # # wait at the middle position for data acquisition
        # self.sigFinger.emit("Waiting for data acquisition..")
        # self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        # self.robot.movel(self.middl_p, acc=0.5, vel=0.5)

    def movefingerup_totransport(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerup_totransport'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        if self.whereisFinger() == 'samplestage':
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        if self.whereisFinger() == 'magazine':
            self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
#        self.whereisFinger()

    def movefingerdown_tosamplestage(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerdown_tosamplestage'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
#        self.whereisFinger()

    def movefingerdown_tomagazine(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerdown_tomagazine'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
#        self.whereisFinger()
    
    def transfingertosamplestage(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transfingertosamplestage'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        #        if self.whereisFinger() == 'samplestage':
#            self.transport_from_samplestage_up_to_samplestage_up()
        if self.whereisFinger() == 'middle':
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'nowhere':
            self.goto_default()
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'magazine':
            self.transport_from_magazine_up_to_samplestage_up()
#        self.whereisFinger()

    def transfingertomagazine(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transfingertomagazine'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        if self.whereisFinger() == 'samplestage':
            self.transport_from_samplestage_up_to_magazine_up()
        if self.whereisFinger() == 'middle':
            self.transport_from_default_to_magazine_up()
        if self.whereisFinger() == 'nowhere':
            self.goto_default()
            self.transport_from_default_to_magazine_up()
        if self.whereisFinger() == 'magazine':
            self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')
#        self.whereisFinger()

    def transport_from_default_to_magazine_up(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transport_from_default_to_magazine_up'
        self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')

    def out(self, x=0.05, acc=0.01, vel=0.01, wait=True):
        p = self.robot.getl()
        p[0] += x
        self.robot.movel(p, acc=acc, vel=vel, wait=wait)

    def along(self, y=0.05, acc=0.01, vel=0.01, wait=True):
        p = self.robot.getl()
        p[1] += y
        self.robot.movel(p, acc=acc, vel=vel, wait=wait)

    def up(self, z=0.05, acc=0.01, vel=0.01, wait=True):
        p = self.robot.getl()
        p[2] += z
        self.robot.movel(p, acc=acc, vel=vel, wait=wait)

    def down(self, z=0.05, acc=0.01, vel=0.01, wait=True):
        """
        Move down in csys z
        """
        self.up(-z, acc, vel, wait)

    def putsampledown(self, z=0.150, offset=None):
        # robot arm will go down by abs(z)-offset
        #
        if offset == None:
            if hasattr(self, 'vert_offset'):
                offset = self.vert_offset*0.001
            else:
                offset = 0.001

        if self.whereisFinger() == "nowhere":
            print("current figner position is at nowhere.")
        if self.whereisFinger() == 'samplestage':
            run = 1
            cposz = self.robot.get_pos()
            z = abs(cposz[2] - self.samdn_p.pos[2])-offset
        if self.whereisFinger() == 'magazine':
            run = 2
            cposz = self.robot.get_pos()
            z = abs(cposz[2] - self.magdn_p.pos[2])-offset
#        print("amount to move is %0.3f, offset is %0.3f"%(z, offset))
        #pos = self.robot.getl()
        #pos[2] = pos[2]-z/1000
        #self.robot.movel(pos, acc=0.05, vel=0.02, wait=False)
        self.down(z, acc=0.05, vel=0.05, wait=False)
        timeoutcnt = 0
        timeout = False
#        print('See if program is running.')
        while not self.robot.is_program_running():
            time.sleep(0.01)
            timeoutcnt = timeoutcnt + 1
            if (timeoutcnt>100):
                timeout = True
                break
#        print('Program is running.')
#        print('timeout is ', timeout)
        if timeout is False:
#            print('force to be read.')
            while self.robot.get_force() < 20:
#                time.sleep(0.01)
#                print(self.robot.get_force(), " I am going down.")
                if not self.robot.is_program_running():
                    break
#            print('what?.')
            self.robot.stopl()
        else:
#            print('pass.')
            pass
#        self.release()
        self.loosen()
        time.sleep(0.1)
        #try:
        self.robot.down(0.002)
        #except:
        #    return -1
        self.release()
        time.sleep(0.1)
        if run == 1:
            self.robot.movel(self.samup_p, acc=0.5, vel=1)
        if run == 2:
            self.robot.movel(self.magup_p, acc=0.5, vel=1)
#        self.whereisFinger()
        return 0
#        self.robot.movel(pos_org, acc=0.4, vel=0.2, wait=True)

    def whereisFinger(self):
        ang = self.robot.getj()
        v = numpy.asarray(self.middl_q)
        if sum((v-ang)*(v-ang)) < 0.01:
            return 'middle'
        pos = self.robot.get_pose()
        t = pos.pos - self.magup_p.pos
        
        t[2] = 0 # make Z 0. only compare (x, y)
        if (pos.pos[0] > 0) and (pos.pos[1]>-0.2): #somewhere sample area
            self.sigFingerPosition.emit('magazine')
            return 'magazine'
        #if t.length < 0.06:
        #    return 'magazine'
        t = pos.pos - self.samup_p.pos
        t[2] = 0 # make Z 0. only compare (x, y)
        if t.length < 0.06:
            self.sigFingerPosition.emit('samplestage')
            return 'samplestage'
        t = pos.pos - self.middl_p.pos
        t[2] = 0 # make Z 0. only compare (x, y)
        if t.length < 0.06:
            self.sigFingerPosition.emit('middle')
            return 'middle'
        else:
            self.sigFingerPosition.emit('nowhere')
            return "nowhere"