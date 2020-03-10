#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import math
import brickpi3

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 26 # mm
        self.L = 152 # mm
        #self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B-right and C-left (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B,
            self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C,
            self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = .005 # 5 ms



    def setSpeed(self, v,w):
        """ Establece la velocidad del robot v (mm/s), w(rad/s) """
        print()
        print("setting speed to %.2f %.2f" % (v, w))
        print()

        # compute the speed that should be set in each motor ...

        wd = 1.0/self.R * v + self.L/(2.0 * self.R) * w
        wi = 1.0/self.R * v - self.L/(2.0 * self.R) * w
        
        speedDPS_right = 180. * wd / math.pi
        speedDPS_left = 180. * wi / math.pi
        
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_right)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_left)


    def readSpeed(self):
        """ To be filled"""

        return 0,0

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):
        """ To be filled ...  """

        ant_enconder_l = 0.
        ant_enconder_r = 0.
        
        f = open('odometry.log', 'w+')

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                [encoder_l, encoder_r] = [self.BP.get_motor_encoder(self.BP.PORT_C),
                    self.BP.get_motor_encoder(self.BP.PORT_B)]

                dSl = 2. * math.pi * self.R * (encoder_l - ant_enconder_l) / 360.
                dSr = 2. * math.pi * self.R * (encoder_r - ant_enconder_r) / 360.
                dS = (dSl + dSr) / 2. if abs((dSl + dSr) / 2.) >= 1e-6 else 0.0
                dth = (dSr - dSl) / self.L if abs((dSr - dSl / self.L)) >= 1e-6 else 0.0

                self.lock_odometry.acquire()
                x, y, th = self.readOdometry()
                self.lock_odometry.release()

                dx = dS * math.cos(th + (dth / 2.))
                dy = dS * math.sin(th + (dth / 2.))

                self.lock_odometry.acquire()
                self.x.value += dx
                self.y.value += dy
                # print(dSl, dSr, dth)
                if -math.pi < (th + dth) < math.pi:
                    self.th.value += dth
                elif -math.pi > (th + dth):
                    self.th.value = math.pi - (-math.pi - (th + dth))
                elif math.pi < (th + dth):
                    self.th.value = -math.pi + ((th + dth) - math.pi)
                
                self.lock_odometry.release()

                ant_enconder_l = encoder_l
                ant_enconder_r = encoder_r

            except IOError as error:
                #print(error)
                sys.stdout.write(error)


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########
            self.lock_odometry.acquire()
            f.write("X=  %.5f, Y=  %.5f, th=  %.5f \n" %(self.x.value, self.y.value, self.th.value))
            self.lock_odometry.release()

            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        f.close()


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()

