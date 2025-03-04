#!/usr/bin/python
# -*- coding: UTF-8 -*-


###########################################
#   Autores: Daniel Cay (741066)          #
#            Javier Fañanás (737987)      #
#            David Solanas (738630)       #
#                                         #
#   Fichero: Robot.py                     #
#   Robótica - Práctica 3                 #
###########################################


from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np
import brickpi3
import cv2
import picamera
from picamera.array import PiRGBArray

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        # Robot construction parameters
        self.R = 26 # mm
        self.L = 140 # mm

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B-right and C-left (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B,
            self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C,
            self.BP.get_motor_encoder(self.BP.PORT_C))

        # reset encoder A-hook
        self.BP.offset_motor_encoder(self.BP.PORT_A,
            self.BP.get_motor_encoder(self.BP.PORT_A))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # Initialize camera
        self.cam = picamera.PiCamera()
        self.cam.resolution = (640, 480)
        self.cam.framerate = 32
        # It'll contain the photo
        self.rawCapture = PiRGBArray(self.cam, size=(640, 480))

        # Configure for an NXT ultrasonic sensor.
        # BP.set_sensor_type configures the BrickPi3 for a specific sensor.
        # BP.PORT_1 specifies that the sensor will be on sensor port 1.
        # BP.SENSOR_TYPE.NXT_ULTRASONIC specifies that the sensor will be an NXT ultrasonic sensor.
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM) # Configure for an EV3 ultrasonic sensor.
        # odometry update period --> UPDATE value!
        self.P = .005 # 5 ms
        
        # allow the camera to warmup
        time.sleep(.10)



    def setSpeed(self, v,w):
        """ Establece la velocidad del robot v (mm/s), w(rad/s) """
        print()
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        wd = 1.0/self.R * v + self.L/(2.0 * self.R) * w
        wi = 1.0/self.R * v - self.L/(2.0 * self.R) * w
        
        speedDPS_right = 180. * wd / np.pi
        speedDPS_left = 180. * wi / np.pi
        
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_right)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_left)


    def readSpeed(self):
        """ To be filled"""

        return 0,0


    def readOdometry(self):
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        return x, y, th


    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)


    def norm_pi(self, th):
        """ Normalizes 'th' to be in range [-pi, pi] """
        th_norm = th
        while th_norm > np.pi:
            th_norm -= 2 * np.pi

        while th_norm < -np.pi:
            th_norm += 2 * np.pi

        return th_norm

    
    def updateOdometry(self):
        """ Reads the encoders and updates the position of the robot \
            X, Y, Theta, it also stores a log with the robot's position. """

        ant_enconder_l = 0.
        ant_enconder_r = 0.
        
        # Opens the log file
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

            
                dSl = 2. * np.pi * self.R * (encoder_l - ant_enconder_l) / 360.
                dSr = 2. * np.pi * self.R * (encoder_r - ant_enconder_r) / 360.

                dS = (dSl + dSr) / 2.
                dth = (dSr - dSl) / self.L 
                
                x, y, th = self.readOdometry()

                dx = dS * np.cos(th + (dth / 2.))
                dy = dS * np.sin(th + (dth / 2.))

                # Normalizes the angle, th always in the range [-pi, pi]
                dth = self.norm_pi(th + dth)

                self.lock_odometry.acquire()
                self.x.value += dx
                self.y.value += dy
                self.th.value = dth
                self.lock_odometry.release()

                ant_enconder_l = encoder_l
                ant_enconder_r = encoder_r

            except IOError as error:
                #print(error)
                sys.stdout.write(error)


            # save LOG
            # Stores X, Y and theta
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


    def rot(self, th_goal, sign=0, offset=0.07):
        _, _, th = self.readOdometry()
        if sign != 0:
            self.setSpeed(0, sign*np.pi / 3)
        else:
            if th < th_goal:
                self.setSpeed(0, np.pi / 3)
            else:
                self.setSpeed(0, -np.pi / 3)
        stop = th_goal - offset < th < th_goal + offset
        while not stop:
            time.sleep(0.005)
            _, _, th = self.readOdometry()
            stop = th_goal - offset < th < th_goal + offset
        
        return




    # Get an image
    def get_photo(self):
        """ Captures an image and converts to HSV colorspace """
        self.cam.capture(self.rawCapture, 'bgr')
        frame = self.rawCapture.array
        # cv2.imshow('imagen', frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.rawCapture.truncate(0)
        # cv2.waitKey()
        return frame


    def detect_blobs(self, frame, redMin1=(0, 70, 50), redMax1=(10, 255, 255),
                     redMin2=(170, 70, 50), redMax2=(180, 255, 255)):
        """ Given an image and color ranges in HSV colorspace, detects blobs
            in the image that corresponds to that color """
        params = cv2.SimpleBlobDetector_Params()
        # These are just examples, tune your own if needed
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Color
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        #params.blobColor = 0
        params.filterByConvexity = False
        params.filterByInertia = False


        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        # filter certain RED COLOR channels

        # Pixels with 0 <= H <= 10, 170 <= H <= 180, 
        # 70 <= S <= 255, 50 <= V <= 255 will be considered red.
        # Create RED mask in HSV colorspace
        mask_red1=cv2.inRange(frame, redMin1, redMax1)
        mask_red2 = cv2.inRange(frame, redMin2, redMax2)
        mask_red = mask_red1 + mask_red2
        
        # detector finds "dark" blobs by default
        keypoints_red = detector.detect(mask_red)

        # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
        kp_size = -1
        kp_ball = None
        # Stores the blob with biggest area
        for kp in keypoints_red:
            if kp.size > kp_size:
                kp_ball = kp
                kp_size = kp.size

        if kp_ball is None or kp_size < 14:
            return False, -1, -1, -1

        return True, kp_ball.pt[0], kp_ball.pt[1], kp_ball.size


    def search_ball(self, w):
        """ Rotates the robot at w rad/s until it recognises a red ball,
            when the red ball is detected the robot stops """
        found = False
        # Maximum number of loops
        count = 0
        while not found and count < (10 * np.pi):
            frame = self.get_photo()
            found, x_blob, y_blob, area_blob = self.detect_blobs(frame)

            if not found:
                self.setSpeed(0, w)
            else:
                self.setSpeed(0,0)
                break
            count += abs(w) * .002
            time.sleep(.002)

        return x_blob, y_blob, area_blob


    def trackObject(self, colorRange=[(0, 70, 50), (10, 255, 255), (170, 70, 50), (180, 255, 255)]):
        """ Tracks the object with a specific color 'colorRange', it follows the object
            taking pictures and searching for blobs with that color. When it founds one,
            the robot approaches to the blob until it is close enough to the blob, then
            the robot centers the blob and stop. """
        x_blob = -1
        y_blob = -1
        area_blob = -1
        # Variable to know where to rotate when the blob is lost
        x_blob_ant = -1
        # First search the blob
        x_blob, y_blob, area_blob = self.search_ball(-np.pi / 3)
        stop_y = False
        stop_x = False

        # Approach to the blob until it's close enough
        while (not stop_y) or (not stop_x):
            # Get a photo
            frame = self.get_photo()
            # Search the blob and get its coordinates in the image
            visible, x_blob, y_blob, area_blob = self.detect_blobs(frame)
            print(visible, x_blob, y_blob, area_blob)
            # Check if is visible or not, if not then search the blob again
            if not visible:
                # Check if the robot has to rotate to left or right
                if x_blob_ant > 320:
                    # Rotate to right
                    w = -np.pi / 3
                else:
                    # Rotate to left
                    w = np.pi / 3
                # Search again the blob
                x_blob, y_blob, area_blob = self.search_ball(w)

            # Calculate the offset of the blob from the center of the image
            offset = 320 - x_blob

            # Recalculate stop conditions
            stop_y = y_blob > 370
            stop_x = abs(offset) < 10

            # Assign the velocities, offset * .002 because .002 is the period of
            # the loop
            w = offset * .001 if not stop_x else 0
            v = 100 if not stop_y else 0
            # Set the robot's speed
            self.setSpeed(v, w)
            x_blob_ant = x_blob

            # Sleep .002 seconds
            time.sleep(.001)
        
        # In this point the robot is close enough to the blob, recenter the robot
        # and get a little closer to the blob. Fitst of all, stop the robot
        self.setSpeed(0, 0)
        time.sleep(0.001)

        # Get a little closer to the blob
        self.setSpeed(155, 0)
        time.sleep(1.1)
        self.setSpeed(0, 0)


    def catch(self):
        """ Catches the ball dropping the box """
        self.BP.set_motor_dps(self.BP.PORT_A, -120)
        time.sleep(1)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)


    def release(self):
        """ Releases the ball lifting the box """
        self.BP.set_motor_dps(self.BP.PORT_A, 120)
        time.sleep(1)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)


    def get_distance_sonar(self):
        """ Returns the distance from the robot to an obstacle
            located in the front of him in cm """
        value = self.BP.get_sensor(self.BP.PORT_1)
        return value

    def detectObstacle(self, max_=25):
        """ Returns true if the sonar detects an object """
        return 2 < self.get_distance_sonar() < max_

     
 
    def match_images(self, img1_bgr, img2_bgr):
        """ Given img1_bgr as reference, search matches with img2_bgr and if they are found,
            returns True and their coordinates, otherwise returns False. """
        # max number of features to extract per image
        MAX_FEATURES = 500
        # REQUIRED number of correspondences (matches) found:
        MIN_MATCH_COUNT=20          # initially
        MIN_MATCH_OBJECTFOUND=9    # after robust check, to consider object-found

        # Feature extractor uses grayscale images
        img1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)
        
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
            # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
            detector = cv2.ORB()

        else: 
            # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
            # but this is the best performing one in this version
            detector = cv2.BRISK_create()
            

        # find the keypoints and corresponding descriptors
        kp1, des1 = detector.detectAndCompute(img1,None)
        kp2, des2 = detector.detectAndCompute(img2,None)

        if des1 is None or des2 is None:
            print("WARNING: empty detection?")
            return False, None
        if len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
            print("WARNING: not enough FEATURES (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            return False, None
        print(" FEATURES extracted (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1,des2)
        matches = sorted(matches, key = lambda x:x.distance)
        good = matches

        print(" Initial matches found: %d" %(len(good)))
        dst = None

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            H_21, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
            matchesMask = mask.ravel().tolist()
            num_robust_matches = np.sum(matchesMask)
            if num_robust_matches < MIN_MATCH_OBJECTFOUND:
                found = False
                print("NOT enough ROBUST matches found - %d (required %d)" % 
                    (num_robust_matches, MIN_MATCH_OBJECTFOUND))
                return found, dst

            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,H_21)
            img2_res = cv2.polylines(img2_bgr, [np.int32(dst)], True, 
                                    color=(255,255,255), thickness=3)
            found = True
            print("ROBUST matches found - %d (out of %d) --> OBJECT FOUND" % (np.sum(matchesMask), len(good)))
        else:
            print("Not enough initial matches are found - %d (required %d)" % (len(good), MIN_MATCH_COUNT))
            matchesMask = None
            found = False

        return found, dst
    
    
    def find_template(self, img_captured=None, imReference=None):
        """ Given a two images search matches with function match_images and return if founded,
            or not and its coordinates. """
        img = cv2.cvtColor(img_captured, cv2.COLOR_HSV2BGR)
        found, pts = self.match_images(imReference, img)
        return found, pts
