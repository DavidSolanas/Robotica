#!/usr/bin/python
# -*- coding: UTF-8 -*-


###########################################
#   Autores: Daniel Cay (741066)          #
#            Javier Fañanás (737987)      #
#            David Solanas (738630)       #
#                                         #
#   Fichero: p3_base.py                   #
#   Robótica - Práctica 3                 #
###########################################

import argparse
import cv2
import numpy as np
import time
from Robot import Robot
import math
from MapLib import Map2D


def slalom(robot, map_a):
    stop = False
    estado = 0
    while not stop:
        
        # Leer coordenadas del robot
        x, y, th = robot.readOdometry()
        print(x, y, th)
        if estado == 0:
            # estado 1, empieza la trayectoria
                estado = 1
                # Actualizar velocidad
                if map_a:
                    robot.setSpeed(400*np.pi/5, np.pi/5.)
                else:
                    robot.setSpeed(400*np.pi/5, -np.pi/5.)

        elif estado == 1:
            # estado 2, llega al centro del 8
            if (599 <= x <= 601 and 1795 <= y <= 1805 and map_a) or \
                (2199 <= x <= 2201 and 1795 <= y <= 1805 and  not map_a):
                estado = 2
                # Actualizar velocidad
                if map_a:
                    robot.setSpeed(400*np.pi/5, -np.pi/5.)
                else:
                    robot.setSpeed(400*np.pi/5, np.pi/5.)

        elif estado == 2:
            # estado 3, llega arriba del 8
            if (599 <= x <= 625 and 995 <= y <= 1008 and map_a) or \
                (2199 <= x <= 2225 and 995 <= y <= 1008 and  not map_a):
                stop = True

        time.sleep(0.005)

    robot.setSpeed(0, 0)

    x, y, th = robot.readOdometry()

    th_goal = 0 if map_a else np.pi
    robot.rot(th_goal)

    robot.setSpeed(0, 0)
    return


def slalom2(robot, map_a):
    stop = False
    estado = 0
    while not stop:
        
        # Leer coordenadas del robot
        x, y, th = robot.readOdometry()
        print(x, y, th)
        if estado == 0:
            # estado 1, empieza la trayectoria
            estado = 1
            # Actualizar velocidad
            robot.setSpeed(200, 0)
                
        elif estado == 1:
            # estado 2, llega al centro del 8
            if (199 <= x <= 201 and map_a) or (2599 <= x <= 2601 and  not map_a):
                estado = 2
                # Actualizar velocidad
                s = 1 if map_a else -1
                robot.rot(-np.pi / 2, sign=s)
                robot.setSpeed(200, 0)

        elif estado == 2:
            # estado 3, llega arriba del 8
            if 1799 <= y <= 1801:
                estado = 3
                if map_a:
                    robot.rot(0)
                else:
                    robot.rot(np.pi)
                robot.setSpeed(200, 0)

        elif estado == 3:
            # estado 3, llega arriba del 8
            if (999 <= x <= 1001 and map_a) or (1799 <= x <= 1801 and  not map_a):
                estado = 4
                s = -1 if map_a else 1
                robot.rot(-np.pi / 2, sign=s)
                robot.setSpeed(200, 0)
        
        elif estado == 4:
            # estado 3, llega arriba del 8
            if 999 <= y <= 1001:
                estado = 5
                if map_a:
                    robot.rot(np.pi, sign=-1)
                else:
                    robot.rot(0, sign=1)
                robot.setSpeed(200, 0)
            
        elif estado == 5:
            # estado 3, llega arriba del 8
            if (599 <= x <= 601 and map_a) or (2199 <= x <= 2201 and  not map_a):
                if map_a:
                    robot.rot(0, sign=1)
                else:
                    robot.rot(np.pi, sign=-1)
                
                robot.setSpeed(0, 0)
                stop = True

        time.sleep(0.005)

    return


def get_center(robot,map_a):
    if map_a:
        robot.rot(0)
    else:
        robot.rot(np.pi)
    
    robot.setSpeed(0,0)
    time.sleep(0.02)

    d_x = robot.get_distance_sonar()
    v = d_x - 80
    stop = False
    if v > 0:
        robot.setSpeed(100, 0)
    else:
        robot.setSpeed(-100, 0)
    
    while not stop:
        time.sleep(0.02)
        d_x = robot.get_distance_sonar()
        stop = 78 < d_x < 82

    robot.setSpeed(0,0)
    time.sleep(0.02)

    robot.rot(np.pi/2)

    robot.setSpeed(0,0)
    time.sleep(0.02)

    d_y = robot.get_distance_sonar()
    v = d_y - 60
    stop = False
    if v > 0:
        robot.setSpeed(100, 0)
    else:
        robot.setSpeed(-100, 0)
    
    while not stop:
        time.sleep(0.02)
        d_y = robot.get_distance_sonar()
        stop = 58 < d_y < 62

    robot.setSpeed(0,0)
    time.sleep(0.02)

    return


def find_exit(robot, map_a):
    get_center(robot,map_a)
    r2_d2 = 'robots/R2-D2_s.png'
    bb8 = 'robots/BB8_s.png'
    img_r2 = cv2.imread(r2_d2)
    img_bb8 = cv2.imread(bb8)
    found1 = False
    found2 = False
    while (not found1) or (not found2):
        frame = robot.get_photo()
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        cv2.imshow("A", frame)
        cv2.waitKey()
        if not found1:
            found1, pts1 = robot.find_template(img_captured=frame, imReference=img_r2)
        if not found2:
            found2, pts2 = robot.find_template(img_captured=frame, imReference=img_bb8)
    
    x_pts1 = np.mean(pts1[:, 0][:, 0])
    x_pts2 = np.mean(pts2[:, 0][:, 0])
    
    
    if map_a:
        # PTS1 es lo q buscamos
        if x_pts1 < x_pts2:
            # Salir por la izq
            robot.rot(np.pi)

        else:
            # Salir por la derecha
            robot.rot(0)

    else:
        # PTS2 es lo q buscamos
        if x_pts2 < x_pts1:
            # Salir por la izq
            robot.rot(np.pi)

        else:
            # Salir por la dch
            robot.rot(0)
    robot.setSpeed(0, 0)
    
    return


def _exit(robot):
    robot.setSpeed(200,0)

    while not robot.detectObstacle():
        print(robot.get_distance_sonar())
        time.sleep(0.02)
    
    robot.setSpeed(0, 0)
    time.sleep(0.002)
    robot.rot(np.pi / 2)
    robot.setSpeed(0, 0)
    time.sleep(0.002)

    robot.setSpeed(200, 0)
    _, y, _ = robot.readOdometry()
    while y < 2800:
        time.sleep(0.02)
        _, y, _ = robot.readOdometry()
    
    robot.setSpeed(0, 0)
    return



def main(m, r, a):
    try:
        
        if a:
            robot = Robot(init_position=[600.0, 2600.0, np.pi])
            point_ini=np.array([3,5])
            point_end=np.array([7,5])
            map_file = 'mapas/mapaA_CARRERA2020.txt'
        else:
            robot = Robot(init_position=[2200.0, 2600.0, 0])
            point_ini=np.array([11,5])
            point_end=np.array([7,5])
            map_file = 'mapas/mapaB_CARRERA2020.txt'
        
        
        robot.startOdometry()
        
        slalom2(robot, a)
    
        myMap = Map2D(map_file)
        myMap.findPath(point_ini,point_end)
        time.sleep(0.5)
        myMap.move(robot)

        th_goal = np.pi / 2
        robot.rot(th_goal)

        robot.setSpeed(200, 0)
        time.sleep(2)
        robot.setSpeed(0, 0)
        th_goal = np.pi / 2
        robot.trackObject()
        # Catch the object
        robot.catch()

        robot.rot(th_goal)
        robot.setSpeed(0,0)
        time.sleep(0.02)
        
        find_exit(robot, map_a)
        _exit(robot)

        robot.stopOdometry()
        """
        slalom2(robot, a)
        robot.stopOdometry()
        
        
        robot.setSpeed(200, 0)
        time.sleep(2)

        robot.setSpeed(0,0)
        time.sleep(0.02)

        robot.rot(np.pi / 2)

        robot.setSpeed(0,0)
        time.sleep(0.02)

        robot.setSpeed(200, 0)
        time.sleep(2)

        robot.stopOdometry()
        robot.startOdometry()
        robot.setSpeed(100, 0)
        time.sleep(2)
        robot.setSpeed(0, -np.pi/3)
        stop = False
        while not stop:
            gyro = robot.get_gyro()
            print(gyro)
            time.sleep(.005)
            stop = abs(gyro) >= 84
        
        robot.setSpeed(0,0)
        time.sleep(1.0)
        aux = robot.get_gyro()
        robot.setSpeed(0, np.pi/3)
        stop = False
        while not stop:
            gyro = robot.get_gyro()
            print(gyro)
            time.sleep(.005)
            stop = gyro >= (aux + 84)
        # myMap = Map2D(m)
        # myMap.findPath(point_ini,point_end)
        print(robot.readOdometry())
        robot.setSpeed(100, 0)
        time.sleep(2)
        robot.stopOdometry()
        
        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        #myMap.verbose = True
        #myMap.drawMap(saveSnapshot=False)

        # you can set verbose to False to stop displaying plots interactively
        # (and maybe just save the snapshots of the map)
        #myMap.verbose = False

        # sample commands to see how to draw the map
        sampleRobotLocations = [ [0,0,0] ]
        # this will save a .png with the current map visualization,
        # all robot positions, last one in green
        myMap.verbose = True
        #myMap.drawMapWithRobotLocations( sampleRobotLocations, saveSnapshot=False )

        # this shows the current, and empty, map and an additionally closed connection
        #myMap.deleteConnection(0,0,0)
        #myMap.verbose = True
        #myMap.drawMap(saveSnapshot=False)

        # this will open a window with the results, but does not work well remotely
        #myMap.verbose = True
        #sampleRobotLocations = [ [200, 200, 3.14/2.0], [200, 600, 3.14/4.0], [200, 1000, -3.14/2.0],  ]
        #myMap.drawMapWithRobotLocations( sampleRobotLocations, saveSnapshot=False )

        point_ini=np.array([1,1])
        point_end=np.array([5,1])
        
        myMap.findPath(point_ini,point_end)
        print(myMap.costMatrix.transpose())
        myMap.drawMap(saveSnapshot=False)
        """

    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
    """

        if 0 < 0:
            print('d must be a positive value')
            exit(1)

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot() 
        # 1. launch updateOdometry thread()
        robot.startOdometry()
        
        # 2. Loop running the tracking until ??, then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
    	# res = robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255], 
        #                   targetSize=??, target??=??, ...)

        # Track the object until the robot is close enough
        robot.trackObject()
        # Catch the object
        robot.catch()
        # Retreat a little
        robot.setSpeed(-400, 0)
        time.sleep(1)
        # Stops the robot
        robot.setSpeed(0, 0)
        time.sleep(0.1)
        # Release the box
        robot.release()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    
    """
    print('a')

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", help="map that has to been followed by the robot",
                     default="A")
    args = parser.parse_args()

    if args.map == "A":
        _map = 'mapas/mapaA_CARRERA2020.txt'
        _robot = 'robots/BB8_s.png'
        map_a = True
    else:
        _map = 'mapas/mapaB_CARRERA2020.txt'
        _robot = 'robots/R2-D2_s.png'
        map_a = False

    main(_map, _robot, map_a)


