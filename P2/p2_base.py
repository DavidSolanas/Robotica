#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import math
from Robot import Robot


def trajectory_8(robot):

    d = 400 # mm, baldosa
    stop = False
    estado = 0

    while not stop:

        robot.lock_odometry.acquire()
        x, y, th = robot.readOdometry()
        robot.lock_odometry.release()

        if estado == 0:
            estado = 1
            # Actualizar velocidad
            robot.setSpeed(100*math.pi, math.pi/4.)

        elif estado == 1:
            # estado 1, empieza la trayectoria
            if (-25 <= x <= 25) and (2*d - 25 <= y <= 2*d + 25) and (math.pi - 0.1 <= th <= math.pi + 0.1):
                estado = 2
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/4.)

        elif estado == 2:
            # estado 2, llega al centro del 8
            if (-25 <= x <= 25) and (4*d - 25 <= y <= 4*d + 25) and (-0.1 <= th <= 0.1):
                estado = 3
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/4.)

        elif estado == 3:
            # estado 3, llega arriba del 8
            if (-25 <= x <= 25) and (2*d - 25 <= y <= 2*d + 25) and (math.pi - 0.1 <= th <= math.pi + 0.1):
                estado = 4
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, math.pi/4.)

        elif estado == 4:
            # estado 4, vuelve al centro del 8
            if (-25 <= x <= 25) and (-25 <= y <= 25) and (-0.1 <= th <= 0.1):
                stop = True
        
        time.sleep(0.005)
    return


def trajectory_2(robot):

    d1 = 200 # mm, media baldosa
    d2 = 400 # mm, baldosa
    alpha = 0.2617 # 15º
    stop = False
    estado = 0

    while not stop:

        robot.lock_odometry.acquire()
        x, y, th = robot.readOdometry()
        robot.lock_odometry.release()

        if estado == 0:
            estado = 1
            # Actualizar velocidad
            robot.setSpeed(0., math.pi / 2.)

        elif estado == 1:
            # estado 1, empieza la trayectoria
            if (-25 <= x <= 25) and (-25 <= y <= 25) and (math.pi/2. - 0.1 <= th <= math.pi/2. + 0.1):
                estado = 2
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/2.)

        elif estado == 2:
            # estado 2, llega al centro del 8
            if (alpha - 0.1 <= th <= alpha + 0.1):
                estado = 3
                # Actualizar velocidad
                robot.setSpeed((2*d2 + d1) / 4., 0)

        elif estado == 3:
            # estado 3, llega arriba del 8
            print(x,y,th)
            if (2*d2 + d1 -50 <= x <= 2*d2 + d1 + 50) and (d2 - 50 <= y <= d2 + 50) and (alpha - 0.1 <= th <= alpha + 0.1):
                estado = 4
                # Actualizar velocidad
                robot.setSpeed(0., -alpha * 0.2)

        elif estado == 4:
            # estado 4, llega arriba del 8
            if (2*d2 + d1 -25 <= x <= 2*d2 + d1 + 25) and (d2 - 25 <= y <= d2 + 25) and (-0.1 <= th <=  0.1):
                estado = 5
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/4.)

        elif estado == 5:
            # estado 5, vuelve al centro del 8
            if (2*d2 + d1 -25 <= x <= 2*d2 + d1 + 25) and (-d2 - 25 <= y <= -d2 + 25) and (-math.pi -0.1 <= th <= -math.pi + 0.1):
                estado = 6
                # Actualizar velocidad
                robot.setSpeed(0., -alpha * 0.2)

        elif estado == 6:
            # estado 6, vuelve al centro del 8
            if (2*d2 + d1 -25 <= x <= 2*d2 + d1 + 25) and (-d2 - 25 <= y <= -d2 + 25) and (-3.4033 -0.1 <= th <= -3.4033 + 0.1):
                estado = 7
                # Actualizar velocidad
                robot.setSpeed((2*d2 + d1) / 4., 0)
                
        elif estado == 7:
            # estado 7, vuelve al centro del 8
            if (d1 -25 <= x <= d1 + 25) and (-d1 - 25 <= y <= -d1 + 25) and (-3.4033 -0.1 <= th <= -3.4033 + 0.1):
                estado = 8
                # Actualizar velocidad
                robot.setSpeed(0., alpha * 0.2)

        elif estado == 8:
            # estado 8, vuelve al centro del 8
            if (d1 -25 <= x <= d1 + 25) and (-d1 - 25 <= y <= -d1 + 25) and (-math.pi -0.1 <= th <= -math.pi + 0.1):
                estado = 9
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/2.)

        elif estado == 8:
            # estado 8, vuelve al centro del 8
            if (-25 <= x <= 25) and (-25 <= y <= 25) and (math.pi/2. -0.1 <= th <= math.pi/2. + 0.1):
                stop = True
                

        time.sleep(0.005)
    return


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        robot = Robot(np.array([0,0,0])) # init_position=args.pos_ini)
        # robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory


        # PART 1:
        # trajectory_8(robot)
        trajectory_2(robot)

            

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



