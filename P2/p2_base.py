#!/usr/bin/python
# -*- coding: UTF-8 -*-

###########################################
#   Autores: Daniel Cay (741066)          #
#            Javier Fañanás (737987)      #
#            David Solanas (738630)       #
#                                         #
#   Fichero: p2_base.py                   #
#   Robótica - Práctica 2                 #
###########################################


import argparse
import numpy as np
import time
import math
from Robot import Robot


def trajectory_8(robot):
    """ Realiza una trayectoria en forma de 8, con 4 estados \
        a seguir durante la trayectoria.  """
    d = 400 # mm, baldosa
    stop = False
    estado = 0

    while not stop:
        
        # Leer coordenadas del robot
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
    """ Realiza una trayectoria en forma de huevo, con 7 estados \
        a seguir durante la trayectoria.  """
    d1 = 200 # mm, media baldosa
    d2 = 400 # mm, baldosa
    alpha = 0.2617 # 15º
    stop = False
    estado = 0

    while not stop:

        # Leer coordenadas del robot
        robot.lock_odometry.acquire()
        x, y, th = robot.readOdometry()
        robot.lock_odometry.release()

        if estado == 0:
            estado = 1
            # Actualizar velocidad
            robot.setSpeed(0., math.pi / 2.)

        elif estado == 1:
            # estado 1, empieza la trayectoria
            if (-5 <= x <= 5) and (-5 <= y <= 5) and (math.pi/2. - 0.01 <= th <= math.pi/2. + 0.01):
                estado = 2
                # Actualizar velocidad
                robot.setSpeed(50*math.pi, -math.pi/4)

        elif estado == 2:
            # estado 2, semicícurlo
            if (alpha - 0.01 <= th <= alpha + 0.015):
                estado = 3
                # Actualizar velocidad
                robot.setSpeed((2*d2 + d1) / 5., 0.)

        elif estado == 3:
            # estado 3, reposicionamiento
            if (2*d2 + d1 - 25 <= x <= 2*d2 + d1 + 25) and (d2 - 25 <= y <= d2 + 25) and (alpha - 0.04 <= th <= alpha + 0.04):
                estado = 4
                # Actualizar velocidad
                robot.setSpeed(0., -alpha * 0.8)

        elif estado == 4:
            # estado 4, línea recta
            if (2*d2 + d1 -25 <= x <= 2*d2 + d1 + 25) and (d2 - 30 <= y <= d2 + 30) and (-0.002 <= th <=  0.002):
                estado = 5
                # Actualizar velocidad
                robot.setSpeed(100*math.pi, -math.pi/4)

        elif estado == 5:
            # estado 5, segundo semicírculo
            if (math.pi -alpha/1.5 <= th <= math.pi-alpha/1.5 + 0.02):
                estado = 6
                # Actualizar velocidad
                # robot.setSpeed(0., -alpha * 0.5)
                robot.setSpeed((2*d2 + d1) / 5., 0)
                
        elif estado == 6:
            # estado 6, vuelve al origen
            if (d1 - 30 <= x <= d1 - 25):
                estado = 7
                # Actualizar velocidad
                robot.setSpeed(50*math.pi, -math.pi/4)

        elif estado == 7:
            # estado 7, vuelve a posición inicial
            if (-10 <= x <= 10) and (-10 <= y <= 10):
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
        # trajectory_2(robot)


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



