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


def slalom2(robot, map_a):
    """ Realiza la primera parte del recorrido, el slalom correspondiente
        al mapa A o al mapa B. Se realiza con giros de 90 grados. """
    stop = False
    estado = 0
    while not stop:
        
        # Leer coordenadas del robot
        x, y, th = robot.readOdometry()

        if estado == 0:
            # estado 0, empieza la trayectoria
            estado = 1
            # Actualizar velocidad
            robot.setSpeed(200, 0)
                
        elif estado == 1:
            # estado 1, llega al límite, toca girar y avanzar hasta la siguiente posición
            if (199 <= x <= 201 and map_a) or (2599 <= x <= 2601 and  not map_a):
                estado = 2
                # Dependiendo del mapa, se gira en un sentido u otro
                s = 1 if map_a else -1
                robot.rot(-np.pi / 2, sign=s)
                # Avanzar recto
                robot.setSpeed(200, 0)

        elif estado == 2:
            # estado 2, llega al centro del slalom, girar y avanzar
            if 1799 <= y <= 1801:
                estado = 3
                # Dependiendo del mapa, se gira en un sentido u otro
                if map_a:
                    robot.rot(0)
                else:
                    robot.rot(np.pi, sign=-1)
                robot.setSpeed(200, 0)

        elif estado == 3:
            # estado 3, se termina el centro del slalom y avanza hacia abajo
            if (999 <= x <= 1001 and map_a) or (1799 <= x <= 1801 and  not map_a):
                estado = 4
                # Dependiendo del mapa, se gira en un sentido u otro
                s = -1 if map_a else 1
                robot.rot(-np.pi / 2, sign=s)
                # Avanzar hasta el final
                robot.setSpeed(200, 0)
        
        elif estado == 4:
            # estado 3, llega a la parte final del slalom y se encara para terminar
            if 999 <= y <= 1001:
                estado = 5
                # Dependiendo del mapa, se gira en un sentido u otro
                if map_a:
                    robot.rot(np.pi, sign=-1,  offset=0.12)
                else:
                    robot.rot(0, sign=1,  offset=0.12)
                robot.setSpeed(200, 0)
            
        elif estado == 5:
            # estado 5, termina el slalom, se encara para empezar nueva ruta
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
    """ Función para centrar el robot para poder identificar por dónde hay que salir.
        Se coloca en el centro del espacio de la pelota mirando hacia las imágenes. """

    # Se orienta hacia las paredes para colocarse según el sónar
    if map_a:
        robot.rot(0)
    else:
        robot.rot(np.pi)
    
    robot.setSpeed(0,0)
    time.sleep(0.02)

    # Avanza (o se aleja) hacia la pared hasta estar a 80 cm de ella
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

    # Se encara el robot hacia las imágenes
    robot.rot(np.pi/2)

    robot.setSpeed(0,0)
    time.sleep(0.02)

    # Avanza (o se aleja) hacia las imágenes hasta estar a 60 cm de ellas.
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
    """ Busca la imagen correspondiente al mapa y según su posición,
        se determina por dónde hay que salir. """

    get_center(robot,map_a)
    r2_d2 = 'robots/R2-D2_s.png'
    bb8 = 'robots/BB8_s.png'
    img_r2 = cv2.imread(r2_d2)
    img_bb8 = cv2.imread(bb8)
    found1 = False
    found2 = False

    # Busca hasta encontrar matches suficientes de ambas imágnes
    while (not found1) or (not found2):
        frame = robot.get_photo()
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        if not found1:
            found1, pts1 = robot.find_template(img_captured=frame, imReference=img_r2)
        if not found2:
            found2, pts2 = robot.find_template(img_captured=frame, imReference=img_bb8)
    
    # Se calcula la posición media de cada imágen
    x_pts1 = np.mean(pts1[:, 0][:, 0])
    x_pts2 = np.mean(pts2[:, 0][:, 0])
    
    # Dependiendo del mapa y las coordenadas de los matches se orienta en un sentido
    # u otro para salir del laberinto.
    if map_a:
        if x_pts1 < x_pts2:
            # Salir por la izq
            robot.rot(np.pi)

        else:
            # Salir por la derecha
            robot.rot(0)

    else:
        if x_pts2 < x_pts1:
            # Salir por la izq
            robot.rot(np.pi)

        else:
            # Salir por la dch
            robot.rot(0)
    robot.setSpeed(0, 0)
    
    return


def _exit(robot, map_a):
    """ El robot sale del laberinto haciendo uso del sónar """
    robot.setSpeed(200,0)

    # Avanza recto hasta detectar la pared
    while not robot.detectObstacle():
        time.sleep(0.02)
    
    robot.setSpeed(0, 0)
    time.sleep(0.002)
    
    # Se orienta hacia la salida y avanza recto hasta salir del mapa
    sign = -1 if map_a else 1 
    robot.rot(np.pi / 2, sign=sign)
    robot.setSpeed(0, 0)
    time.sleep(0.002)

    robot.setSpeed(200, 0)
    _, y, _ = robot.readOdometry()
    # Se avanza recto hasta alcanzar la posición y=3200
    while y < 3200:
        time.sleep(0.02)
        _, y, _ = robot.readOdometry()
    
    # Se ha salido del laberinto, parar el robot
    robot.setSpeed(0, 0)
    return



def main(m, r, a):
    """ Se realiza el circuito completo, slalom, laberinto, atrapar pelota y salir. """
    try:
        # Dependiendo de los argumentos se elige un mapa u otro
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
        # Se hace el slalom
        slalom2(robot, a)
    
        # Se carga el mapa y se avanza hasta la posición objetivo
        myMap = Map2D(map_file)
        myMap.findPath(point_ini,point_end)
        time.sleep(0.5)
        myMap.move(robot)

        # Se orienta hacia arriba para entrar en la zona de la pelota
        th_goal = np.pi / 2
        robot.rot(th_goal)
        # Avanza para entrar en la zona de la pelota
        robot.setSpeed(200, 0)
        time.sleep(2.5)

        robot.setSpeed(0, 0)
        th_goal = np.pi / 2

        # Se busca la pelota y se acerca a ella
        robot.trackObject()
        # Atrapa la pelota
        robot.catch()
        
        # Busca la salida
        find_exit(robot, map_a)
        # Sale del mapa
        _exit(robot, map_a)

        robot.stopOdometry()
        

    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


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


