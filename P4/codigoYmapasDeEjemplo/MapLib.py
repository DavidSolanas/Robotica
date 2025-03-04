#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import time
import math
import os

class Map2D:
    def __init__(self, map_description_file):
        """
        Load and initialize map from file. \

        map_description_file: path to a text file containing map description in the standard format. \
        Example for a 3x3 grid map, with (squared) cells of 400mm side length called mapa0. \
        All free space, i.e., all connections between cells are open, except those on the limits of the map.
        For more details on the format, see class documentation.

        mapa0.txt content:
        3 3 400
        0 0 0 0 0 0 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 0 0 0 0 0 0

        """
        # params to visualize
        self.mapLineStyle='r-'
        self.costValueStyle='g*'
        self.verbose = True
        # set to False to stop displaying plots interactively (and maybe just save the screenshots)
        # self.verbose = False
        self.current_ax = None

        # variables about map params
        self.sizeX=0
        self.sizeY=0
        self.sizeCell=0

        self.connectionMatrix = None
        self.costMatrix =  None
        self.currentPath =  None

        self.point_ini = None
        self.point_end = None

        if self._loadMap(map_description_file):
            print("Map %s loaded ok" % map_description_file)
        else:
            print("Map %s NOT loaded" % map_description_file)


    # from python docs: https://docs.python.org/3/tutorial/classes.html#private-variables
    # “Private” instance variables that cannot be accessed except from inside an object don’t exist in Python.
    # However, there is a convention that is followed by most Python code: a name prefixed with an underscore \
    # (e.g. _spam) should be treated as a non-public part of the API (whether it is a function, a method or a data member).

    # ############################################################
    # private methods
    # ############################################################
    def _initConnections(self, init_value=0):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.connectionMatrix = np.ones( (2*self.sizeX+1, 2*self.sizeY+1) ) * init_value

    def _initCostMatrix(self, init_value=-2):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.costMatrix = np.ones( (self.sizeX, self.sizeY) ) * init_value

        # Example costMatrix (filled manually!) for Map1
        # if we plan to go from 0,0 to 2,0
        # self.costMatrix[2,0] = 0
        # self.costMatrix[1,0] = 1
        # self.costMatrix[1,1] = 2
        # self.costMatrix[1,2] = 3
        # self.costMatrix[0,2] = 4
        # self.costMatrix[2,2] = 4
        # self.costMatrix[0,1] = 5
        # self.costMatrix[2,1] = 5
        # self.costMatrix[0,0] = 6



    def _loadMap(self, mapFileName):
        """
        Load map from a txt file (mapFileName) to fill the map params and connectionMatrix. \
        NOTES: \
        \t connectionMatrix is a numpy array \
        \t Function will return False if something went wrong loading the map file.
        """
        try:
            # FILL GLOBAL VARIABLES dimX dimY cellSize
            loadingOk=False
            mapF = open(mapFileName, "r")

            # 1. special case for first line. initialize dimX dimY cellSize
            header = mapF.readline() #next()
            tmp = header.split() # any whitespace string is a separator and empty strings are removed from the result
            if self.verbose:
                print("Header line: %s " % header)
            parsed_header = [int(c) for c in tmp]
            # expected to have three numbers: sizeX sizeY sizeCell_in_mm
            if len(parsed_header)==3:
                self.sizeX, self.sizeY, self.sizeCell = parsed_header
            else:
                print("Wrong header in map file: %s" % header)
                return False

            # 2.init connectionMatrix and costMatrix
            self._initConnections()
            self._initCostMatrix()

            # 3. load rest of the map connection lines information
            for indx, line in enumerate(mapF):
                # we start loading from the file the "top" row of the map
                current_row = (self.connectionMatrix.shape[1]-1) - indx
                # Split numbers in the line. Any whitespace string is a separator and empty strings are removed from the result
                tmp = line.split()
                if self.verbose:
                    print("Line for map row %d: %s " % (current_row, line))
                parsed_line = [int(c) for c in tmp]

                if len(parsed_line) == self.connectionMatrix.shape[0] and indx < self.connectionMatrix.shape[1]:
                    self.connectionMatrix[:, current_row] = parsed_line
                elif len(parsed_line): # don't give errors because of empty lines
                    print("Wrong connectionMatrix (%s) row data: %s" % (self.connectionMatrix.shape(), line) )
                    return False
            mapF.close()
            loadingOk = True
        except Exception as e:
            print("ERROR:", e.__doc__)
            print(e)
            #raise
            loadingOk = False

        return loadingOk

    def _cell2connCoord(self, cellX, cellY, numNeigh):
        """
        Input:
            cellX, cellY: cell coordinates (cellX, cellY) in the map grid
            numNeigh: index of one of the cell 8-neighbours

        Output:
            (connX,connY): 2D coordinates (in the connectionMatrix!!) \
            of the connection of the input cell to the input neighbour
        """
        connX=2*cellX+1
        connY=2*cellY+1
        p = [connX, connY]

        result = {
            0: lambda p: [ p[0],    p[1]+1],
            1: lambda p: [ p[0]+1,  p[1]+1],
            2: lambda p: [ p[0]+1,  p[1]],
            3: lambda p: [ p[0]+1,  p[1]-1],
            4: lambda p: [ p[0],    p[1]-1],
            5: lambda p: [ p[0]-1,  p[1]-1],
            6: lambda p: [ p[0]-1,  p[1]],
            7: lambda p: [ p[0]-1,  p[1]+1],
        }

        return result[numNeigh](p)

    def _pos2cell(self, x_mm, y_mm):
        """ Convert from robot odometry coordinates (in mm) to cell coordinates """
        # make sure we discretize the result to the closest lower integer value
        x_cell = int(np.floor(x_mm/self.sizeCell))
        y_cell = int(np.floor(y_mm/self.sizeCell))
        return [x_cell, y_cell]


    # ############################################################
    # public methods
    # ############################################################
    def setConnection(self, cellX, cellY, numNeigh):
        """
        open a connection, i.e., we can go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY]=1 # True

    def deleteConnection(self, cellX, cellY, numNeigh):
        """
        close a connection, i.e., we can NOT go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 0 # False

    def isConnected(self, cellX, cellY, numNeigh):
        """
        returns True if the connnection from cell (x,y) to its neighbour number numNeigh is open.

        The neighbour indexing is considered as follows
        (8-neighbours from cell x,y numbered clock-wise):

        7     0       1
        6   (x,y)     2
        5     4       3

        """
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        return self.connectionMatrix[connX, connY]

    # aux functions to display (or save image) with robot and map stuff
    def _drawGrid(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        plt.rc('grid', linestyle="--", color='gray')
        plt.grid(True)
        plt.tight_layout()

        x_t = range(0, (self.sizeX+1)*400, 400)
        y_t = range(0, (self.sizeY+1)*400, 400)
        x_labels = [str(n) for n in x_t]
        y_labels = [str(n) for n in y_t]
        plt.xticks(x_t, x_labels)
        plt.yticks(y_t, y_labels)

        # Main rectangle
        X = np.array([0, self.sizeX, self.sizeX, 0,          0]) * self.sizeCell
        Y = np.array([0, 0,          self.sizeY, self.sizeY, 0]) * self.sizeCell
        self.current_ax.plot(X, Y, self.mapLineStyle)

        # "vertical" walls
        for i in range(2, 2*self.sizeX, 2):
            for j in range(1, 2*self.sizeY, 2):
                if not self.connectionMatrix[i,j]:
                    # paint "right" wall from cell (i-1)/2, (j-1)/2
                    cx= np.floor((i-1)/2)
                    cy= np.floor((j-1)/2)
                    X = np.array([cx+1, cx+1]) * self.sizeCell
                    Y = np.array([cy, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)

        # "horizontal" walls
        for j in range(2, 2*self.sizeY, 2):
            for i in range(1, 2*self.sizeX, 2):
                if not self.connectionMatrix[i,j]:
                    # paint "top" wall from cell (i-1)/2, (j-1)/2
                    cx=np.floor((i-1)/2)
                    cy=np.floor((j-1)/2)
                    X = np.array([cx, cx+1]) * self.sizeCell
                    Y = np.array([cy+1, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)
        plt.axis('equal')

        return True


    # aux functions to display the current CostMatrix on the map
    def _drawCostMatrix(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        # "center" of each cell
        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                cx= i*self.sizeCell + self.sizeCell/2.
                cy= j*self.sizeCell + self.sizeCell/2.
                X = np.array([cx])
                Y = np.array([cy])
                cost = self.costMatrix[2*i+1,2*j+1]
                self.current_ax.text(X, Y, str(cost))


        plt.axis('equal')

        return True

    # Dibuja robot en location_eje con color (c) y tamano (p/g)
    def _drawRobot(self, loc_x_y_th=[0,0,0], robotPlotStyle='b', small=False):
        """
        UPDATES existing plot to include current robot position
        It expects an existing open figure (probably with the map already on it)

        loc_x_y_th is the position x,y and orientation in mm and radians of the main axis of the robot

        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        if small:
            largo, corto, descentre = [80, 50, 5]
        else:
            largo, corto, descentre = [160, 100, 10]

        trasera_dcha=np.array([-largo,-corto,1])
        trasera_izda=np.array([-largo,corto,1])
        delantera_dcha=np.array([largo,-corto,1])
        delantera_izda=np.array([largo,corto,1])
        frontal_robot=np.array([largo,0,1])

        tita=loc_x_y_th[2]
        Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_x_y_th[0]],
                 [np.sin(tita), np.cos(tita), loc_x_y_th[1]],
                  [0,        0 ,        1]])

        Hec=np.array([[1,0,descentre],
                  [0,1,0],
                  [0,0,1]])

        extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot=np.dot(Hwe, np.dot(Hec,np.transpose(extremos)))

        self.current_ax.plot(robot[0,:], robot[1,:], robotPlotStyle)

        return True

    def drawMapWithRobotLocations(self,
                                  robotPosVectors=[ [0,0,0], [600, 600, 3.14] ],
                                  saveSnapshot=True):
        """ Overloaded version of drawMap to include robot positions """
        return self.drawMap(robotPosVectors=robotPosVectors, saveSnapshot=saveSnapshot)


    def drawMap(self, robotPosVectors = None, saveSnapshot=False):
        """
        Generates a plot with currently loaded map status

        NOTE:
        if verbose, it displays the plot
        if saveSnapshot: saves a figure as mapstatus_currenttimestamp_FIGNUM.png
        """
        self.verbose=True
        #self.verbose=False

        # create a new figure and set it as current axis
        current_fig = plt.figure()
        self.current_ax = current_fig.add_subplot(111)

        self._drawGrid()

        # if flag is true, draw also current CostMatrix
        if self.verbose:
            self._drawCostMatrix()

        if robotPosVectors:
            for loc in robotPosVectors:
                print("Robot in pos: ", loc)
                self._drawRobot(loc_x_y_th=loc, robotPlotStyle='b--')
            # plot last robot position with solid green line
            self._drawRobot(loc_x_y_th=loc, robotPlotStyle='g-')

        if saveSnapshot:
            ts = str(time.time())
            snapshot_name = "mapstatus_"+ts+"_F"+str(current_fig.number)+".png"
            print("saving %s " % snapshot_name)
            plt.savefig(snapshot_name)

        if self.verbose:
            current_fig.set_visible(True)
            current_fig.show()
            print("Press ENTER in the plot window to continue ... ")
            current_fig.waitforbuttonpress()
        else:
            current_fig.set_visible(False)

        return current_fig


    def findPath(self, point_ini, point_end):
        """ overloaded call to planPath (x_ini,  y_ini, x_end, y_end) """
        return self.planPath(point_ini[0], point_ini[1],
                             point_end[0], point_end[1])

    # ############################################################
    # METHODS to IMPLEMENT in P4
    # ############################################################

    def fillCostMatrixR(self,x_end, y_end, cost):
        """ Calculate the weight of the grids (NF1) """
        r = False
        #right
        if self.costMatrix[x_end+1,y_end] != -1 and x_end+2 <= 2*self.sizeX:
            if self.costMatrix[x_end+2,y_end] < -1 or self.costMatrix[x_end+2,y_end] > cost:
                r = True
                self.costMatrix[x_end+2,y_end] = cost
                self.fillCostMatrixR(x_end+2,y_end, cost+1)

        #left
        if self.costMatrix[x_end-1,y_end] != -1 and x_end-2 >= 0:
            if self.costMatrix[x_end-2,y_end] < -1 or self.costMatrix[x_end-2,y_end] > cost:
                r = True
                self.costMatrix[x_end-2,y_end] = cost
                self.fillCostMatrixR(x_end-2,y_end, cost+1)

        #top
        if self.costMatrix[x_end,y_end+1] != -1 and y_end+2 <= 2*self.sizeY:
            if self.costMatrix[x_end,y_end+2] < -1 or self.costMatrix[x_end,y_end+2] > cost:
                r = True
                self.costMatrix[x_end,y_end+2] = cost
                self.fillCostMatrixR(x_end,y_end+2, cost+1)

        #down
        if self.costMatrix[x_end,y_end-1] != -1 and y_end-2 >= 0:
            if self.costMatrix[x_end,y_end-2] < -1 or self.costMatrix[x_end,y_end-2] > cost:
                r = True
                self.costMatrix[x_end,y_end-2] = cost
                self.fillCostMatrixR(x_end,y_end-2, cost+1)

        if not r:
            return

    def fillCostMatrix(self, point_ini, point_end):
        """ Part 1 and 2 of NF1 algorithm (initialize costMatrix and assign weights to the grids) """

        #fill costMatrix with zeros
        self.costMatrix = np.zeros((2*self.sizeX+1,2*self.sizeY+1))

        #assign -1 in wall´s positions and -2 in non visited ones
        for i in range(2*self.sizeX+1):
            for j in range(2*self.sizeY+1):
                if self.connectionMatrix[i,j] == True:
                    self.costMatrix[i,j] = -2
                else:
                    self.costMatrix[i,j] = -1

        #save initial position
        self.point_ini = point_ini
        #save goal position
        self.point_end = point_end

        self.costMatrix[point_end[0],point_end[1]] = 0
        self.fillCostMatrixR(point_end[0], point_end[1],1)


    def planPath(self, x_ini,  y_ini, x_end, y_end):
        """
        Plans the path that has to be followed by the robot in order to reach the end position.
        x_ini, y_ini, x_end, y_end: integer values that indicate \
            the x and y coordinates of the starting (ini) and ending (end) cell
        """
        point_ini=[x_ini,y_ini]
        point_end=[x_end,y_end]
        self.fillCostMatrix(point_ini,point_end)
        print(self.costMatrix)
        #Minimum moves for getting the goal 
        num_steps = int(self.costMatrix[x_ini,y_ini])
        self.currentPath = np.array( [ [0,0] ] * num_steps )

        index = 0
        x=x_ini
        y=y_ini
        minX=x_ini
        minY=y_ini		

        #finding the best path from the start to the goal (NF1)
        while self.costMatrix[minX,minY] != 0:
            #left
            if self.costMatrix[x-1,y] != -1 and self.costMatrix[x-2,y] < self.costMatrix[minX,minY]:
                minX = x-2		

            #right
            elif self.costMatrix[x+1,y] != -1 and self.costMatrix[x+2,y] < self.costMatrix[minX,minY]:
                minX = x+2	
            
            #down
            elif self.costMatrix[x,y-1] != -1 and self.costMatrix[x,y-2] < self.costMatrix[minX,minY]:
                minY = y-2		
            
            #top
            elif self.costMatrix[x,y+1] != -1 and self.costMatrix[x,y+2] < self.costMatrix[minX,minY]:
                minY = y+2	
            
            #draw the best path
            self.costMatrix[x,y] = 100-index		
            
            x = minX
            y = minY
            
            #save the path´s positions
            self.currentPath[index] = [x,y]

            index+=1	
        pathFound = True
        return pathFound
    
    def calculatePosition(self,x,y, th, x_goal, y_goal):
        """ Obtain the current robot´s position with the odometry """
        res_x = x_goal - x
        res_y = y_goal - y
        d_x = x / self.sizeCell
        d_y = y / self.sizeCell
        if th == 0 or th == np.pi:
            if abs(res_x) < 1:
                _x = int(math.ceil(d_x)) * 2
            else:
                _x = int(math.floor(d_x)) * 2 if res_x > 0 else int(math.ceil(d_x))
        else:
            _x = int(round(d_x)) * 2

        if th == np.pi/2 or th == -np.pi/2:
            if abs(res_y) < 1:
                _y = int(math.ceil(d_y)) * 2
            else:
                _y = int(math.floor(d_y)) * 2 if res_y > 0 else int(math.ceil(d_y)) * 2
        else:
            _y = int(round(d_y)) * 2

        return _x + 1, _y + 1


    def calculateOrientation(self,th):
        """ Obtain the current robot´s orientation with the odometry """
        angle=0

        #right
        if -np.pi/4 < th <= np.pi/4:
            angle = 0
        
        #up
        elif np.pi/4 < th <= 3*np.pi/4:
            angle = np.pi/2
        
        #down
        elif -3*np.pi/4 <= th <= -np.pi/4:
            angle = -np.pi/2
        
        #left
        else:
            angle = np.pi

        return angle

    def go(self,x_goal, y_goal, robot):
        """ Advance from the current cell to the next target cell if there is no obstacle between them,
            if the robot detects and obstacle a wall is marked in the map and returns value TRUE """
        x,y,th=robot.readOdometry()

        #calculate the cell you are 
        currentTh = self.calculateOrientation(th)
        x_next = x_goal / 2 * self.sizeCell
        y_next = y_goal / 2 * self.sizeCell
        currentX,currentY=self.calculatePosition(round(x/100)*100,round(y/100)*100, currentTh, x_next, y_next)
        
        
        #calculate the robot´s orientation (it can be only in 4 positions)
        vertical = y_goal - currentY
        horizontal = x_goal - currentX
        angle=0
        #the robot has to go up or down
        if horizontal == 0:
            angle = np.pi/2 - currentTh if vertical > 0 else -np.pi/2 - currentTh
        #the robot has to go to the right or to the left
        else:
            if horizontal > 0:
                angle = 0 - currentTh 
            else:
                angle =  -np.pi - currentTh if currentTh < 0 else np.pi - currentTh
                
        #orient the robot facing the grid to which it has to move
        if angle != 0:
            rot = np.pi / 2 if angle > 0 else -np.pi/2
            robot.setSpeed(0, rot)
            time.sleep(2 * abs(angle) / np.pi)

        """
        if angle != 0:
            th_gyro = robot.get_gyro()
            while th_gyro > (currentTh + angle + 0.005) or th_gyro < (currentTh + angle - 0.005):
                robot.setSpeed(0,angle)
                time.sleep(.008)
                th_gyro = robot.get_gyro()
                print(th_gyro) 
        """
        
        #stop moving
        robot.setSpeed(0,0)
        time.sleep(0.001)

        #move to the grid´s goal
        if not robot.detectObstacle():
            robot.setSpeed(150,0)

        #advance to the target box if no obstacle is detected
        while (currentX != x_goal or currentY != y_goal) and not robot.detectObstacle() :
            x,y, th = robot.readOdometry()
            currentTh = self.calculateOrientation(th)
            currentX,currentY=self.calculatePosition(x,y, currentTh, x_next, y_next)
            time.sleep(0.005)
        
        #if an obstacle was detected, we calculate where it is based on the orientation of the robot
        if (currentX != x_goal or currentY != y_goal):
            robot.setSpeed(0,0)
            x, y = currentX, currentY
            #right
            if(currentTh==0):
                currentX+=1
            #up
            elif(currentTh==np.pi/2):
                currentY+=1
            #down
            elif(currentTh==-np.pi/2):
                currentY-=1
            #left
            elif(currentTh==np.pi):
                currentX-=1
            
            #mark the obstacle cell
            self.connectionMatrix[currentX,currentY] = False
            #planning the new route
            self.replanPath(x,y,self.point_end[0],self.point_end[1])

        return robot.detectObstacle()

    def move(self,robot):
        """ Allows the robot to go from the initial position to the target following the planned path """
        index=0
        stop=False
        obstacle=False
        #while the robot is not in the target
        while not stop:
            #the robot advances cell by cell 
            obstacle = self.go(self.currentPath[index][0],self.currentPath[index][1], robot)
            #if there is an obstacle, it is necessary to start reading the current path again
            if obstacle:
                index = 0
            else:
                #if current cell has a 0, the robot is in target
                x = self.currentPath[index][0]
                y = self.currentPath[index][1]
                if self.costMatrix[x,y] == 0:
                    stop = True
                #the robot needs to go to the next cell of the current path
                else:
                    index += 1
    

    def replanPath(self, x, y, x_goal, y_goal):
        """ Replans the path if the robot detects an unexpected obstacle so
            the robot can reach the goal position """
        print('replaning...', x, y)
        self.planPath(x, y, x_goal, y_goal)
    

