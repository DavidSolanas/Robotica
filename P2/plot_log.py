import numpy as np
import matplotlib.pyplot as plt
# import time



# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c,tamano):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=loc_eje[2]
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  plt.plot(robot[0,:], robot[1,:], c)
  


def read_log(filename):
  f = open(filename, 'r')
  lines = f.readlines()
  coords = []
  for line in lines:
    line = line.strip()
    x = np.float(line.split(",")[0].strip()[3:])
    y = np.float(line.split(",")[1].strip()[3:])
    th = np.float(line.split(",")[2].strip()[3:])
    coords.append([x, y, th])
  
  return coords




coord = np.array(read_log('odometry_part2.log'))

for c in coord:
  dibrobot(c, 'b', 'p')

plt.show()