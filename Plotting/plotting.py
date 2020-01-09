import matplotlib.pyplot as plt
import os
from time import sleep

#TODO might be better to first read the entire file as a string/list of strings
# and then work on that string
# this way we don't have to hold the file open as long
def readFileAndShow():
  plt.cla()
  dir_path = os.path.dirname(os.path.realpath(__file__))
  data_path = dir_path + "\data.txt"

  f = open(data_path, "r")
  line = f.readline()
  while not (line == "car_estimations\n"):
    carx, cary = [], []
    row = line.split()
    carx.append(float(row[0]))
    cary.append(float(row[1]))
    carx.append(float(row[2]))
    cary.append(float(row[3]))
    carx.append(float(row[4]))
    cary.append(float(row[5]))
    carx.append(float(row[6]))
    cary.append(float(row[7]))
    carx.append(float(row[0]))
    cary.append(float(row[1]))
    plt.plot(carx, cary, color="red")
    line = f.readline()
  
  line = f.readline()
  ex, ey = [], []
  while not (line == "walls\n"):
    row = line.split()
    ex.append(float(row[0]))
    ey.append(float(row[1]))
    line = f.readline()
  
  line = f.readline()
  x, y = [], []
  while line:
    row = line.split()
    x.append(float(row[0]))
    y.append(float(row[1]))
    line = f.readline()
  
  f.close()
  
  plt.scatter(ex, ey, color="black")
  plt.scatter(x, y, color="blue")
  plt.draw()
  plt.pause(1)


plt.show(block=False)
while(True):
  readFileAndShow()