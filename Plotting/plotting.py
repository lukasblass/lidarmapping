import matplotlib.pyplot as plt
import os
from time import sleep


def readFileAndShow():
  dir_path = os.path.dirname(os.path.realpath(__file__))
  data_path = dir_path + "\data.txt"
  x, y = [], []
  carx, cary = [], []
  walls = False
  f = open(data_path, "r")
  for line in f:
    row = line.split()
    if (row[0] == "walls"):
      walls = True
    elif not walls:
      carx.append(float(row[0]))
      cary.append(float(row[1]))
    else:
      x.append(float(row[0]))
      y.append(float(row[1]))
  f.close()
  
  plt.cla()
  plt.scatter(carx, cary, color="red")
  plt.scatter(x, y, color="blue")
  plt.draw()
  plt.pause(1)


plt.show(block=False)
while(True):
  readFileAndShow()