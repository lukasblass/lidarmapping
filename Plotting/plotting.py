import matplotlib.pyplot as plt
import os
import time

def readFileAndShow():
  plt.cla()
  dir_path = os.path.dirname(os.path.realpath(__file__))
  data_path = dir_path + "\data.txt"
  # we first read the full file as a string, then process the 
  # string for the plotting
  # this way we occupy the file for less amount of time than if we 
  # read line by line from the file
  f = open(data_path, "r")
  s = f.read()
  f.close()
  
  lines = s.split("\n")
  index = 0
  line = lines[0]
  while not (line == "car_estimations"):
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
    index = index + 1
    line = lines[index]
  
  index = index + 1
  line = lines[index]
  
  while not (line == "walls"):
    ex, ey = [], []
    row = line.split()
    ex.append(float(row[0]))
    ey.append(float(row[1]))
    ex.append(float(row[2]))
    ey.append(float(row[3]))
    ex.append(float(row[4]))
    ey.append(float(row[5]))
    ex.append(float(row[6]))
    ey.append(float(row[7]))
    ex.append(float(row[0]))
    ey.append(float(row[1]))
    plt.plot(ex, ey, color="black")
    index = index + 1
    line = lines[index]
  
  index = index + 1
  line = lines[index]

  x, y = [], []
  while line:
    row = line.split()
    x.append(float(row[0]))
    y.append(float(row[1]))
    index = index + 1
    line = lines[index]
  
  plt.scatter(x, y, color="blue")
  plt.draw()
  plt.pause(0.05)


plt.show(block=False)
while(True):
  readFileAndShow()