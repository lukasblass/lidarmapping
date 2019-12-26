import matplotlib.pyplot as plt

x, y = [], []
f = open("data.txt", "r")
for line in f:
  row = line.split()
  x.append(float(row[0]))
  y.append(float(row[1]))

plt.scatter(y,x)
plt.show()