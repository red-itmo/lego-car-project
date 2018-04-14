from matplotlib import pyplot as plt
import math
cam_res = (640, 480)
px_to_m = 3.60 / math.hypot(cam_res[0], cam_res[1])


with open('logs_gui.txt') as f1:
    data_gui = f1.read()

with open('test.txt') as f2:
    data_car = f2.read()

data_gui, data_car = data_gui.split('\n'), data_car.split('\n')
print(data_car)
x_gui = [float(row.split(' ')[0])*px_to_m for row in data_gui[0:-1]]
y_gui = [float(row.split(' ')[1])*px_to_m for row in data_gui[0:-1]]
theta_gui = [float(row.split(' ')[2]) for row in data_gui[0:-1]]

x_car = [float(row.split(' ')[1]) for row in data_car[0:-1]]
y_car = [float(row.split(' ')[2]) for row in data_car[0:-1]]
theta_car = [float(row.split(' ')[7]) for row in data_car[0:-1]]

arrows=[]
for index,_ in enumerate(x_car):
    arrows.append(plt.arrow(x_car[index], y_car[index],
                            0.1*math.cos(theta_car[index]),
                            0.1*math.sin(theta_car[index]),
                            head_width=0.005, head_length=0.01))
ax = plt.gca()
for arrow in arrows:
    arrow.set_facecolor('r')
    ax.add_patch(arrow)
plt.scatter(x_car,y_car)
plt.show()

arrows_gui=[]
for index,_ in enumerate(x_gui):
    arrows_gui.append(plt.arrow(x_gui[index], y_gui[index],
                            0.1*math.cos(theta_gui[index]),
                            0.1*math.sin(theta_gui[index]),
                            head_width=0.1, head_length=0.01))
ax = plt.gca()
for arrow in arrows_gui:
    arrow.set_facecolor('g')
    ax.add_patch(arrow)
plt.scatter(x_gui,y_gui)
plt.show()
