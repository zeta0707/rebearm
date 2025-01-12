import numpy as np
from scipy.optimize import curve_fit
import csv
import os
import matplotlib.pyplot as plt

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

input = []
output = []
csvfile = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_ml/rebearm_ml/dataset/train/kinematics_pose.csv')

with open(csvfile, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        input.append(float(row.get('x')))
        output.append(float(row.get('data[0]')))
        
popt, pcov = curve_fit(calcPwm, input, output)
print(popt)

plt.plot(input, output, 'b-', label='data')
plt.plot(input, calcPwm(input, popt[0], popt[1]), 'r-', label='fit: a=%2.2f, b=%2.2f' %tuple(popt))
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True) 
plt.legend()
plt.show()