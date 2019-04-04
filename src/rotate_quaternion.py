#!/usr/bin/python

import sys
import numpy as np

def euler_to_quaternion(yaw, pitch, roll):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  print qx,qy,qz,qw
  return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
  import math
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  X = math.degrees(math.atan2(t0, t1))
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = math.degrees(math.asin(t2))
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  Z = math.degrees(math.atan2(t3, t4))
  return X, Y, Z

if len(sys.argv) != 5:
  print 'Correct usage: \n python rotate_quaternion.py <x> <y> <z> <w>'

x = float(sys.argv[1])
y = float(sys.argv[2])
z = float(sys.argv[3])
w = float(sys.argv[4])

euler_one = quaternion_to_euler(x,y,z,w)
x_two, y_two, z_two = raw_input('Input Euler Angles <x> <y> <z>').split() 

x_final = euler_one[0] + float(x_two)
y_final = euler_one[1] + float(y_two)
z_final = euler_one[2] + float(z_two)

quat = euler_to_quaternion(x_final, y_final, z_final)
print quat
