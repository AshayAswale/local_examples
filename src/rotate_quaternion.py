#!/usr/bin/python

import numpy as np
import math
import quaternion

def euler_to_quaternion(euler):
  roll  = euler[0]
  pitch = euler[1]
  yaw   = euler[2]

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]


def quaternion_to_euler(quaternion):
  x = quaternion[0]
  y = quaternion[1]
  z = quaternion[2]
  w = quaternion[3]

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll = math.degrees(math.atan2(t0, t1))

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch = math.degrees(math.asin(t2))

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw = math.degrees(math.atan2(t3, t4))

  return roll, pitch, yaw


def quaternion_multiply(quaternion1, quaternion0):
  x0, y0, z0, w0 = quaternion0
  x1, y1, z1, w1 = quaternion1
  return np.array([w0*x1 + x0*w1 + y0*z1 - z0*y1,
                   w0*y1 - x0*z1 + y0*w1 + z0*x1,
                   w0*z1 + x0*y1 - y0*x1 + z0*w1,
                   w0*w1 - x0*x1 - y0*y1 - z0*z1], dtype=np.float64)



quat_one = raw_input('Input Quaternion Angles <x> <y> <z> <w>  ').split() 
quat_one = [float(i) for i in quat_one]

euler_two = raw_input('Input Euler Angles <roll> <pitch> <yaw>  ').split() 
euler_two = [float(i) for i in euler_two]

quat_two = euler_to_quaternion(euler_two)
print quat_two

quat_output = quaternion_multiply(quat_two, quat_one)
quat_formatted = [ '%.4f' % elem for elem in quat_output ]

print '\nRotated Quaternion: <x> <y> <z> <w>:'
print quat_formatted
