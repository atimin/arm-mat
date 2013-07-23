import numpy as np
import serial
from math import cos, sin, pi, asin, acos, atan, sqrt

def dcos(deg):
  return cos(float(deg) / 180.0 * pi)

def dsin(deg):
  return sin(float(deg) / 180.0 * pi)

def dacos(v):
  return acos(v) / pi * 180

def dasin(v):
  return asin(v) / pi * 180


PHETA = 0
D     = 1
A     = 2
ALPHA = 3

class Man:

  def __init__(self, config):
    self.config = config
    self.init_config = config
    self.update_config()

  def update_config(self):
    self.A = [ np.matrix([ 
      [dcos(dh[PHETA]), -1*dcos(dh[ALPHA])*dsin(dh[PHETA]), dsin(dh[ALPHA])*dsin(dh[PHETA]),      dh[A]*dcos(dh[PHETA])],
      [dsin(dh[PHETA]), dcos(dh[ALPHA])*dcos(dh[PHETA]),    -1*dsin(dh[ALPHA])*dcos(dh[PHETA]),   dh[A]*dsin(dh[PHETA])],
      [0,               dsin(dh[ALPHA]),                    dcos(dh[ALPHA]),                      dh[D]],
      [0,               0,                                  0,                                    1]])

      for dh in self.config ]

  def turnJoint(self, num, delta):
    self.config[num][PHETA] += delta
    self.update_config()

  def reset(self):
    self.config = self.init_config
    self.update_config()

  def getCoordJoint(self, num):
    coord = np.matrix(np.identity(4))
    for i in xrange(num + 1):
      coord = coord * self.A[i]

    return coord * np.matrix("0;0;0;1")

  def setTool(self, x,y,z):
    raise Exeption("Method isn't implemented")


def check_model(man, x, y, z):
  print "Last config: " + str(man.init_config)

  newCfg = man.setTool(x, y, z)

  testMan = Man(newCfg)

  print "New config: " + str(newCfg)

  print "We want to move to " + str([x,y,z]) 
  print "But manipulator has moved to " + str(testMan.getCoordJoint(4)[0:3])

class Arm(Man):
  def __init__(self, M, port):
    self._M = M
    self.port = port

    Man.__init__(self, [
      [90, 76, 0, -90],
      [-90, 0, 100, 0],
      [90, 0, 100, 0],
      [0, -40, 50, -90],
      [0, 45 + M, 32, 0]])

  def setTool(self, x, y, z):
    ncfg = self.config[:]
    xm = x + ncfg[3][D]
    ym = y - (ncfg[3][A] + ncfg[4][A])
    zm = z + ncfg[4][D] + self._M

    print((xm, ym, zm))

    K = atan(xm/ym)
    Lp = sqrt(xm**2 + ym**2)
    L = sqrt((zm - ncfg[0][D])**2 + Lp**2)

    print "L = " + str(L)
    print "K = " + str(K)
    
    PHETA_1 = dasin(float(zm - ncfg[0][D]) / L)
    PHETA_2 = dacos(float(ncfg[1][A]**2 + L**2 - ncfg[2][A]**2) / float(2*ncfg[1][A]*L))

    print "PHETA_1 = " + str(PHETA_1)
    print "PHETA_2 = " + str(PHETA_2)

    ncfg[0][PHETA] = 90 - K * 180 /  pi
    ncfg[1][PHETA] = - PHETA_1 - PHETA_2
    ncfg[2][PHETA] = dacos(float(ncfg[1][A]**2 + ncfg[2][A]**2 - L**2) / float(2*ncfg[1][A]*ncfg[2][A]))
    ncfg[3][PHETA] = 90 - ((90 - PHETA_1) + (180 - PHETA_2 - ncfg[2][PHETA]))

    self.config = ncfg
    return ncfg[:]

  def SendToArm(self):
    ser = serial.Serial(self.port, 38400)
    speed = 0.1
    
    ser.write("[SERV=0 ANGLE=%f VEL=%f]" % (self.config[0][PHETA]*0.00865, speed))
    print("[SERV=0 ANGLE=%f VEL=%f]" % (self.config[0][PHETA], speed))
    print(ser.read(10));
    ser.write("[SERV=1 ANGLE=%f VEL=%f]" % ((180+self.config[1][PHETA])*0.00865, speed))
    print("[SERV=1 ANGLE=%f VEL=%f]" % (self.config[1][PHETA], speed))
    print(ser.read(10));
    ser.write("[SERV=2 ANGLE=%f VEL=%f]" % ((180+self.config[1][PHETA])*0.00865, speed))
    print("[SERV=2 ANGLE=%f VEL=%f]" % (self.config[1][PHETA], speed))
    print(ser.read(10));
    ser.write("[SERV=3 ANGLE=%f VEL=%f]" % ((90 +  self.config[2][PHETA])*0.00865, speed))
    print("[SERV=3 ANGLE=%f VEL=%f]" % ((90 + self.config[2][PHETA])*0.00865, speed))
    print(ser.read(10));
    ser.write("[SERV=4 ANGLE=%f VEL=%f]" % ((self.config[3][PHETA])*0.00865 , speed))
    print("[SERV=4 ANGLE=%f VEL=%f]" % ((self.config[3][PHETA])*0.00865, speed))
    print(ser.read(10));

    ser.close()

x, y, z = 20.0, 182.0, 131.0

man = Arm(0, "/dev/ttyACM0",)
check_model(man, x, y, z)
# man.setTool(x, y, z)
# man.SendToArm()
