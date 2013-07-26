import numpy as np
import serial
from math import cos, sin, pi, asin, acos, atan, sqrt

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
      [cos(dh[PHETA]), -1*cos(dh[ALPHA])*sin(dh[PHETA]), sin(dh[ALPHA])*sin(dh[PHETA]),      dh[A]*cos(dh[PHETA])],
      [sin(dh[PHETA]), cos(dh[ALPHA])*cos(dh[PHETA]),    -1*sin(dh[ALPHA])*cos(dh[PHETA]),   dh[A]*sin(dh[PHETA])],
      [0,               sin(dh[ALPHA]),                    cos(dh[ALPHA]),                      dh[D]],
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
      [pi/2, 76.0, 0.0, -pi/2],
      [-pi/2, 0.0, 100.0, 0.0],
      [pi/2, 0.0, 100.0, 0.0],
      [0.0, -40.0, 50.0, -pi/2],
      [0.0, 45.0 + M, 32.0, 0.0]])

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
    
    PHETA_1 = asin((zm - ncfg[0][D]) / L)
    PHETA_2 = acos((ncfg[1][A]**2 + L**2 - ncfg[2][A]**2) / (2*ncfg[1][A]*L))

    print "PHETA_1 = " + str(PHETA_1)
    print "PHETA_2 = " + str(PHETA_2)

    ncfg[0][PHETA] = pi/2 - K
    ncfg[1][PHETA] = - (PHETA_1 + PHETA_2)
    ncfg[2][PHETA] = acos((ncfg[1][A]**2 + ncfg[2][A]**2 - L**2) / (2*ncfg[1][A]*ncfg[2][A]))
    ncfg[3][PHETA] = PHETA_2 + ncfg[2][PHETA] + PHETA_1 - pi

    self.config = ncfg
    return ncfg[:]

  def _sendCmd(self, port, cmd):
    port.write(cmd)
    print(cmd)
    print(port.read(10));


  def SendToArm(self):
    port = serial.Serial(self.port, 38400)
    speed = 0.3
    
    drvAngles = [ self.config[0][PHETA],
        -self.config[1][PHETA],
        self.config[2][PHETA],
        self.config[3][PHETA] + pi/2,
        ]

    self._sendCmd(port, "[SERV=0 ANGLE=%f VEL=%f]" % (drvAngles[0], speed))
    self._sendCmd(port, "[SERV=1 ANGLE=%f VEL=%f]" % (drvAngles[1], speed))
    self._sendCmd(port, "[SERV=3 ANGLE=%f VEL=%f]" % (drvAngles[2], speed))
    self._sendCmd(port, "[SERV=4 ANGLE=%f VEL=%f]" % (drvAngles[3], speed))

    port.close()


# x, y, z =  40.0, 182.0, 132.0
x, y, z = 40.0, 192.0, 170.0

man = Arm(0, "/dev/ttyACM0",)
check_model(man, x, y, z)
man.setTool(x, y, z)
man.SendToArm()
