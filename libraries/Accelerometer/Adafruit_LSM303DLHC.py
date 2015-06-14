#!/usr/bin/python

import sys
import os
import time
import smbus
import math
from datetime import datetime, date
from Adafruit_I2C import Adafruit_I2C
import unittest

# ===========================================================================
# LSM303DLHC Class
# by Shigeru KAWAGUCHI
# ===========================================================================

class LSM303DLHC :
  i2c_accel = None
  i2c_mag = None
  accelFactor = None
  accelDataRate = None
  accelLowPower = None
  accelAxes = None
  accelHiRez = None
  accelScale = None
  magFacot = None
  tempEnabled = None
  magDataRate = None

  # Device Addresses
  __LSM303DLHC_ADDRESS_ACCEL		= 0x32
  __LSM303DLHC_ADDRESS_MAG			= 0x3C
  __LSM303DLHC_ID					= 0b00110010

  # LSM303DLHC Registers								Default		Type
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG1_A		= 0x20	# 00000111   rw
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG2_A		= 0x21	# 00000000   rw
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG3_A		= 0x22	# 00000000   rw
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG4_A		= 0x23	# 00000000   rw
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG5_A		= 0x24	# 00000000   rw
  __LSM303DLHC_REGISTER_ACCEL_CTRL_REG6_A		= 0x25	# 00000000   rw
  __LSM303DLHC_REGISTER_ACCEL_REFERENCE_A		= 0x26	# 00000000   r
  __LSM303DLHC_REGISTER_ACCEL_STATUS_REG_A		= 0x27	# 00000000   r
  __LSM303DLHC_REGISTER_ACCEL_OUT_X_L_A			= 0x28
  __LSM303DLHC_REGISTER_ACCEL_OUT_X_H_A			= 0x29
  __LSM303DLHC_REGISTER_ACCEL_OUT_Y_L_A			= 0x2A
  __LSM303DLHC_REGISTER_ACCEL_OUT_Y_H_A			= 0x2B
  __LSM303DLHC_REGISTER_ACCEL_OUT_Z_L_A			= 0x2C
  __LSM303DLHC_REGISTER_ACCEL_OUT_Z_H_A			= 0x2D
  __LSM303DLHC_REGISTER_ACCEL_FIFO_CTRL_REG_A	= 0x2E
  __LSM303DLHC_REGISTER_ACCEL_FIFO_SRC_REG_A	= 0x2F
  __LSM303DLHC_REGISTER_ACCEL_INT1_CFG_A		= 0x30
  __LSM303DLHC_REGISTER_ACCEL_INT1_SOURCE_A		= 0x31
  __LSM303DLHC_REGISTER_ACCEL_INT1_THS_A		= 0x32
  __LSM303DLHC_REGISTER_ACCEL_INT1_DURATION_A	= 0x33
  __LSM303DLHC_REGISTER_ACCEL_INT2_CFG_A		= 0x34
  __LSM303DLHC_REGISTER_ACCEL_INT2_SOURCE_A		= 0x35
  __LSM303DLHC_REGISTER_ACCEL_INT2_THS_A		= 0x36
  __LSM303DLHC_REGISTER_ACCEL_INT2_DURATION_A	= 0x37
  __LSM303DLHC_REGISTER_ACCEL_CLICK_CFG_A		= 0x38
  __LSM303DLHC_REGISTER_ACCEL_CLICK_SRC_A		= 0x39
  __LSM303DLHC_REGISTER_ACCEL_CLICK_THS_A		= 0x3A
  __LSM303DLHC_REGISTER_ACCEL_TIME_LIMIT_A		= 0x3B
  __LSM303DLHC_REGISTER_ACCEL_TIME_LATENCY_A	= 0x3C
  __LSM303DLHC_REGISTER_ACCEL_TIME_WINDOW_A		= 0x3D

  __LSM303DLHC_REGISTER_MAG_CRA_REG_M			= 0x00
  __LSM303DLHC_REGISTER_MAG_CRB_REG_M			= 0x01
  __LSM303DLHC_REGISTER_MAG_MR_REG_M			= 0x02
  __LSM303DLHC_REGISTER_MAG_OUT_X_H_M			= 0x03
  __LSM303DLHC_REGISTER_MAG_OUT_X_L_M			= 0x04
  __LSM303DLHC_REGISTER_MAG_OUT_Z_H_M			= 0x05
  __LSM303DLHC_REGISTER_MAG_OUT_Z_L_M			= 0x06
  __LSM303DLHC_REGISTER_MAG_OUT_Y_H_M			= 0x07
  __LSM303DLHC_REGISTER_MAG_OUT_Y_L_M			= 0x08
  __LSM303DLHC_REGISTER_MAG_SR_REG_Mg			= 0x09
  __LSM303DLHC_REGISTER_MAG_IRA_REG_M			= 0x0A
  __LSM303DLHC_REGISTER_MAG_IRB_REG_M			= 0x0B
  __LSM303DLHC_REGISTER_MAG_IRC_REG_M			= 0x0C
  __LSM303DLHC_REGISTER_MAG_TEMP_OUT_H_M		= 0x31
  __LSM303DLHC_REGISTER_MAG_TEMP_OUT_L_M		= 0x32

  def getPiRevision(self):
    "Gets the version number of the Raspberry Pi board"
    # Courtesy quick2wire-python-api
    # https://github.com/quick2wire/quick2wire-python-api
    try:
      with open('/proc/cpuinfo','r') as f:
        for line in f:
          if line.startswith('Revision'):
            return 1 if line.rstrip()[-1] in ['1','2'] else 2
    except:
      return 0

  # Constructor
  def __init__(self, address_accel=0x19, address_mag=0x1E, debug=False, busId=1):
    self.i2c_accel = Adafruit_I2C(address_accel, smbus.SMBus(busId), debug)
    self.i2c_mag = Adafruit_I2C(address_mag, smbus.SMBus(busId), debug)

    self.address_accel = address_accel
    self.address_mag = address_mag
    self.debug = debug

    # Enable the accelerometer
    # 100Hz measurement, normal mode, XYZ enabled
    self.i2c_accel.write8(self.__LSM303DLHC_REGISTER_ACCEL_CTRL_REG1_A, 0x57)
    self.accelDataRate = 0b0101
    self.accelLowPower = 0b0
    self.accelAxes = 0b111
    # continuous update, little endian, +-2G, high resolution disable, 4 wire serial
    self.i2c_accel.write8(self.__LSM303DLHC_REGISTER_ACCEL_CTRL_REG4_A, 0x00)
    self.accelScale = 0b00
    self.accelHiRez = 0b0
    self.accelFactor = 0.001

    # Enable the magnetometer
    # continuous conversion mode
    self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_MR_REG_M, 0x00)
    # temperature unabled, 15Hz minimum output rate
    self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRA_REG_M, 0x10)
    self.tempEnabled = 0b0
    self.magDataRate = 0b100
    # sensor input field range +-1.9 gauss
    self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x40)
    self.magFactor = 1 / 855.0

  def readAccelerations(self):
    "Reads the accelerometer data from the sensor"
    xlo = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_X_L_A)
    if (self.debug):
      print "DBG: accel X lo: 0x%04X (%d)" % (xlo & 0xFFFF, xlo)
    xhi = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_X_H_A)
    if (self.debug):
      print "DBG: accel X hi: 0x%04X (%d)" % (xhi & 0xFFFF, xhi)
    ylo = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_Y_L_A)
    if (self.debug):
      print "DBG: accel Y lo: 0x%04X (%d)" % (ylo & 0xFFFF, ylo)
    yhi = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_Y_H_A)
    if (self.debug):
      print "DBG: accel Y hi: 0x%04X (%d)" % (yhi & 0xFFFF, yhi)
    zlo = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_Z_L_A)
    if (self.debug):
      print "DBG: accel Z lo: 0x%04X (%d)" % (zlo & 0xFFFF, zlo)
    zhi = self.i2c_accel.readU8(self.__LSM303DLHC_REGISTER_ACCEL_OUT_Z_H_A)
    if (self.debug):
      print "DBG: accel Z hi: 0x%04X (%d)" % (zhi & 0xFFFF, zhi)

    accelData = Obj3D()
    accelData.x = (((xhi << 8) + xlo) >> 4)
    accelData.y = (((yhi << 8) + ylo) >> 4)
    accelData.z = (((zhi << 8) + zlo) >> 4)

    return accelData

  def readAccelerationsG(self):
    "Returns accelerometer reading in G unit"
    accelData = self.readAccelerations()

    accelVal3D = Obj3D()
    accelVal3D.x = self.__twos_comp(accelData.x, 12) * self.accelFactor
    accelVal3D.y = self.__twos_comp(accelData.y, 12) * self.accelFactor
    accelVal3D.z = self.__twos_comp(accelData.z, 12) * self.accelFactor
    return accelVal3D

  def setAccelerometerDataRate(self, rate):
    "Sets the accelerometer data rate"
    if rate == 0:
      self.accelDataRate = 0b0000
    elif rate == 1:
      self.accelDataRate = 0b0001
    elif rate == 10:
      self.accelDataRate = 0b0010
    elif rate == 25:
      self.accelDataRate = 0b0011
    elif rate == 50:
      self.accelDataRate = 0b0100
    elif rate == 100:
      self.accelDataRate = 0b0101
    elif rate == 200:
      self.accelDataRate = 0b0110
    elif rate == 400:
      self.accelDataRate = 0b0111
    elif rate == 1620:
      self.accelDataRate = 0b1000
      self.accelLowPower = 0b1
    elif rate == 1344:
      self.accelDataRate = 0b1001
      self.accelLowPower = 0b0
    elif rate == 5376:
      self.accelDataRate = 0b1001
      self.accelLowPower = 0b1
    else:
      print "setAccelerometerDataRate can be 0, 1, 10, 25, 50, 100, 200, 400, 1344, 1620 or 5376"
    self.__setCtrlReg1A()

  def setAccelerometerLowPowerMode(self, val):
    "Sets the accelerometer power mode"
    if val == False:
      self.accelLowPower = 0b0
    elif val == True:
      self.accelLowPower = 0b1
    else:
      print "setAccelerometerLowPowerMode takes boolean input only"
    self.__setCtrlReg1A()

  def setAccelerometerScale(self, scale):
    "Sets the accelerometer measurement scale"
    if scale == 2:
      self.accelScale = 0b00
      self.accelFactor = 0.001
    elif scale == 4:
      self.accelScale = 0b01
      self.accelFactor = 0.002
    elif scale == 8:
      self.accelScale = 0b10
      self.accelFactor = 0.004
    elif scale == 16:
      self.accelScale = 0b11
      self.accelFactor = 0.012
    else:
      print "setAccelerometerScale takes values 2, 4, 8 or 16 only"
    self.__setCtrlReg4A()

  def setAccelerometerHighResolution(self, val):
    "Sets the accelerometer high resolution mode"
    if val == False:
      self.accelHiRez = 0b0
    elif val == True:
      self.accelHiRez = 0b1
    else:
      print "setAccelerometerHighResolution takes boolean input only"
    self.__setCtrlReg4A()

  def __setCtrlReg1A(self):
    param = (self.accelDataRate << 4) + (self.accelLowPower << 3) + self.accelAxes
    if (self.debug):
      print "New CTRL_REG1_A(20h): 0x%02X (%d)" % (param & 0xFF, param)
    self.i2c_accel.write8(self.__LSM303DLHC_REGISTER_ACCEL_CTRL_REG1_A, param)

  def __setCtrlReg4A(self):
    param = (self.accelScale << 4) + (self.accelHiRez << 3)
    if (self.debug):
      print "New CTRL_REG4_A(23h): 0x%02X (%d)" % (param & 0xFF, param)
    self.i2c_accel.write8(self.__LSM303DLHC_REGISTER_ACCEL_CTRL_REG4_A, param)

  def readMagnetics(self):
    "Reads the magmetometer data from the sensor"
    xlo = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_X_L_M)
    if (self.debug):
      print "DBG: mag X lo: 0x%04X (%d)" % (xlo & 0xFFFF, xlo)
    xhi = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_X_H_M)
    if (self.debug):
      print "DBG: mag X hi: 0x%04X (%d)" % (xhi & 0xFFFF, xhi)
    ylo = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_Y_L_M)
    if (self.debug):
      print "DBG: mag Y lo: 0x%04X (%d)" % (ylo & 0xFFFF, ylo)
    yhi = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_Y_H_M)
    if (self.debug):
      print "DBG: mag Y hi: 0x%04X (%d)" % (yhi & 0xFFFF, yhi)
    zlo = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_Z_L_M)
    if (self.debug):
      print "DBG: mag Z lo: 0x%04X (%d)" % (zlo & 0xFFFF, zlo)
    zhi = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_OUT_Z_H_M)
    if (self.debug):
      print "DBG: mag Z hi: 0x%04X (%d)" % (zhi & 0xFFFF, zhi)

    magData = Obj3D()
    magData.x = (((xhi & 0x000F) << 8) + xlo)
    magData.y = (((yhi & 0x000F) << 8) + ylo)
    magData.z = (((zhi & 0x000F) << 8) + zlo)

    return magData

  def readMagneticsGauss(self):
    "Returns magnetometer readings in gauss unit"
    magData = self.readMagnetics()

    magVal3D = Obj3D()
    magVal3D.x = self.__twos_comp(magData.x, 12) * self.magFactor
    magVal3D.y = self.__twos_comp(magData.y, 12) * self.magFactor
    magVal3D.z = self.__twos_comp(magData.z, 12) * self.magFactor
    return magVal3D

  def readMagneticHeading(self):
    "Return the heading in degrees"
    magData = self.readMagneticsGauss() # fixed thanks to aufder's input 20130210
    return math.degrees(math.atan2(magData.x, magData.z))

  def setMagnetometerRange(self, range=1.3):
    "Sets the gain for magnetometer"
    if range == 1.3:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x20)
      self.magFactor = 1 / 1100.0
    elif range == 1.9:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x40)
      self.magFactor = 1 / 855.0
    elif range == 2.5:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x60)
      self.magFactor = 1 / 670.0
    elif range == 4.0:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x80)
      self.magFactor = 1 / 450.0
    elif range == 4.7:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0xA0)
      self.magFactor = 1 / 400.0
    elif range == 5.6:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0xC0)
      self.magFactor = 1 / 330.0
    elif range == 8.1:
      self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0xE0)
      self.magFactor = 1 / 230.0
    else:
      print "setMagnetometerRange values allowed are 1.3, 1.9, 2.5, 4.0, 4.7, 5.6 or 8.1."

  def setTempEnabled(self, val=False):
    if val == False:
      self.tempEnabled = 0b0
    elif val == True:
      self.tempEnabled = 0b1
    else:
      print "setTempEnabled takes boolean input only"
    self.__setCraRegM()

  def setMagDataRate(self, rate=15.0):
    if rate == 0.75:
      self.magDataRate = 0b000
    elif rate == 1.5:
      self.magDataRate = 0b001
    elif rate == 3.0:
      self.magDataRate = 0b010
    elif rate == 7.5:
      self.magDataRate = 0b011
    elif rate == 15:
      self.magDataRate = 0b100
    elif rate == 30:
      self.magDataRate = 0b101
    elif rate == 75:
      self.magDataRate = 0b110
    elif rate == 220:
      self.magDataRate = 0b111
    else:
      print "setMagDataRate can be 0.75, 1.5, 3.0, 7.5, 15, 30, 75 or 220"
    self.__setCraRegM()

  def __setCraRegM(self):
    param = (self.tempEnabled << 7) + (self.magDataRate << 2)
    if (self.debug):
      print "New CRA_REG_M(00h): 0x%02X (%d)" % (param & 0xFF, param)
    self.i2c_mag.write8(self.__LSM303DLHC_REGISTER_MAG_CRA_REG_M, param)

  def readTemperature(self):
    "Reads the temperature data from the sensor"
    tlo = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_TEMP_OUT_L_M)
    if (self.debug):
      print "DBG: Temp lo: 0x%04X (%d)" % (tlo & 0xFFFF, tlo)
    thi = self.i2c_mag.readU8(self.__LSM303DLHC_REGISTER_MAG_TEMP_OUT_H_M)
    if (self.debug):
      print "DBG: Temp hi: 0x%04X (%d)" % (thi & 0xFFFF, thi)
    return ((thi << 8) + tlo) >> 4

  def readTemperatureCelsius(self):
    "Returns temprature reading in celsius unit"
    temp = self.readTemperature()
    return (self.__twos_comp(temp, 12) / 8.0) + 18

  def __twos_comp(self, val, bits):
    "compute the 2's compliment of int value val"
    if( (val&(1<<(bits-1))) != 0 ):
        val = val - (1<<bits)
    return val

    return 0

class Obj3D :
	x = None
	y = None
	z = None

class LSM303DLHCLibraryTests(unittest.TestCase):
	def setUp(self):
		self.lsm = LSM303DLHC()
		self.lsm.setTempEnabled(True)
		pass

	def tearDown(self):
		self.lsm = None
		pass

	def test_reading_raw(self):
		count = 0
		print ""
		while count < 3:
			time.sleep(0.25)
			accel = self.lsm.readAccelerations()
			mag = self.lsm.readMagnetics()
			temp = self.lsm.readTemperature()
			print "Timestamp: %s" % datetime.now().isoformat() #strftime('%Y-%m-%dT%H:%M:%S(%Z)')
			print "Accel X: 0x%04X (%d) Y: 0x%04X (%d) Z: 0x%04X (%d)" % (accel.x & 0xFFFF, accel.x, accel.y & 0xFFFF, accel.y, accel.z & 0xFFFF, accel.z)
			print "Mag   X: 0x%04X (%d) Y: 0x%04X (%d) Z: 0x%04X (%d)" % (mag.x & 0xFFFF, mag.x, mag.y & 0xFFFF, mag.y, mag.z & 0xFFFF, mag.z)
			print "Temp   : 0x%04X (%d)" % (temp & 0xFFFF, temp)
			count = count + 1
		pass

	def test_reading_converted(self):
		count = 0
		print ""
		while count < 3:
			time.sleep(0.25)
			accel = self.lsm.readAccelerationsG()
			mag = self.lsm.readMagneticsGauss()
			temp = self.lsm.readTemperatureCelsius()
			heading = self.lsm.readMagneticHeading()
			print "Timestamp: %s" % datetime.now().isoformat() #strftime('%Y-%m-%dT%H:%M:%S(%Z)')
			print "Accel X: %6.3f G,     Y: %6.3f G,     Z: %6.3f G" % (accel.x, accel.y, accel.z)
			print "Mag   X: %6.3f gauss, Y: %6.3f gauss, Z: %6.3f gauss" % (mag.x, mag.y, mag.z)
			print "Temp:    %6.3f C" % (temp)
			print "Heading: %6.3f" % (heading)
			count = count + 1
		pass

if __name__ == '__main__':
	#unittest.main()
	suite = unittest.TestLoader().loadTestsFromTestCase(LSM303DLHCLibraryTests)
	unittest.TextTestRunner(verbosity=2).run(suite)
