#!/usr/bin/python

####################################
# Written by Jeremy Dahan for Trublion.org
# Based on an initial sketch by Mark J. Hughes for AllAboutCircuits.com
####################################


## Requires to fix the read function in the python implementation of libmraa, named mraa.py :
# line 1128 : 

    # def read(self, length):
    #     """
    #     read(I2c self, int length) -> int

    #     Parameters
    #     ----------
    #     length: int
        
    #     """
    #     return _mraa.I2c_read(self, length)




import time
import mraa
import struct

import multiprocessing


class TLV493D:
	def __init__(self, address=0x5e, delaytime=0.01, debug=False):
		if(debug) : print("Initialising the sensor")

		self.magSensor = mraa.I2c(2, True)
		self.magSensor.address(address)
		self.rbuffer = bytearray([0]*10)
		self.magSensor.writeReg(0x00, 0x05)
		self.latestMeasurements = (0,0,0,0,False)
		#self.shouldContinueContinuousMeasurements = True
		#self.latestMeasurementsTime = 0
		if(debug) : print("Sensor set to low power mode")

	@staticmethod
	def decodeX( a,  b):
		ans = ( a << 4 ) | (((b & 0b11110000) >> 4) & 0b00001111)
		if( ans >= 2048):
			ans = ans - 4096
		return ans

	@staticmethod
	def decodeY( a,  b):
		ans = (a << 4) | (b & 0b00001111)
		if( ans > 2048):
			ans = ans - 4096
		return ans

	@staticmethod
	def decodeZ( a,  b):
		ans = (a << 4) | (b & 0b00001111)
		if( ans >= 2048):
			ans = ans - 4096
		return ans
	@staticmethod
	def decodeT( a,  b):
		a &= 0b11110000
		ans = (a << 4) | b
		if( ans > 2048):
			ans -= 4096
		return ans
	@staticmethod
	def convertToMag(a):
		return round(a * 0.098, 3)
	@staticmethod
	def convertToCelsius(a):
		return round((a-320)* 1.1, 3)

	def getRawMagTempValues(self):
		try:
			answer = self.magSensor.read(7)
			self.rbuffer = struct.unpack('BBBBBBB', answer)
			x = self.decodeX(self.rbuffer[0],self.rbuffer[4])
			y = self.decodeY(self.rbuffer[1],self.rbuffer[4])
			z = self.decodeZ(self.rbuffer[2],self.rbuffer[5])
			t = self.decodeT(self.rbuffer[3],self.rbuffer[6])
	
			if((self.rbuffer[3] & 0b00000011) != 0): # If bits are not 0, TLV is still reading Bx, By, Bz, or T
				print("Data read error!")
				return (0,0,0,0,False)
			else:
				return(x,y,z,t, True)
		except Exception as e:
			print(e)
			print("Data collecting error!")
			return (0,0,0,0,False)
		
	def getConvertedMagTempValues(self):
		x,y,z,t,b = self.getRawMagTempValues()
		return(self.convertToMag(x),self.convertToMag(y),self.convertToMag(z),self.convertToCelsius(t),b)

	# def initialiseContinuousMeasurements(self):
	# 	while(self.shouldContinueContinuousMeasurements):
	# 		p = multiprocessing.Process(target=self.continuousMeasurements)
	# 		p.start()
	# 		p.join(0.10)
	# 		p.terminate()

	# def continuousMeasurements(self):
	# 	self.latestMeasurements = self.getConvertedMagTempValues()
	# 	time.sleep(0.05)

	def getLatestMeasurements(self):
		return self.latestMeasurements
		
if __name__ == '__main__':
	mag = TLV493D(address=0x5e)
	while(True):
		print(mag.getConvertedMagTempValues())
		time.sleep(0.5)
