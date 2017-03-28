#!/usr/bin/python
# coding=utf-8
from gevent import monkey
monkey.patch_all()

from flask import Flask
from flask.ext.socketio import SocketIO

app = Flask(__name__)

@app.route('/')
def home():
    return redirect(url_for('static', filename='index.html'))
    

sio = SocketIO(app, async_mode='threading')





import serial
import time
from ST_VL6180x import *
from TLV493D import *
import threading

CNC_SERIAL_PORT = '/dev/ttyACM0'
CNC_SERIAL_BAUDRATE = 115200

X_OFFSET = -226.330
Y_OFFSET = -210.485
Z_OFFSET = -90.000

X_MAX_ABSOLUTE = -5
Y_MAX_ABSOLUTE = -5
Z_MAX_ABSOLUTE = -5

X_HOME = -5.
Y_HOME = -5.
Z_HOME = -5.


POSITION_PRECISION = 0.001

isMachineScanning = False
socketHasToBeQuiet = False
requestingMachineState = False

inputCurrent = 0.0

def linspace(start, stop, step):
    for i in range(int((stop-start)/step + 1)):
        yield round(start + step * i,4)

def cncGoToRelativePosition(x,y,z, speed = 400):
	if(x >= 0 and y >= 0 and z >= 0 and x + X_OFFSET <= X_MAX_ABSOLUTE + 10*POSITION_PRECISION and y + Y_OFFSET <= Y_MAX_ABSOLUTE + 10*POSITION_PRECISION and z + Z_OFFSET <= Z_MAX_ABSOLUTE + 10*POSITION_PRECISION):
		x = x + X_OFFSET
		y = y + Y_OFFSET
		z = z + Z_OFFSET
		cnc.write("N0G1X%fY%fZ%fF%f \n" % (x,y,z,speed))
	else:
		print("Error : Target position is outside safe area.")
		return False


def cncGoToRelativeZ(z, speed = 400):
	if(z >= 0 and z + Z_OFFSET< Z_MAX_ABSOLUTE):
		z = z + Z_OFFSET
		cnc.write("N0G1Z%fF%f \n" % (z,speed))
	else:
		print("Error : Target position is outside safe area.")
		raise Exception("Outside Safe Area")

def cncGoHome():
	cnc.write("G28\n")


def cncGoToRelativePositionAndWait(x,y,z, speed = 400):
	timeout = 60
	try:
		cncGoToRelativePosition(x,y,z,speed)
		latestMachineState = getMachineState(True)
		startTime = time.time()
		while (( \
			(abs(latestMachineState["AbsolutePosition"][0] -(x + X_OFFSET))>=POSITION_PRECISION) or \
			(abs(latestMachineState["AbsolutePosition"][1] -(y + Y_OFFSET))>=POSITION_PRECISION) or \
			(abs(latestMachineState["AbsolutePosition"][2] -(z + Z_OFFSET))>=POSITION_PRECISION) \
			) and (time.time() - startTime < timeout)):
			time.sleep(0.5)
			latestMachineState = getMachineState(True)
		if(time.time() - startTime >= timeout):
			print "Error : timeout reached, machine probably not answering"
			sio.emit('machineInfo', {'status' : "Error", 'message' : "Error : timeout reached, machine probably not answering"})
		else:
			getMachineState(True)
			return False
	except : 
		sio.emit('machineInfo', {'status' : "Error", 'message' : "Target position is outside safe area."})

#x_left_init, x_right_init, y_bottom_init, y_top_init = 0, 10, 0, 10



# example String : "<Run,MPos:-4.075,-5.000,-5.000,WPos:-4.075,-5.000,-5.000,Buf:2,RX:0,Ln:0,F:1070.>"
def grblParse(grblStr):
	parsedGrbl = {}
	if(len(grblStr)>1 and grblStr[0] == "<" and grblStr[-1] == ">"):
		grblStr = grblStr[1:-1]
		grblSplitted = grblStr.split(",")
		parsedGrbl["State"] = grblSplitted[0]
		parsedGrbl["AbsolutePosition"] = [0,0,0]
		parsedGrbl["AbsolutePosition"][0] = float(grblSplitted[1][5:])
		parsedGrbl["AbsolutePosition"][1] = float(grblSplitted[2])
		parsedGrbl["AbsolutePosition"][2] = float(grblSplitted[3])
		parsedGrbl["RelativePosition"] = [0,0,0]
		parsedGrbl["RelativePosition"][0] = parsedGrbl["AbsolutePosition"][0] - X_OFFSET
		parsedGrbl["RelativePosition"][1] = parsedGrbl["AbsolutePosition"][1] - Y_OFFSET
		parsedGrbl["RelativePosition"][2] = parsedGrbl["AbsolutePosition"][2] - Z_OFFSET
		parsedGrbl["Speed"] = float(grblSplitted[10][2:])
	else:
		print("Error parsing " + grblStr)
		parsedGrbl["State"] = "Unknown"
		parsedGrbl["AbsolutePosition"] = [-1,-1,-1]
		parsedGrbl["RelativePosition"] = [-1,-1,-1]
		parsedGrbl["Speed"] = 0
	return parsedGrbl

def getMachineState(send = False):
	global socketHasToBeQuiet
	global requestingMachineState
	if(not requestingMachineState):
		requestingMachineState = True
		socketHasToBeQuiet = True
		cnc.flushInput()
		cnc.write("?\n")
		grblStr = cnc.readline()
		grblStr = grblStr.rstrip()
		if(send):
			print("Received : '" + grblStr + "'")
		while not(len(grblStr)>1 and grblStr[0] == "<" and grblStr[-1] == ">"):
			time.sleep(0.01)
			cnc.write("?\n")
			grblStr = cnc.readline()
			grblStr = grblStr.rstrip()
			if(send):
				print("Received : '" + grblStr + "'")
		machineState = grblParse(grblStr)
		if(send):
			sio.emit("machineInfo",machineState)
		socketHasToBeQuiet = False
		requestingMachineState = False
		return machineState
	else:
		raise Exception("Machine State has already been requested")



def inwardSquareSpiralPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1):
	x_right_init, x_left_init = max(x_right_init, x_left_init), min(x_right_init, x_left_init)
	y_top_init, y_bottom_init = max(y_top_init, y_bottom_init), min(y_top_init, y_bottom_init)
	x_right_init = x_left_init + int((x_right_init-x_left_init)/dx)*dx
	y_top_init = y_bottom_init + int((y_top_init-y_bottom_init)/dy)*dy
	x_left, x_right, y_bottom, y_top = x_left_init, x_right_init, y_bottom_init, y_top_init
	number_of_turns = int(max((x_right_init - x_left_init)/dx, (y_top_init - y_bottom_init)/dy)/2)
	total_length = max(2 * number_of_turns * ((x_right_init - x_left_init) - number_of_turns * dx) + 2 * number_of_turns * ((y_top_init - y_bottom_init) - number_of_turns * dy), 1)
	current_length = 0
	path = [[x_left, y_bottom, 0]]
	for i in range(number_of_turns):
		y_top = max(y_top-dy, y_bottom)
		current_length += abs(path[-1][0] - x_left) + abs(path[-1][1] - y_top)
		path.append([x_left, y_top, current_length*1.0/total_length])
		x_right = max(x_right-dx, x_left)
		current_length += abs(path[-1][0] - x_right) + abs(path[-1][1] - y_top)
		path.append([x_right,y_top, current_length*1.0/total_length])
		y_bottom = min(y_bottom+dy, y_top)
		current_length += abs(path[-1][0] - x_right) + abs(path[-1][1] - y_bottom)
		path.append([x_right,y_bottom, current_length*1.0/total_length])
		x_left = min(x_left+dx, x_right)
		current_length += abs(path[-1][0] - x_left) + abs(path[-1][1] - y_bottom)
		path.append([x_left,y_bottom, current_length*1.0/total_length])
	return(path)

def outwardSquareSpiralPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1):
	spiral = inwardSquareSpiralPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx, dy)
	return spiral.reverse()



def inwardSquareSpiralGridPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1):
	x_right_init, x_left_init = max(x_right_init, x_left_init), min(x_right_init, x_left_init)
	y_top_init, y_bottom_init = max(y_top_init, y_bottom_init), min(y_top_init, y_bottom_init)
	x_right_init = x_left_init + int((x_right_init-x_left_init)/dx)*dx
	y_top_init = y_bottom_init + int((y_top_init-y_bottom_init)/dy)*dy
	x_left, x_right, y_bottom, y_top = x_left_init, x_right_init, y_bottom_init, y_top_init
	number_of_turns = int(max((x_right_init - x_left_init)/dx, (y_top_init - y_bottom_init)/dy)/2)+1
	total_length = max(2 * number_of_turns * ((x_right_init - x_left_init) - number_of_turns * dx) + 2 * number_of_turns * ((y_top_init - y_bottom_init) - number_of_turns * dy), 1)
	current_length = 0
	y_bottom -= dy # Hack !
	path = []
	for i in range(number_of_turns):
		y = y_bottom
		for j in range(int((y_top-y_bottom)/dy)-1):
			y = round(y + dy, 4)
			current_length = round(current_length + dy, 4)
			path.append([x_left,y, current_length*1.0/total_length])
		y_top = y
		x = x_left
		for j in range(int((x_right-x_left)/dx)-1):
			x = round(x + dx, 4)
			current_length = round(current_length + dx, 4)
			path.append([x,y_top, current_length*1.0/total_length])
		x_right = x
		y = y_top
		for j in range(int((y_top-y_bottom)/dy)-1):
			y = round(y - dy, 4)
			current_length = round(current_length + dy, 4)
			path.append([x_right,y, current_length*1.0/total_length])
		y_bottom = y
		x = x_right
		for j in range(int((x_right-x_left)/dx)-1):
			x = round(x - dx, 4)
			current_length = round(current_length + dx, 4)
			path.append([x,y_bottom, current_length*1.0/total_length])
		x_left = x
	return(path)

def outwardSquareSpiralGridPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1):
	spiral = inwardSquareSpiralGridPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx, dy)
	return spiral[::-1]

def zScanArea(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1, z = 60, sensorPeriod = 0.1):
	output_file = open("Z_Scan_%s.csv" % time.strftime("%d-%m-%y_%H-%M"), "w+")
	print("Output file created")
	output_file.write("# OpenMagneticScanner Z-Scan output file. v0.7 \n")
	output_file.write("# Chosen Z = %f\n" % z)
	output_file.write("X,Y,Detected Distance\n\n")
	spiral = inwardSquareSpiralPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx, dy)
	cncGoToRelativePosition(x_left_init, y_bottom_init, z, 2000)
	latestMachineState = getMachineState()
	while (not latestMachineState["AbsolutePosition"] == [x_left_init + X_OFFSET, y_bottom_init + Y_OFFSET, z + Z_OFFSET ]):
		print("Waiting for the machine to reach target start position\n")
		time.sleep(0.5)
		latestMachineState = getMachineState()
	print("Starting spiral pattern \n")
	for pos in spiral:
		cncGoToRelativePosition(pos[0],pos[1], z)
		latestMachineState = getMachineState()
		time.sleep(0.1)
		while (not latestMachineState["AbsolutePosition"] == [pos[0] + X_OFFSET, pos[1] + Y_OFFSET, z + Z_OFFSET ]):
			distance = distanceSensor.get_distance()
			latestMachineState = getMachineState()
			output_file.write("%f,%f,%f\n" % (latestMachineState["AbsolutePosition"][0] - X_OFFSET, latestMachineState["AbsolutePosition"][1] - Y_OFFSET,distance))
			output_file.flush()
			print("Sensor Info : %f,%f,%f" % (latestMachineState["AbsolutePosition"][0] - X_OFFSET, latestMachineState["AbsolutePosition"][1] - Y_OFFSET,distance))
		print("Spiral : %f%% \n" % (pos[2]*100))
	output_file.close()
	print("Done with that nasty spiral.")

def zScanAreaGrid(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1, z = 60, sensorPeriod = 0.1, timeout=5):
	global scanHasNotBeenStopped
	output_file = open("ZScans/Z_Scan____%s.csv" % time.strftime("%d-%m-%y_%H-%M"), "w+")
	print("Output file created")
	output_file.write("# OpenMagneticScanner Z-Scan output file. v0.7 \n")
	output_file.write("# Chosen Z = %f\n" % z)
	output_file.write("X,Y,Detected Distance\n\n")
	spiral = inwardSquareSpiralGridPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx, dy)
	cncGoToRelativePosition(x_left_init, y_bottom_init, z, 2000)
	latestMachineState = getMachineState()
	sio.emit('zScanInfo', {'status' : "Starting", 'message' : "Waiting for the machine to reach target start position" })
	while ((abs(latestMachineState["AbsolutePosition"][0] - (x_left_init + X_OFFSET))>=POSITION_PRECISION or abs(latestMachineState["AbsolutePosition"][1] - (y_bottom_init + Y_OFFSET))>=POSITION_PRECISION or abs(latestMachineState["AbsolutePosition"][2] - (z + Z_OFFSET))>=POSITION_PRECISION)):
		print("Waiting for the machine to reach target start position\n")
		time.sleep(0.5)
		latestMachineState = getMachineState()
	print("Starting spiral pattern \n")
	sio.emit('zScanInfo', {'status' : "Starting", 'message' : "Starting spiral pattern"})
	for pos in spiral:
		if(scanHasNotBeenStopped):
			cncGoToRelativePosition(pos[0],pos[1], z)
			latestMachineState = getMachineState()
			time.sleep(0.1)
			startTime = time.time()
			while (( \
				(abs(latestMachineState["AbsolutePosition"][0] -(pos[0] + X_OFFSET))>=POSITION_PRECISION) or \
				(abs(latestMachineState["AbsolutePosition"][1] -(pos[1] + Y_OFFSET))>=POSITION_PRECISION) or \
				(abs(latestMachineState["AbsolutePosition"][2] -(z + Z_OFFSET))>=POSITION_PRECISION) \
				) and (time.time() - startTime < timeout)):
				time.sleep(0.01)
				latestMachineState = getMachineState()
			if(time.time() - startTime >= timeout):
				print "Error : timeout reached, machine probably not answering"
				sio.emit('zScanInfo', {'status' : "Error", 'message' : "Error : timeout reached, machine probably not answering"})
			else:
				distance = distanceSensor.get_averaged_distance(5)
				output_file.write("%f,%f,%f\n" % (pos[0], pos[1],distance))
				sio.emit('zScanData', {'x': pos[0], 'y': pos[1], 'z': distance})
				output_file.flush()
				print("Sensor Info : %f,%f,%f" % (pos[0], pos[1],distance))
				print("Spiral : %f%% \n" % round(pos[2]*100, 3))
				sio.emit('zScanInfo', {'status' : "Scanning", 'progress' : round(pos[2]*100, 3)})
	output_file.close()
	
	if(scanHasNotBeenStopped):
		print("Done with that nasty spiral.")
		sio.emit('zScanInfo', {'status' : "done", 'message' : "Done with that nasty spiral."})
	else:
		print("Scan has been stopped. Machine Idle")
		sio.emit('zScanInfo', {'status' : "stopped", 'message' : "Scan has been stopped. Machine Idle"})

def magScanAreaGrid(x_left_init, x_right_init, y_bottom_init, y_top_init, dx = 0.1, dy = 0.1, z_bottom = 20, z_top = 60, dz = 0.1, timeout=5):
	global scanHasNotBeenStopped
	global isMachineScanning
	global inputCurrent
	isMachineScanning = True
	output_file = open("MagScans/MAG_Scan____%s_.csv" % (time.strftime("%d-%m-%y_%H-%M")), "w+")
	#output_file = open("MagScans/MAG_Scan____%s.csv" % time.strftime("%d-%m-%y_%H-%M"), "w+")
	print("Output file created")
	output_file.write("# OpenMagneticScanner Mag-Scan output file. v0.7 \n")
	output_file.write("# Parameters :  x_left_init = %f, x_right_init = %f, y_bottom_init = %f, y_top_init = %f, dx = %f, dy = %f, z_bottom = %f, z_top = %f, dz = %f \n" % (x_left_init,x_right_init,y_bottom_init,y_top_init,dx,dy,z_bottom,z_top,dz))
	output_file.write("X,Y,Z, Bx, By, Bz\n\n")
	spiralIn = inwardSquareSpiralGridPath(x_left_init, x_right_init, y_bottom_init, y_top_init, dx, dy)
	spiralOut = spiralIn[::-1]
	cncGoToRelativePosition(x_left_init, y_bottom_init, z_bottom, 2000)
	latestMachineState = getMachineState(True)
	sio.emit('zScanInfo', {'status' : "Starting", 'message' : "Waiting for the machine to reach target start position" })
	while ((abs(latestMachineState["AbsolutePosition"][0] - (x_left_init + X_OFFSET))>=POSITION_PRECISION or abs(latestMachineState["AbsolutePosition"][1] - (y_bottom_init + Y_OFFSET))>=POSITION_PRECISION or abs(latestMachineState["AbsolutePosition"][2] - (z_bottom + Z_OFFSET))>=POSITION_PRECISION)):
		print("Waiting for the machine to reach target start position\n")
		time.sleep(0.5)
		latestMachineState = getMachineState(True)
	print("Starting spiral pattern \n")
	sio.emit('zScanInfo', {'status' : "Starting", 'message' : "Starting spiral pattern"})
	isSpiralInward = True
	sendIndex = 0
	sendBuffer = [0]*10
	for z in list(linspace(z_bottom, z_top, dz)):
		progressZ = (z-z_bottom)*1.0/(z_top-z_bottom)
		if(isSpiralInward):
			spiral = spiralIn
		else : 
			spiral = spiralOut
		isSpiralInward = not isSpiralInward
		cncGoToRelativeZ(z)
		for pos in spiral:
			if(scanHasNotBeenStopped):
				sendIndex = (sendIndex + 1)%10
				shouldSendMessage = (sendIndex == 0)
				cncGoToRelativePosition(pos[0],pos[1], z)
				try:
					latestMachineState = getMachineState(shouldSendMessage)
				except:
					print("failed to get machine state")
				time.sleep(0.1)
				startTime = time.time()
				while (( \
					(abs(latestMachineState["AbsolutePosition"][0] -(pos[0] + X_OFFSET))>=POSITION_PRECISION) or \
					(abs(latestMachineState["AbsolutePosition"][1] -(pos[1] + Y_OFFSET))>=POSITION_PRECISION) or \
					(abs(latestMachineState["AbsolutePosition"][2] -(z + Z_OFFSET))>=POSITION_PRECISION) \
					) and (time.time() - startTime < timeout)):
					time.sleep(0.01)
					try:
						latestMachineState = getMachineState(shouldSendMessage)
					except:
						print("failed to get machine state")
				if(time.time() - startTime >= timeout):
					print "Error : timeout reached, machine probably not answering"
					sio.emit('magScanInfo', {'status' : "Error", 'message' : "Error : timeout reached, machine probably not answering"})
				else:
					Bx,By,Bz,T, isDataValid = magSensor.getConvertedMagTempValues()
					magStartTime = time.time()
					while((not isDataValid) and (time.time() - startTime < 0.5)):
						Bx,By,Bz,T,isDataValid = magSensor.getConvertedMagTempValues()
						time.sleep(0.05)
					if((time.time() - startTime >= 0.5)):
						print("Timeout...")
					# The position of the sensor imposes us to rearrange the data : Real Bz is -SensorBx, Real Bx is +SensorBy, Real By is -SensorBz
					output_file.write("%f,%f,%f,%f,%f,%f\n" % (pos[0], pos[1],z, By,-Bz,-Bx))
					output_file.flush()
					sendBuffer[sendIndex] = {'x': pos[0], 'y': pos[1], 'z': z, 'Bx' : By, 'Bz' : -Bz, 'Bz' : -Bx}
					if(shouldSendMessage):
						sio.emit('magScanData', sendBuffer)
						sio.emit('magScanInfo', {'status' : "Scanning", 'progress' : round(progressZ, 3)})
						print("Sensor Info : %f,%f,%f,%f,%f,%f\n" % (pos[0], pos[1],z, By,-Bz,-Bx))
						print("Spiral : %f%% \n" % round(progressZ*100, 3))
	output_file.write("### End of scan")
	#inputCurrent -= 0.1
	output_file.close()
	isMachineScanning =False
	if(scanHasNotBeenStopped):
		print("Done with that nasty spiral.")
		sio.emit('magScanInfo', {'status' : "done", 'message' : "Done with that nasty spiral."})
	else:
		print("Scan has been stopped. Machine Idle")
		sio.emit('magScanInfo', {'status' : "stopped", 'message' : "Scan has been stopped. Machine Idle"})


#sio = socketio.Server()
#app = Flask(__name__)

 # wrap Flask application with engineio's middleware
#app = socketio.Middleware(sio, app)
# deploy as an eventlet WSGI server

@sio.on('zScanInstructions')
def zScanInstructionsHandler(arg):
	if(not socketHasToBeQuiet):
		zScanAreaGrid(float(arg["xmin"]), float(arg["xmax"]), float(arg["ymin"]), float(arg["ymax"]), float(arg["dx"]), float(arg["dy"]), float(arg["z"]))

@sio.on('machinePositionInstructions')
def machinePositionInstructionsHandler(arg):
	if(not socketHasToBeQuiet):
		if("x" in arg and "y" in arg):
			if("speed" in arg):
				cncGoToRelativePositionAndWait(float(arg["x"]), float(arg["y"]), float(arg["z"]), float(arg["speed"]))
			else:
				cncGoToRelativePositionAndWait(float(arg["x"]), float(arg["y"]), float(arg["z"]))
		else:
			cncGoToRelativeZ(float(arg["z"]))

@sio.on('magScanInstructions')
def magScanInstructionsHandler(arg):
	if(not socketHasToBeQuiet):
		magScanAreaGrid(float(arg["xmin"]), float(arg["xmax"]), float(arg["ymin"]), float(arg["ymax"]), float(arg["dx"]), float(arg["dy"]), float(arg["zmin"]), float(arg["zmax"]), float(arg["dz"]))



@sio.on('machineStateRequest')
def machineStateRequestHandler():
	if(not socketHasToBeQuiet):
		getMachineState(True)

@sio.on('stopLoops')
def stopLoops():
	global scanHasNotBeenStopped
	scanHasNotBeenStopped = False

@sio.on('stopMachine')
def stopMachine():
	global scanHasNotBeenStopped
	global cnc
	scanHasNotBeenStopped = False
	cnc.write("!")



def initialiseSocketIo():
	global app
	#eventlet.wsgi.server(eventlet.listen(('', 8000)), app)
	sio.run(app, host="0.0.0.0")

def initialiseTheRest():
	global cnc
	global scanHasNotBeenStopped
	scanHasNotBeenStopped = True
	global magSensor
	# distanceSensor = VL6180X()
	# distanceSensor.default_settings()
	magSensor = TLV493D() 
	#magSensor.continuousMeasurements()

	print("Initializing CNC serial Port...")
	cnc = serial.Serial(CNC_SERIAL_PORT, CNC_SERIAL_BAUDRATE, timeout=1)
	latestMachineState = getMachineState(True)
	if(latestMachineState["State"] == "Alarm"):
		print("CNC in alarm mode, unlocking")
		cnc.write('$X\n')
	time.sleep(0.5)
	print("Homing...")
	cnc.write('$H\n')
	time.sleep(2)
	latestMachineState = getMachineState(True)
	while (not latestMachineState["AbsolutePosition"] == [X_HOME,Y_HOME,Z_HOME]):
		print("Waiting for the machine to reach Home\n")
		time.sleep(0.5)
		latestMachineState = getMachineState(True)

	# zScanAreaGrid(0,200,0,200, 2, 2, 20)
	#magScanAreaGrid(190,200,190,200, 2, 2, 80, 85, 2, 5)


if __name__ == '__main__':
	socketThread = threading.Thread(target=initialiseSocketIo)
	socketThread.start()
	initialiseTheRest()