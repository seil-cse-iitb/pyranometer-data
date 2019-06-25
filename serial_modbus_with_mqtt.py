import minimalmodbus
import time
import serial
# import paho.mqtt.client as mqtt
# import paho.mqtt.client as publish
import sys
from threading import Thread
import glob
# from thread import Thread

MQTT_HOST = "10.129.149.9"
MQTT_PORT = 1883
MQTT_CLIENT = "PYRANOMETER"

MQTT_TOPIC_PUB = "test_data/kresit/radiation/pyranometer"
MQTT_TOPIC_PUB_PSOC = "test_data/kresit/psoc/"

client = mqtt.Client(MQTT_CLIENT)


IO_SENSOR1_DATA = 5
IO_RAW_SENSOR1_DATA = 6
IO_BODY_TEMPERATURE = 8

IO_SENSOR2_DATA = 10
IO_RAW_SENSOR2_DATA = 11

slaveAddress = 1

def serial_port():
	""" Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
	"""
    	if sys.platform.startswith('win'):
        	ports = ['COM%s' % (i + 1) for i in range(256)]
    	elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        	# this excludes your current terminal "/dev/tty"
	        ports = glob.glob('/dev/tty[A-Za-z]*')
    	elif sys.platform.startswith('darwin'):
        	ports = glob.glob('/dev/tty.*')
    	else:
        	raise EnvironmentError('Unsupported platform')

    	result = []

    	for port in ports:
        	try:
           		s = serial.Serial(port)
            		s.close()
            		result.append(port)
		except (OSError, serial.SerialException):
            		pass
    	return result

def serial_read(serialPort, port):
	psoc_port = ""
	isPsoc = False
	# print "Serial port is " + str(serialPort)
	print "Serial data over mqtt"
	while True:
		if serialPort.in_waiting:
			data = serialPort.readline()
			print "-------------------------Serial--------------------"
			print(data)
			print "--------------------------------------------------"
			#Publish over MQTT afterwards
			mqtt.publish(MQTT_TOPIC_PUB_PSOC + port[-1], data)

def pyranometer_read(serialPort):
	global slaveAddress
	pyranometer_port = ""
	isPyranometer = False
	while 1:

		try:
			for ports in serial_port():
				if ports[5:11] == "ttyUSB":
					pyranometer_port == ports
					isPyranometer = True

			# if (isPyranometer):
			pyranometer = minimalmodbus.Instrument(serialPort, slaveAddress)
			
			pyranometer.serial.baudrate = 19200
			pyranometer.serial.bytesize = 8
			pyranometer.serial.parity = serial.PARITY_EVEN
			pyranometer.serial.stopbits = 1
			pyranometer.serial.timeout = 0.05
			## print "******\n %s \n*****" %(pyranometer)

			## Read the register values

			dataToSend = "$" + str(slaveAddress) + ","
			registerAddress = 3
			while registerAddress != 10:
				data = str(registerAddress) + "," + str(pyranometer.read_register(registerAddress, 1))
				# temp = data + ","
				dataToSend = dataToSend + data + ","
				registerAddress = registerAddress + 1

			# sensorData  = pyranometer.read_register(IO_SENSOR1_DATA, 1)
			# rawSensorData = pyranometer.read_register(IO_RAW_SENSOR1_DATA, 1)
			# bodyTemperature = pyranometer.read_register(IO_BODY_TEMPERATURE, 1)
			# 
			# sensorData2 = pyranometer.read_register(IO_SENSOR2_DATA, 1)
			# rawSensorData2 = pyranometer.read_register(IO_RAW_SENSOR2_DATA, 1)

			# dataToSend = str(slaveAddress) +","+ str(sensorData) +","+ str(rawSensorData) +","+ str(bodyTemperature)
			dataToSend = dataToSend[:-1] + "#"
			# client.publish(MQTT_TOPIC_PUB, dataToSend)

			print "-------------------------------------------------------------\n"
			print "Variaous data from the sensor is as follows: \n"
			print "Sent data " + dataToSend
			# print " Sensor %d Data: %d \n Raw Sensor Data: %d \n Body Temperature: %d\n" %(slaveAddress, sensorData, rawSensorData, bodyTemperature)
			print "-------------------------------------------------------------\n\n"

			isPyranometer = False
			# else:
			#	print "Pls, connect pyranometer"
		except Exception as e:
			print str(e)

		print "Incrementing slaveAddress"

		if slaveAddress == 4:
			slaveAddress = 1
		else:
			slaveAddress = slaveAddress +1
			
		time.sleep(2)


def on_connect(client, userdata, flags, rc):
	print "Connected to MQTT client with result " + str(rc)

def mqtt_init():

	print "Initializing MQTT"
	client.on_connect = on_connect

	client.connect(MQTT_HOST, MQTT_PORT, 60)

	client.loop_forever()

if __name__ == "__main__":
	
	print "Initializing Serial"
	ports = serial_port()

	noOfPorts = 0

	# MQTT Thread
	# mqtt_thead = Thread(target=mqtt_init)
	# mqtt_thead.start()

	for ports in serial_port():
		if ports[5:11] == "ttyUSB":
			print "Reading pyranometer"
			pyranometer_thread = Thread(target=pyranometer_read, args=(ports,))
			pyranometer_thread.start()
		elif ports[5:11] == "ttyACM":
			print "Threads for PSoC"
			if ports[-1] == "0":
				print(ports)
				serialPort_0 = serial.Serial(ports, 115200)
				serialThread_0 = Thread(target=serial_read, args=(serialPort_0,ports,))
				serialThread_0.start()
			elif ports[-1] == "1":
				print(ports)
				serialPort_1 = serial.Serial(ports, 115200)
				serialThread_1 = Thread(target=serial_read, args=(serialPort_1,ports,))
				serialThread_1.start()
			elif ports[-1] == "2":
				print(ports)
				serialPort_2 = serial.Serial(ports, 115200)
				serialThread_2 = Thread(target=serial_read, args=(serialPort_2,ports,))
				serialThread_2.start()
			elif ports[-1] == "3":
				print(ports)
				serialPort_3 = serial.Serial(ports, 115200)
				serialThread_ = Thread(target=serial_read, args=(serialPort_3,ports,))
				serialThread_3.start()
