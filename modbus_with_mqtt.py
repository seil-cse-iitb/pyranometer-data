import minimalmodbus
import time
import serial
import paho.mqtt.client as mqtt
import paho.mqtt.client as publish


MQTT_HOST = "10.129.149.9"
MQTT_PORT = 1883
MQTT_CLIENT = "PYRANOMETER"

MQTT_TOPIC_PUB = "test_data/kresit/radiation/pyranometer"

client = mqtt.Client(MQTT_CLIENT)


IO_SENSOR1_DATA = 5
IO_RAW_SENSOR1_DATA = 6
IO_BODY_TEMPERATURE = 8

IO_SENSOR2_DATA = 10
IO_RAW_SENSOR2_DATA = 11

slaveAddress = 1

def pyranometer_read():
	global slaveAddress
	while 1:

		try:
			pyranometer = minimalmodbus.Instrument('/dev/ttyUSB0', slaveAddress)
			
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
			client.publish(MQTT_TOPIC_PUB, dataToSend)

			print "-------------------------------------------------------------\n"
			print "Variaous data from the sensor is as follows: \n"
			print "Sent data " + dataToSend
			# print " Sensor %d Data: %d \n Raw Sensor Data: %d \n Body Temperature: %d\n" %(slaveAddress, sensorData, rawSensorData, bodyTemperature)
			print "-------------------------------------------------------------\n\n"
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

	print "Reading pyranometer"
	pyranometer_read()

	client.loop_forever()

	# print "Reading pyranometer"
	# pyranometer_read()


if __name__ == "__main__":
	mqtt_init()
