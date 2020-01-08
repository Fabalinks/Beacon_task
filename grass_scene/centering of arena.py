import serial
actuator_port = 'COM7'
actuator = serial.Serial(actuator_port, 9600)
actuator.write('c')
print ('actuators moved')
