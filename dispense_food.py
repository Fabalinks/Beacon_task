import serial
import time

print('starting script...')

arduino = serial.Serial('COM12', 9600)
print(arduino.read_until('\n'))

for rep in range(3):
    arduino.write('f')
    print(arduino.read_until('\n'))
    time.sleep(.3)


print('...script ended.')