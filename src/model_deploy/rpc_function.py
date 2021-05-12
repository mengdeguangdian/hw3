import serial
import time
serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)

s.write(bytes("\r", 'UTF-8'))
line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
print(line)
line=s.readline() # Read an echo string from mbed terminated with '\n' (RPC reply)
print(line)
time.sleep(1)

s.write(bytes("/GUI_Thread/run 3 1\r", 'UTF-8'))
line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
print(line)
line=s.readline() # Set up successfully
print(line)
time.sleep(1)

line=s.readline() # You select () degree
print(line)
time.sleep(1)

line=s.readline() # /myled3/write 1
print(line)
time.sleep(1)

s.write(bytes("/DET_Thread/run 2 1\r", 'UTF-8'))
line=s.readline() # Read an echo string from mbed terminated with '\n' (putc())
print(line)
line=s.readline() # Read an echo string from mbed terminated with '\n' (RPC reply)
print(line)
time.sleep(1)

line=s.readline() # receive reference acceleration vector(average gravity acceleration)
print(line)
time.sleep(1)

line=s.readline() # "The angle is (), which is larger than ()"
print(line)
time.sleep(1)

line=s.readline() # "The angle is (), which is larger than ()"
print(line)
time.sleep(1)

line=s.readline() # "The angle is (), which is larger than ()"
print(line)
time.sleep(1)

line=s.readline() # /myled2/write 1
print(line)
time.sleep(1)

s.close()