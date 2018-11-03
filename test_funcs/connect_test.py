#! /usr/bin/python


from dronekit import connect
grover = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print grover.version
