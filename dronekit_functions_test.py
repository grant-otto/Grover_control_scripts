#! /usr/bin/python


from dronekit import connect
grover = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print grover.version

cmds=grover.commands
cmds.download()

missionlist=[]
for cmd in cmds:
	print(cmd)
	missionlist.append(cmd)
print missionlist
missionlist[0].command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
cmds.clear()
for cmd in missionlist:
	cmds.add(cmd)

cmds.upload()
