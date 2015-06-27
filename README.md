# vicon2mav_mocap (v2m)

### What v2m does:
It connects to the Vicon Datastream over a network, graps the relevant data, edits it according to the att\_pos\_mocap MavLink message and sends the message out to a TCP/IP sever.

**TLDR**: Vicon Datastream in, att_pos_mocap MavLink message out.

### How to use v2m, how to get it running:
1. What libraries are needed:
	* the ViconDataStreamSDK, espially the *client.h* and the *libViconDataStreamSDK_CPP.so*
	* the MavLink library (found here : https://github.com/mavlink/mavlink)
	* a TCP/IP server which can forward information unaltered to a serial port (I use [ser2net](http://sourceforge.net/projects/ser2net/))

2. Before buildingthe code:
	* I included ...
		* *client.h* [here](https://github.com/moskytoo/vicon2mav_mocap/blob/master/main.cpp#L26)
		* *libViconDataStreamSDK_CPP.so* in the .pro file of my Qt project
		* *mavlink.h* [here](https://github.com/moskytoo/vicon2mav_mocap/blob/master/main.cpp#L30)
	* Setting the constants: Constants concerning your system are set in *functions.h*. In the *main.cpp* are four constants that decide in which 'mode' v2m should run.

3. About the PX4 Firmware: Use the [mocap_support_restruct branch](https://github.com/TSC21/Firmware/tree/mocap_support_restruct) by TSC21. It is not part of the master branch yet (26.06.2015), but I suppose it will be merged in soon (follow the development [here](https://github.com/PX4/Firmware/pull/2361)).

4. Make a connection between v2m and the PX4
	* Hardware: I use a USB-Serial connector like [this](http://www.ftdichip.com/Support/Documents/DataSheets/Modules/DS_UMFT234XD.pdf) and use the Telem2 connector of the Pixhawk (for the wiring look [here](https://pixhawk.org/users/wiring#common_uart_pinout)).
	* Software: I used ser2net to create a TCP server to which v2m can connect as a client. Install it with sudo apt-get install ser2net, open the configuration file with sudo gedit /etc/ser2net.conf, add the line 5763:raw:0:/dev/tty/USB0:921600 8DATABITS NONE 1STOPBIT RTSCTS -XONXOFF -LOCAL and run it. Once started ser2net will start everytime with system bootup. (you can probably also can use other software and other settings, this is just what I used)

5. Initiate the Vicon Stream and you should be ready to go.

### Various Things
		 
I used the Qt creator to write the source code and build the application. As this was created as a Qt project, I would recommend doing the same.

This software, as well as the used PX4 Firmware, is still not properly tested, so use it with care.

One issue which isn't resolved yet is the case when the motion capture system doesn't see an object and don't give the correct positions. Right now it's handled this way: If there is no valid data by the Vicon Datastream available, v2m stops sending att_pos_mocap messages to the PX4, in the hope that it's possible to maneuver the drone with the other sensors.

Part of the att_pos_mocap message are quaternions for the attitude data. The current development of the PX4 Firmware only uses the mocap data for yaw / heading. For roll and pitch the IMU is used. This way stability critical data is still supplied in real time.


Julian L. Nicklas

julian.nicklas (at) posteo (.) de
