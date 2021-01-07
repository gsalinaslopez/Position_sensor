
Position sensor
===================================


Introduction
------------


Pre-requisites
--------------

### Compile
- Android SDK 27
- Android Build Tools v27.0.2
- Android Support Repository

### Run
- Minimun SDK version 21
- adb shell

Getting Started
---------------

To interface with the application via the command line connect the phone to the computer via USB and then run the following from the command line to:
```
adb forward tcp:6100 tcp:6666
```
The above command forwards the computer (host) port 6100 to the app's port 6666
change the first parameter to whatever you need, leave the second parameter intact.

Once setup the forwarding rule, open up any TCP socket application and connect to the specified host port *(port 6100 in the above example)*

Once connected, send the following string to start listening to position updates:
```
start [-p polling rate] [-e event detection threshold] [-xt movement detection threshold (x-axis)] [-yt movement detection threshold (y-axis)]
```
For example, running
```
start -p 500 -e 0.2 -xt 1.5 -yt 1.0
```
Will start the position updates with an log rate of 500ms, set the event detection threshold to 0.2 and movement detection threshold to 1.5 for the x-axis and 1.0 for the y-axis

The default values are -p 100ms, -e 0.3, -xt 1.0 and -yt 1.0, if not specified.

The app output format is as follows:
```
[xraw:val, yraw:val, xsqr:val, ysqr:val, compass:val, pitch:val]
```

*xraw* and *yraw* are the acceleromenter's raw output value.

*xsqr* and *ysqr* are the squarewave output value, let it be 1, -1 or 0

*compass* is the degree with respect to earth's geomagnetic north

*pitch* is the smartphone's 'lean' value in degrees.

Workflow
---------------
Included is a basic TCP socket script called **client.py** and the apk called **Application-debug.apk**

Usual workflow would go as follows:

1. Execute *adb forward tcp:6666 tcp:6666*
2. Start the application, stay at 'camera' mode
3. Execute **client.py** script and type *start* with your desired parameters (start alone works fine with default parameters)
4. Check **client.py** stdout for sensor log output


Demo
---------------
![alt text](demo.gif)


SideNotes
---------------
You have to be in the 'camera' mode for the TCP command to work, don't send the command when in 'chart' mode. Fix is on the way

Whenever running a new configuration, kill and restart the app first, and then initiate a new TCP connection.
