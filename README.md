# cortex_ros_bridge
A Motion Analysis Bridge to ROS

**Compatibility:** Has been tested with Ubuntu 14.04, ROS Indigo, and Cortex 5.5.

**Overview:** This is a ROS package that was built to act as a linux interface for Motion Analysis' motion capture system an Cortex application. The ROS package communicates with the Cortex application over the network to get motion capture data and then publishes the data. The system, data publishers, and services offered by this ROS package are explained below.

# Requirements

To run this software you will also need to contact Motion Analysis Customer Support (support@motionanalysis.com) and ask them for their SDK (specifically "cortex_sdk_linux") as this package requiures that. Once you have that you will need to make a few modifications to it as follows:

**All changes are made in the cortex.cpp file**

1. Locate the line that is as follows: 
<pre><code>LOCAL unsigned short wCortexPort = xxxx
</code></pre>
and change the default port number to something memorbale (we used 1025).
2. Locate the line that is as follows: 
<pre><code>LOCAL unsigned short wMultiCastPort = xxxx
</code></pre>
and again change the default port number to something memorable (we used 1026).

Once all the above changes have been made you will need to put these files into your project, specifically:

1. Place the header files in the *include/ folder.
2. Place the source files in the *src/ folder.

Once you have that taken care of you will need to make some changes within Cortex itself. Go to the Windows machine that is running Cortex and go to *Tools -> Settings* then navigate to the *System* tab. In this tab you will see a SDK Streaming section. You will need to check the fillowing settings in that section:

1. SDK2 Enabled is checked.
2. The NIC Address is the external address of the windows computer itself (not *127.0.0.1*).
3. The General Request Port has the same port number as you entered for the "wCortexPort" variable in the cortex.cpp file.
4. The Data Multicast Port has the same port number as you entered for the "wMulticastPort" variable in the cortex.cpp file.

Also, this should go without saying, but the computers should be able to see eachother on the network (i.e. ping eachother). This has not been tested using WiFi, only ethernet connected computers. 

# Usage

To run this package you will need to update the launch file to have your cortex pc and linux pc IP addresses. Then, the package can be run using:

<pre><code>roslaunch cortex_bridge cortex_bridge.launch
</code></pre>

# Services and Topics

There are at lease two topics exposed, there can be more if the user has decided to add multiple bodies of interest. They are shown below:

	- Topic for each body, the bodyname itself will be the topic
	- Topic for visualization markers OR non-visualization markers (can switch between them in the code by leaving or commenting out "#define PUBLISH_VISUALIZATION_MARKERS 1"), topic will be either vis_markers or novis_markers

The non visualization markers published will have the following structure:

	- cortex bridge::Markers
		- std msgs/Header header
		- uint32 frame number
		- cortex bridge/Marker[] markers
	- cortex bridge::Marker
		- string marker name
		- string subject name
		- string segment name
		- geometry msgs/Point translation
		- bool occluded

There is a single service exposed that can be used to set the body's current position to the origin (0, 0, 0) position. 

	- cortexSetOrigin

The service request has 4 parameters:

	- subject_name
		- the name of the body we want to set the origin of
	- segment_name
		- not currently implemented
	- n_measurements
		- number of measurements to take, the average becoming the offset
	- z_offset
		- a z offset of the origin (i.e. if you want the origin to be 1 m above it's current position, this value would be 1)

# Future Updates/Bug Fixes

	- Future update to make volume constraints (x, y, z) parameters in launch file instead of hard coded values.

	- Add full support for segmented robots

As bugs become known about every effort will be made to fix them. 

# Authors:
* [Tyler Sorey](mailto:tcsorey@gmail.com)
* [Christos Papachristos](mailto:cpapachristos@unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)
