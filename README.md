# ICMars
<p>Basic code for SIPI rover projects</p>

<p>All code has been developed using ROS Kinetic</p>

<hr>

<h3>The following drivers are needed to use code:</h3>

<p>ROS drivers to use usb camera</p>
<pre>sudo apt install ros-kinetic-usb-cam</pre>

<p>ROS drivers fro indoor MarvelMind GPS system</p>
<pre>sudo apt install ros-kinetic-marvelmind-nav</pre>

<p>ROS drivers for web connection</p>
<pre>sudo apt install ros-kinetic-rosbridge-server</pre>

<p>ROS drivers for usb camera connetion to webserver</p>
<pre>sudo apt-get install ros-kinetic-web-video-server</pre>

<p>ROS drivers to use rosserial</p>
<pre>sudo apt install ros-kinetic-rosserial</pre>

<p>ROS drivers to connect to Arduino using rosserial</p>
<pre>sudo apt install ros-kinetic-rosserial-arduino</pre>

<hr>

<h3>If ROS Desktop was installed, not ROS Desktop full, the following will also need to be installed</h3>

<p></p>
<pre>sudo apt install ros-kinetic-camera-info-manager</pre>

<p></p>
<pre>sudo apt install ros-kinetic-theora-image-transport</pre>

<p></p>
<pre>sudo apt install ros-kinetic-image-view</pre>

<hr>

<h3>Hardware Problems</h3>
<p>During testing, the Logitech C270 usb camera appeared to work fine with the first test.
   But, the camera crashed when the code was ran a second time.
   The usb camera had to unplugged and plugged back in order for it to work again.
   But, the camera would again crash when running the code a second time.
   The C270 camera was replced with a logitech C310 camera that also experienced similar problems as the first camera used.
   This time, the camera would stop working with the third run of the this code, instead of the second.
</p>
<h5>Recommended Cameras</h5>
<p>
   The cameras that have been tested and work are the Logitech c170 and Logitech C920.
</p>
