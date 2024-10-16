# SPY1 Project (Scout)

It's a navigational robot that has sensors and a camera, including night vision.

To control Scout Bot we need to [setup](./docs/ros_setup.md) the ROS environment compatible with the bot hardware on your mac.

## Requirements
I built a few tools and apps for different purposes. I couldn't find anything in Python for macOS, so I reversed-engineered some or leveraged other projects (Linux-based) to put things together. Works reliably for what I needed to do.

Just run ``` pip install  -r requirements.txt ``` to install the dependencies

# Desktop Apps
I made a swift version, too, but I need to clean it up before posting it here (don't hold your breath :-)

The Python version uses PyGame, so you get all the benefits of keyboards and joystick support.

`getcolorcamera.py` streams live from the bot in color to your Mac. No joystick support.
`snoop.py` supports joystick, b/w or color. Change the IP address inside the code where your ROS rely is running.


## Movement keys
Below are all supported keys, or if you have a joystick connected to the host machine where ROS is connected, you can pilot with that, too.

* W: Move forward
* S: Move backward
* A: Rotate left
* D: Rotate right
* Q: Strafe left
* E: Strafe right
* P: Increase speed
* O: Decrease speed
* Space: Take a screenshot
* H: Go home (not implemented yet)
* 9: Turn on light 1 (not implemented yet)
* 0: Turn on light 2 (not implemented yet)
* Esc: Exit program

## experiment and baby tools
under the tools folder there are useful utilties hacked together to reverse engineering nodes and other ROS stuff. I was completely new to the framework so I built bricks and mortar to figure things out. Your mile vary based on your ROS knowledge.