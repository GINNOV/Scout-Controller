# SPY1 Project (Scout)

It's a navigational robot equipped with sensors and a camera, including a night vision capability.

To control Scout Bot we need to [setup](./docs/ros_setup.md) the ROS environment compatible with the bot hardware on your Mac.

## Requirements
I built a few tools and apps for different purposes. I couldn't find anything in Python for macOS, so I reversed-engineered some or leveraged other projects (Linux-based) to put things together. Works reliably for what I needed it to do.

## Dependencies
It's a Python program, so to run one of the two files, you need to install their dependencies.
If you're familiar with UV, that would be the easiest otherwise, use the traditional way:
Just run ``` pip install  -r requirements.txt ``` to install the dependencies

# How to run it
There are two files:
* `getcolorcamera.py` streams live from the bot in color to your Mac. No joystick support.
* `snoop.py` supports joystick, b/w or color. Change the IP address inside the code where your ROS rely server is running. Without a rely server it won't work.

## Set ROS server
Edit the IP address and port based on your settings
ROS_HOST_ADDRESS = "192.168.86.36:11311"
## Launch it
`python3 snoop.py`
If you want to customize the window and see some logging use this:
`python sp1one.py --windowX 1280 --windowY 720 --verbose`

# Movement keys
If you don't have a joystick connected to the machine where the code runs, you can use the keyboard to pilot the bot.
Below are all supported keys.

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

# Some tools
Under the tools folder, there are useful utilities hacked together for reverse engineering nodes and other ROS stuff. I was completely new to the framework when I did this, so I built bricks and mortar to figure things out. Your mileage may vary based on your ROS knowledge. Have fun
