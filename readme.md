Here’s the raw Markdown version — just copy and paste this into your README.md file:

# 🥷 SPY1 Project (Scout)

A navigational robot equipped with sensors and a camera, including night vision capability.

<p align="center">
  <img src="https://www.robotshop.com/cdn/shop/files/moorebot-scout-ai-powered-autonomous-mobile-robot-4.webp?v=1720479033" alt="Scout Robot" style="width:50%;">
</p>

To control Scout Bot, you’ll need to [set up](/docs/ros_setup.md) the ROS environment on your Mac, compatible with the bot’s hardware.

---

## 🛠️ Requirements

I built a few tools and apps for different purposes. I couldn’t find anything in Python for macOS, so I reverse-engineered some pieces or adapted Linux-based projects to make it work. It’s reliable enough for what I needed.

---

## 📦 Dependencies

This is a Python program. To run either script, you need to install the dependencies.

If you’re familiar with [**uv**](https://github.com/astral-sh/uv), that’s the easiest way. Otherwise, use the traditional method:

```bash
pip install -r requirements.txt

If you know what you’re doing, I recommend using uv to create a virtual environment.

⸻

▶️ How to Run

There are two entry points:
	•	getcolorcamera.py — Streams live color video from the bot to your Mac (no joystick support).
	•	snoop.py — Full control with joystick and color/BW modes.
Make sure to set the IP address inside the script to where your ROS relay server is running. Without it, the bot won’t respond.

🧭 Set ROS Server

Edit this line in the script to match your network:

ROS_HOST_ADDRESS = "192.168.86.36:11311"


⸻

🚀 Launch It

Basic launch:

python3 snoop.py

Or customize the window and enable verbose logging:

python3 sp1one.py --windowX 1280 --windowY 720 --verbose


⸻

🎮 Movement Keys

If a joystick isn’t connected, you can use the keyboard:

Key	Action
W / S	Forward / Backward
A / D	Rotate Left / Right
Q / E	Strafe Left / Right
P / O	Increase / Decrease Speed
Space	Take Screenshot
H	Go Home (stub)
9 / 0	Toggle Lights (stub)
Esc	Exit Program


⸻

🧰 Tools

In the tools/ folder, you’ll find small utilities I hacked together to reverse-engineer nodes and experiment with ROS.

I was completely new to the framework when I built this, so I created bricks-and-mortar tools to figure things out.

⚠️ Your mileage may vary depending on your ROS knowledge. Have fun tinkering!

Let me know if you'd like the Markdown as a downloadable `.md` file!
