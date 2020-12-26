# FLLReplayCode
Code for the FLL Replay season robot game using the LEGO Education Spike Prime Set (45678).\
Works with most Linux and MAC OS X systems.\
MAC OS X doesn't have a connection setup writeup yet, because of testing issues.\
\
## Technologies
Created with:
* Python: 3.9.1
* MicroPython: 1.12\
\
## Setup
##### Setup for Linux with cable connection:
Open the terminal\
Install screen:
```
sudo apt-get ihttps://github.com/nutki/spike-toolsnstall -y screen
```
Plug in your Spike Prime hub\
Extract the USB ACM device ID (example = ttyACM0) and copy:
```
sudo dmesg | grep cdc_acm | grep tty
```
Run to connect:
```
sudo screen /dev/your_id 115200
```
Type CTRL+C to stop the log\
\
##### Setup for Linux with bluetooth:
Open the terminal\
Install screen:
```
sudo apt-get install -y screen
```
Turn on bluetooth for both devices\
Try connecting to the Hub using a GUI\
Find the Hub's bluetooth address by connecting to it via cable and running the commands or finding it in the bluetooth GUI:
```
hub_runtime.hub.bluetooth.info()
```
Creates a binding between the RFCOMM device and a remote Bluetooth device:\
Note: If you shut down your computer the device get's deleted and you'll have to rerun the command at startup.
```
sudo rfcomm bind /dev/rfcomm0 hub_mac_address 1
```
Run to connect:
```
screen /dev/rfcomm0
```
Type CTRL+C to stop the log\
\
##### Setup for running programs on the Hub:
After connecting to the Hub, copy the program that you want to execute\
Then go into the terminal and type in order from left to right:\
CTRL+E CTRL+SHIFT+V CTRL+D\
And the program should execute.\
\
#### For setup for installing programs to the hub:
Use: https://github.com/nutki/spike-tools
