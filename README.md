# FLLReplayCode
Code for the FLL Replay season robot game using the Spike Prime.\
Works with most Linux and MAC OS X systems.
## Technologies
Created with:
* Python: 3.9.1
* MicroPython: 1.12
## Setup
##### For Linux with cable connection:
Open the terminal\
Install screen:\
```
sudo apt-get install -y screen
```
Plug in your Spike Prime hub\
Extract the USB ACM device id (example = ttyACM0) and copy:
```
sudo dmesg | grep cdc_acm | grep tty
```
Run to connect:
```
sudo screen /dev/your_id 115200
```
Type CTRL+C to stop the log

##### For Linux with bluetooth:
Open the terminal\
Install screen:\
```
sudo apt-get install -y screen
```
Turn on bluetooth for both devices\
