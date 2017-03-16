Pozyx ROS Package
====================

This package is made to simplify your life with the Pozyx system, using ROS. 

I am not affiliated to Pozyx and therefore I do not have all the answers.

Pinout
---------

![Pinout for the tags](images/pozyx_pins.jpg)

The jumpers : 

* BOOT0
	* Jumper off : DFU mode
	* Jumper on : Boot
* T/A
	* Jumper off : Anchor
	* Jumper on : Tag

Firmware update
---------

### On Windows

Follow the Tutorial on the Pozyx website

### On Linux

Set Udev rules

```
# execute https://github.com/dhylands/usb-ser-mon/blob/master/mk-udev-rules-stm32.sh
```

Install and use dfu-util

```
sudo apt-get install dfu-util
dfu-util --device 0483:df11 --alt 0 -D /home/unmanneddemo2/Downloads/firmware_v1.0.dfu
```

Roadmap 
--------

Here are all the nodes to be done. If you feel awesome, leave an issue with the part you will PR, and feel free to give your ideas if you think there is a missing node!

* [x] Multiple anchor setup
* [ ] UWB settings setup
* [x] 2D position reading
* [x] 3D position reading
* [ ] Ranging between two tags
* [ ] Robot_pose_ekf integration
* [ ] Positioning of a remote tag
* [ ] Positioning of all tags
* [ ] Tags discovery
 
Contributing
--------

Feel free to contribute and make a pull request to add functionnalities.

Credits
--------

* Alexis Paques (@AlexisTM)

