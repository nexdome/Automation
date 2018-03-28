# PDM Dome Configurator #

### A Windows based utility for configuring the NexDome Rotator controller ###

This is the new shiny thing when it comes to the NexDome software. Many of the settings that were invisible, never mind editable, can now be seen and easily changed and tested. I also find it very usefull to actually be able to see all the values changing as the dome operates. I have a much better sense for what is going on (and going wrong).

### Examples of what can be changed ###
- Step mode (must match the dip switch settings!)
- Maximum speed
- Acceleration
- Steps per dome rotation (better than auto homing!)
- Home Azimuth
- Park Azimuth

It also includes a simple terminal program so you can send serial commands manually if so desired.

The best part is being able to fine tune the steps per rotation to exactly match your dome.

## Getting Started ##

With your controller connected to the computer, start Configurator. In the lower left of the window is the serial communications section that includes the connection information, a textbox for entering commands into, and an area where messages are shown.

First thing to do is select your Leonardo in the Com port selector. The actual name Leonardo should appear in the text, if not, you probably have USB issues of some sort. Once that is selected, make sure the baud rate is set to 9600 then click the connect button.

![Configurator before connecting](/Docs/img/CFNotConnected.bmp)

### Calibration Process ###
 - Home the dome
 - Move dome back so home switch is not active
 - Click Calibrate and wait while it does it's work. Note, it only turns 1.2x the current Steps per Rotation value so don't start 180 degrees away from the home switch.
 - When that is complete mark or otherwise identify the exact position of the dome then click "Full turn". The dome will rotate exactly one less step than the amount specified in Steps per Rotation. If the dome ends up right back at your mark, you're done. It should be pretty close but probably not perfect.
 - If the dome has not rotated far enough, enter a new position into the textbox beside the "Go to Pos" button and then click to move the dome. Experiment until the dome is exactly at your mark then enter a new value in the StepsPerRotation box then click "Set" to save that value to the EEPROM.
 - The position value wraps back to zero once it hits the value of StepsPerRotation.
 
![Configurator calibration](/Docs/img/Calibrate.png)

 #### Near Future ####
 - [x] Add buttons for relative moves
 - [x] Remove the "measure home" process. It seemed like a good idea when the dome was slow and bouncing.
 - [ ] Make STOP button change colours when dome is stopped or moving.
 - [ ] Add Shutter control.
 - [ ] Redo communications as I develop the firmware.
 
 #### Futher down the pipe ####
 - Don't know!
 