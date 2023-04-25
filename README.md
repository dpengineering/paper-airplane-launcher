# Paper-Airplane-Launcher Documentation :

## Gadgets

###Maxbotix Ultrasonic Sensor
A sensor deployed onto the front of the exhibit requiring an open area in order to permit a launch. The safe area can be adjusted within the safe_launch method

###ODrive
An open source DC motor controller that manages two motors spinning in opposite directions atop the launcher. 

###LED Button
A highly responsive button attached to the front of the exhibit. Defaut to green but switches to red during PowerDriver cooldown(2 seconds).

###Rotary Encoder
A 360 degree encoder that scommunicates directly to the odrive in order to control the speed of the motors.

## Main Methods

### launch_plane
Launches the paper airplane by contracting the spinning motors inwards through the use of the Power Drive. 

### check_obstruction
Converts data bytes recieved from the MaxBotix ultrasonic sensor into integers that tell the distance in millimeters to the nearest object in front of the exhibit.

### button_action
Checks if it is safe to launch a plane and if so, calls launch_plane and changes color to red. If not, stays green and prints a statement telling the user that there is an object within the safe launching distance.

### encoder_action
Runs continuously and finds the change in the position of the encoder. If the change is negative, the velocity of the motors will  increment down by deltaPow(3). If the change is positive, the velocity of the motors will increment up by deltaPow. Every 5 cycles, if the position change is 0 & the encoder is at the alltime max position, the power will increase by increments of 10 until it has reached max speed(90). If the change is 0 & the encoder is at the alltime min position, the power will decrease by increments of 10 until it has reached min speed(0). The user will typically adjust the encoder to their desired position, and the encoder will be calibrated so that the min and max speed is defined.

## APRIL 20 2023


