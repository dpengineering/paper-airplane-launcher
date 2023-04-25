# Paper-Airplane-Launcher Documentation :

## Gadgets

###Maxbotix Ultrasonic Sensor
A sensor deployed onto the front of the exhibit requiring an open area in order to permit a launch. The safe area can be adjusted within the safe_launch method

###ODrive
An open source DC motor controller that manages two motors spinning in opposite directions atop the launcher. 

###LED Button
A highly responsive button attached to the front of the exhibit. Turns red during PowerDriver cooldown

###Rotary Encoder

## Main Methods

### launch_plane
Launches the paper airplane by contracting the spinning motors inwards through the use of the Power Drive. 

### check_obstruction
Converts data bytes recieved from the MaxBotix 

### button_action
Is called before scooping balls after the initial scoop to stop the momentum of balls to ensure a successful scoop.
Must be called in a thread to ensure the UI and hardware function as intended.

### encoder_action
Is called before scooping balls after the initial scoop to stop the momentum of balls to ensure a successful scoop.
Must be called in a thread to ensure the UI and hardware function as intended.

##ANYTHING ELSE?

## APRIL 20 2023


