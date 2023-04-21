# Paper-Airplane-Launcher Documentation :

## Gadgets

###Maxbotix Ultrasonic Sensor
A sensor deployed onto the front of the exhibit requiring an open area in order to permit a launch. The safe area can be adjusted within the safe_launch method

###ODrive

###LED Button

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

## End of year 2023

### Bugs
After running Newton's Cradle for a while the UI will not update as intended, the cursor will not move but updates the values correctly. The reset widgets method should rn on its own thread or the main thread to guarantee it re draws correctly.

