import sys
import serial
import time
import os
import math

from dpeaDPi.DPiComputer import DPiComputer
from dpeaDPi.DPiPowerDrive import DPiPowerDrive
from time import time
from serial import Serial
from time import sleep
from threading import Thread
from odrive_helpers import *
from odrive.utils import start_liveplotter

"""Main Functions"""


def launch_plane(wait_time):
    dpiPowerDrive.pulseDriverOn(0, 200)
    sleep(wait_time)


def print_info(ax: ODriveAxis):
    print("Current Limit: ", ax.get_current_limit())
    print("Velocity: ", ax.get_vel())
    print("Velocity Limit: ", ax.get_vel_limit())


def measure(portName):
    global maxwait
    ser = Serial(portName, 57600, 8, 'N', 1, timeout=3)
    timeStart = time()
    valueCount = 0

    while time() < timeStart + maxwait:
        if ser.inWaiting():
            bytesToRead = ser.inWaiting()
            valueCount += 1
            if valueCount < 2:  # 1st reading may be partial number; throw it out
                continue
            testData = ser.read(bytesToRead)
            if not testData.startswith(b'R'):
                # data received did not start with R
                continue
            try:
                sensorData = testData.decode('utf-8').lstrip('R')
            except UnicodeDecodeError:
                # data received could not be decoded properly
                continue
            try:
                mm = int(sensorData)
            except ValueError:
                # value is not a number
                continue
            ser.close()
            print(mm)
            return mm

    ser.close()
    raise RuntimeError("Expected serial data not received")


def safe_launch():
    measurement = measure(serialDevice)
    if measurement > 2000:
        return True
    else:
        return False


def button_action(wait_tim):
    while True:
        button_value = dpiComputer.readRGBButtonSwitch(0)  # read the button
        if button_value:
            red = 255
            green = 0
            blue = 0
            dpiComputer.writeRGBButtonColor(0, red, green, blue)  # set button to RED if pushed
            if safe_launch():
                launch_plane(wait_tim)
            else:
                print("Obstruction Detected in Flight Path")
        else:
            red = 0
            green = 240
            blue = 0
            dpiComputer.writeRGBButtonColor(0, red, green, blue)  # set button to Green if not pushed


def encoder_action():
    adjustedMaxPow = 90
    adjustedMinPow = 0
    deltaPow = 3
    power = 0
    previousPos = dpiComputer.readEncoder(0)
    num = 1
    correctedDeltaPos = 1
    maxpos = 0
    minpos = 0
    while True:
        currentPos = dpiComputer.readEncoder(0) / 20
        print("Current Pos: " + str(currentPos))
        deltaPos = currentPos - previousPos
        if currentPos > maxpos:
            maxpos = currentPos
        if currentPos < minpos:
            minpos = currentPos
        if num % 5 == 0:
            if deltaPos == 0 and currentPos > 0 and currentPos == maxpos:  # no movement + positive encoder value
                if power + 10 > 90:
                    power = 90
                else:
                    power += 10
                correctedDeltaPos = 0
            elif deltaPos == 0 and currentPos <= 0 and currentPos == minpos:  # no movement + negative encoder value
                if power - 10 < 0:
                    power = 0
                else:
                    power -= 10
                correctedDeltaPos = 0
        elif currentPos < previousPos:
            correctedDeltaPos = -1
        elif currentPos > previousPos:
            correctedDeltaPos = 1
        else:
            correctedDeltaPos = 0

        if correctedDeltaPos == 0:
            od.axis0.controller.input_vel = power
            print("No change detected... setting power to absolute extrema")
        elif correctedDeltaPos >= 1:
            if currentPos >= maxpos or power >= adjustedMaxPow:
                print("power @ MAX")
                power = 90
            else:
                power += deltaPow
            od.axis0.controller.input_vel = power
        elif correctedDeltaPos <= -1:
            if currentPos <= minpos or power <= adjustedMinPow:
                print("power @ MIN")
                power = 0
            else:
                power -= deltaPow
            od.axis0.controller.input_vel = power

        previousPos = currentPos
        print("Power: ")
        print(power)
        num += 1


def encoder_action_thread():
    Thread(target=button_action, args=(3,)).start()
    Thread(target=encoder_action, args=()).start()


if __name__ == "__main__":
    od = find_odrive()
    dpiComputer = DPiComputer()
    dpiComputer.initialize()

    """Power Drive"""
    dpiPowerDrive = DPiPowerDrive()
    dpiPowerDrive.setBoardNumber(1)
    dpiPowerDrive.setDriverPWM(0, 100)  # PWM values can be set between 0 and 255.
    print("PowerDrive Initialized: " + str(dpiPowerDrive.initialize()))

    """Odrive"""
    if not od.config.enable_brake_resistor:
        print("Check for faulty brake resistor.")

    ax0 = ODriveAxis(od.axis0, current_lim=25, vel_lim=95)  # left wheel -> moves CCW
    ax1 = ODriveAxis(od.axis1, current_lim=25, vel_lim=95)  # right wheel -> moves CW

    ax0.set_calibration_current(20)
    ax1.set_calibration_current(20)
    ax0.set_gains()
    ax1.set_gains()
    od.axis0.controller.config.vel_integrator_gain = 0.16
    od.axis1.controller.config.vel_integrator_gain = 0.16

    od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    if ax0.is_calibrated and ax1.is_calibrated():
        print("calibrated motors")
    else:
        print("calibrating...")
        ax0.calibrate()
        ax1.calibrate()

    od.axis1.controller.config.axis_to_mirror = 0
    od.axis1.controller.config.mirror_ratio = -1.0
    od.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis1.controller.config.input_mode = INPUT_MODE_MIRROR
    od.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    od.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    od.axis0.controller.config.vel_ramp_rate = 5
    #  od.axis0.controller.config.vel_ramp_rate = 5

    dump_errors(od)

    clockDivisor = 360
    dpiComputer.setEncoderClockDivisor(clockDivisor)

    """Maxbotix Sonar"""
    # Reads serial data from Maxbotix ultrasonic rangefinders
    # Gracefully handles most common serial data glitches
    # Use as an importable module with "import MaxSonarTTY"
    # Returns an integer value representing distance to target in millimeters
    serialDevice = "/dev/ttyUSB0"  # default for RaspberryPi
    maxwait = 3  # seconds to try for a good reading before quitting

    print("Current Limit: ", ax0.get_current_limit())
    print("Velocity Limit: ", ax0.get_vel_limit())

    try:
        encoder_action_thread()  # using axis zero
        while True:
            hah = 0
        # sleep(10)
    finally:
        ax0.idle()
        dump_errors(od)
        print("DONE")
