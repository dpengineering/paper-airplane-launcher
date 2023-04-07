import sys
import serial
import time
import os

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


"""
# print(safe_launch(port1))
            if safe_launch(port1) < 1500:
                print("Object Detected In Flight Path...")
            else:
            
            port1
            
            port1
            
            port1
            
            port"""


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


def encoder_action(axis1):
    while True:
        val = dpiComputer.readEncoder(0)
        vel = val / 10
        if val >= 0:
            axis1.set_vel(vel)
            # print(axis1.get_vel())
        else:
            axis1.set_vel(0)
            # print(axis1.get_vel())


def encoder_action_thread(ax, ):
    Thread(target=button_action, args=(3,)).start()
    Thread(target=encoder_action, args=(ax,)).start()


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

    od.axis1.controller.config.axis_to_mirror = 0
    od.axis1.controller.config.mirror_ratio = -1.0
    od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    od.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis1.controller.config.input_mode = INPUT_MODE_MIRROR
    od.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    od.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    od.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    od.axis0.controller.input_vel = 1

    clockDivisor = 360
    dpiComputer.setEncoderClockDivisor(clockDivisor)

    if ax0.is_calibrated and ax1.is_calibrated():
        print("calibrated motors")
    else:
        ax0.calibrate()
        ax1.calibrate()
        print("calibrating...")

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
        # start_liveplotter(lambda: [ax0.axis.encoder.vel_estimate, ax0.axis.controller.input_vel])
        encoder_action_thread(ax0)  # using axis zero
        # while True:
        # sleep(10)
    finally:
        ax0.idle()
        dump_errors(od)
        print("DONE")
