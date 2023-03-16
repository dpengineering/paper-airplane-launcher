import sys

from dpeaDPi.DPiComputer import DPiComputer
from time import sleep
from threading import Thread
from odrive_helpers import *
from odrive.utils import start_liveplotter

"""Hardware Setup"""
dpiComputer = DPiComputer()
dpiComputer.initialize()

clockDivisor = 360  # for hand turned encoders, slow the state machine
dpiComputer.setEncoderClockDivisor(clockDivisor)  # to reduce noise, otherwise skip this operation


def get_value():
    encoder_number = 1
    value = dpiComputer.readEncoder(encoder_number)
    return value


def print_info(ax: ODriveAxis):
    print(f"Curr State: {ax.axis.current_state}")
    print(f"Input Mode = {ax.axis.controller.config.input_mode}")
    print(f"Control Mode: {ax.axis.controller.config.control_mode}")


def encoder_action(ax: ODriveAxis):
    while True:
        x_val = get_value()
        vel = x_val / 10
        ax.set_vel(vel)
        sleep(.1)


def encoder_action_thread(ax):
    Thread(target=encoder_action, args=(ax,), daemon=True).start()


if __name__ == "__main__":
    od = find_odrive()

    assert od.config.enable_brake_resistor is True, "Check for faulty brake resistor."

    ax0 = ODriveAxis(od.axis0)
    if not ax0.is_calibrated():
        print("calibrating...")
        ax0.calibrate()
    ax0.set_gains()

    try:
        start_liveplotter(lambda: [ax0.axis.encoder.pos_estimate, ax0.axis.controller.input_pos, ])
        encoder_action_thread(ax0)
        while True:
            sleep(10)
    finally:
        ax0.idle()
        dump_errors(od)
        print("DONE")
