from dpeaDPi.DPiComputer import DPiComputer
from time import sleep


def main():
    #
    # Create a DPiComputer object
    #
    dpiComputer = DPiComputer()

    #
    # Initialize the board to its default values
    #
    dpiComputer.initialize()

    print("Encoder example:")
    clockDivisor = 360  # for hand turned encoders, slow the state machine
    dpiComputer.setEncoderClockDivisor(clockDivisor)  # to reduce noise, otherwise skip this operation

    encoder_number = 1
    for _i in range(50):
        value = dpiComputer.readEncoder(encoder_number)
        print("  Encoder position: " + str(value))
        sleep(.1)

    print("All examples complete")


if __name__ == "__main__":
    main()
