# R/C to H-Bridge

A PCB to convert R/C PPM signals into H-Bridge compatible PWM and direction signals as used by many popular H bridge boards. This allows
the separation of the R/C signal processing from the H bridge, such that you can combine this board with small to very large H bridge
boards.

The default firmware for this board handles the mixing of the Throttle and Steering channels to drive two differential motors, aka "Tank Style". It also has two AUX inputs that map to two digital outputs that allow R/C channels to toggle the outputs LOW and HIGH based on the R/C value. This can be useful to add things like weapon arm/disarm, self-righting mechanisms, lights, etc.

Many H bridge boards in the 2A range are available on Amazon quite cheap, such as [This one](https://www.amazon.com/dp/B01CC8XI60/ref=sr_ph_1).

# The Board

![PCB](rc_to_hbridge_v2.png)

# Powering

The board requires 5VDC power to operate. It can be powered either from the R/C inputs (which usually come from the R/C receiver which
  gets power from the ESC or BEC) or you can power directly to the + and - port next to the R/C inputs. If powering via the dedicated
  power port, you should disconnect the + in on the R/C inputs.

# Pins

## R/C

The R/C inputs connect directly to an R/C receiver. By default R/C center is 1500uS, and max throw is +/- 500uS in each direction.

All R/C inputs have a Dead Zone which when the input is +/- this value from center (1500uS) the value will not trigger any
change in outputs nor apply power changes to the motors. The default Dead Zone is 25uS.

* T :: Throttle input - expected to be 0 throttle at 1500uS R/C value
* S :: Steering input - expects no left/right adjustment at 1500uS R/C value
* A :: AUX A - Values above R/C center + Dead Zone will turn OUT A to HIGH, values below R/C center - Dead Zone will turn OUT A to LOW
* B :: AUX B - Same as AUX A, but controls OUT B


## H bridge

The H Bridge side of the board is designed to interface to H-Bridge either 4 or 6 pin controllers. The EN pins contain
the PWM signal and the corresponding 1/2 or 3/4 pins will either be LOW/HIGH or HIGH/LOW based on the mixed input of the
throttle/steering R/C signals to control the direction of the motors. During a breaking operation both pins will be set
to HIGH/HIGH. This means 4 pin H bridges will not support a breaking function.

To connect 6 pin H Bridges, simply connect the ENA/1/2 pins to the first motor inputs and ENB/3/4 to the second motors inputs.

To connect a 4 pin H Bridge, connect ENA/1 to the first motor and ENB/4 to the second. The 1 and 4 pins will toggle between
LOW and HIGH to control direction. If you need to change motor direction and cannot do so by reversing the motor wires, you
can use pins 2 and 3 instead.

## OUT

The OUT ports A and B toggle from LOW to HIGH based on the R/C input signals connected to inputs A and B. There is also
5v power available on the OUT port.


## SERIAL

This port is designed as an expansion port for future use and/or hacking. It provides a TTL serial connection to the ATMega
chip along with 5v power.
