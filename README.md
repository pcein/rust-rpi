
pigpio.rs is a Rust port of the excellent Python
library pigpio.py: 

http://abyz.co.uk/rpi/pigpio/python.html

This library can be used to write programs communicating
with "pigpiod" running on a Raspberry Pi server. "pigpiod"
takes care of all the low level stuff needed to do GPIO
programming on a Raspberry Pi:

http://abyz.co.uk/rpi/pigpio/pigpiod.html

This repository contains a local copy of the original
pigpio project (under the directory PIGPIO).

Note: The Rust code has been tested on both a Raspberry
Pi 3 and on desktop systems, but not thoroughly.

Documentation: http://pramode.in/2016/08/31/rust-library-for-rpi-gpio-pgming/

Installation
=============

Make sure you have Rust installed on the Raspberry Pi. 
It is easy to do this using "rustup":

https://www.rust-lang.org/en-US/downloads.html

Clone this repository on a Raspberry Pi. The first step
is to build "pigpiod". For that,
 
    cd PIGPIO
    make pigpiod

You should then run "pigpiod" as root.

Next, build and run the sample program in "main.rs":

    cargo run

Before running the Rust code, make sure you set an environment
variable:

    export PIGPIO_ADDR=localhost

Alternately, you can compile and run the Rust code on your 
desktop/laptop. In that case, make sure you assign the IP
address of the Raspberry Pi as value of the environment variable
PIGPIO_ADDR:

    export PIGPIO_ADDR=192.168.0.102

(assuming Raspberry Pi is at IP 192.168.0.102)


