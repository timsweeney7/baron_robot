#! /usr/bin/env python3

import RPi.GPIO as gpio


def init():
    gpio.setmode(gpio.BOARD)

    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)


def gameover():
    # set all pins to low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

    gpio.cleanup()


init()
gameover()
