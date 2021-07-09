#!/usr/bin/env python

# bsc_arduino.py
# 2016-10-31
# Public Domain

"""
This code is an example of using the Pi as an I2C slave device
receiving messages from an Arduino acting as the I2C bus master.

On the Pi

The BSC slave SDA is GPIO 18 (pin 12) and the BSC slave
SCL is GPIO 19 (pin 35).

On the Arduino

SDA is generally A4 and SCL is generally A5.

Make the following connections.

GPIO 18 <--- SDA ---> A4
GPIO 19 <--- SCL ---> A5

You should also add pull-up resistors of 4k7 or so on each of
GPIO 18 and 19 to 3V3.  The software sets the internal pull-ups
which may work reliably enough.

On the Arduino use the following code.

#include <Wire.h>

void setup()
{
   Wire.begin(); // join i2c bus as master
}

char str[17];

int x = 0;

void loop()
{
   sprintf(str, "Message %7d\n", x);
   if (++x > 9999999) x=0;

   Wire.beginTransmission(9); // transmit to device #9
   Wire.write(str);           // sends 16 bytes
   Wire.endTransmission();    // stop transmitting

   delay(50);
}
"""
import time
import pigpio

SDA=18
SCL=19

I2C_ADDR=9

def i2c(id, tick):
   global pi

   s, b, d = pi.bsc_i2c(I2C_ADDR)

   if b:

      print(d[:-1])

pi = pigpio.pi()

if not pi.connected:
    exit()

# Add pull-ups in case external pull-ups haven't been added

pi.set_pull_up_down(SDA, pigpio.PUD_UP)
pi.set_pull_up_down(SCL, pigpio.PUD_UP)

# Respond to BSC slave activity

e = pi.event_callback(pigpio.EVENT_BSC, i2c)

pi.bsc_i2c(I2C_ADDR) # Configure BSC as I2C slave

time.sleep(600)

e.cancel()

pi.bsc_i2c(0) # Disable BSC peripheral

pi.stop()

