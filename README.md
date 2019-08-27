# Quest 1: Pseudo-Mechanical Alarm Clock
Authors: Gennifer Norman, Cameron Surman, Rubeena Zaveri 2018-09-25

## Summary

This quests implements an alarm clock with digital display.



## Evaluation Criteria

We decided on the following specifications for a successful solution to this quest:
- Able to display time on the alphanumeric display board.
- User is able to set an alarm via the console.
- The time is able to be set by the user
- Two servos are attached to the device: one indicates seconds and the other minutes.
  - The servos ticks like the arms of an analog clock but only for 180˚ then they reset.


## Solution Design

#### Servos

Controlling the servo position is by PWM with a “neutral” signal of 1.5 ms width at a frequency of about 50 Hz (in the range of 40 to 200 Hz). Our servo has an 180 degree range. The signal input pulse width is in the range of 600-2665ms.  That range was broken up into intervals of 35 so that the servo could have 60 different positions. One servo changes positions every second where as the other changes every minute.

#### Timer Interrupts

The timer is controlled via a 1Hz divided clock within the CPU (Group 0, IDX 0). When an interrupt signal is received, the program transfers control to the interrupt handler, which increments the volatile int SECOND by 1. It then checks if any of the other time values have exceeded a value of 59, and resets/increments accordingly. Setting an alarm defaults to a timer of 1 minute. This value decrements within the interrupt handler (every 1 second), until it reached 0, triggering the alarm state.


#### Alphanumeric Display

The alphanumeric display converses with the ESP32 through I2C connection. I2C is a way to communicate between a microcontroller ("master") and connected device(s) ("slave"). It is useful because it only uses 2 pins from the microcontroller even if multiple devices are connected. Of course
this poses a problem: how can the master transmit data to a specific device if there are multiple devices connected? The answer lies in the way the data is transmitted. Rather than the data being bluntly transmitted, it is a concatenation of multiple elements. The data starts with a start byte, includes an address that is assigned to the slave that is a recipient of the data, of course includes the data, and then asks for an acknowledgment from the slave. While this may seem like it's unnecessarily lengthy, it simplifies things by reducing the number of pins the ESP32 uses while avoiding the problem of data getting lost or transmitted to the wrong devices. Using this connection we can easily feed the alphanumeric board with the correct information.

The data that is transmitted to the alphanumeric board is constructed as a bitmap. The board is a 14 segment LED display, and is controlled by a 16 bit bitmap, with each bit corresponding to a specific segment of the display. Furthermore, the I2C data is transmitted one byte at a time, so the bitmap had to be transmitted in 2 data packages rather than all at once.


## Sketches and Photos

![Photo of our Device](https://i.imgur.com/DHiWPsB.jpg)

[Video of Alarm Clock working](https://drive.google.com/file/d/1WlPtB_g9U6Zo2uFw5a3gDD2935-3MzL8/view?usp=sharing)





## Supporting Artifacts
- [ESP example code for servos](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control)
- [ESP example code for interrupts](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/touch_pad_interrupt)
- [ESP example code for i2c](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c)
