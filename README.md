#  DAWS Motor Control Library

---

This library provides a brushed DC PWM traction motor controller for use with an Arduino Nano 33 BLE.

It uses a TI DRV8871 H bridge for motor control  with:

- PI (proportional-integral) control using BEMF derived speed error,
- BEMF sampling during gap in pwm pulse train,
- BEMF filtering using a statistical median digital filter and,
- a simple Infinite Impulse Response (IIR) low pass digital filter



The PWM controller uses PWM peripheral on the NORDIC Semiconductor nRF52840 to generate PWM.

A differential amplifier is used to measure the BEMF.
The SAADC peripheral is used to digitise the analogue
Diff. Amp. output.

---

This library requires the "Arduino Mbed OS Nano Boards" option to be installed via the Arduino IDE 
Boards Manager and subsequent selection of the "Arduino Nano 33 BLE".

The library uses Mbed/rtos functions for thread control, scheduling etc.  The nRF52840 PWM and
SAADC peripherals are accessed directly.
