#  DAWS Motor Control Library

---

This library provides a brushed DC PWM traction motor controller. It uses a
TI DRV8871 H bridge. With:

- PI (proportional-integral) control using bemf derived speed error,
- BEMF sampling during gap in pwm pulse train,
- BEMF filtering using a statistical median digital filter and,
- a simple Infinite Impulse Response (IIR) low pass digital filter



The PWM controller uses PWM peripheral on the NORDIC Semiconductor nRF52840 to generate PWM.

A differential amplifier is used to measure the BEMF.  The SAADC peripheral is used to digitise the analogue
DA output.
