# Proximity-Sensor
Designed for a vision impaired person. Based on an Arduino Nano, the unit uses a low cost ultrasonic distance measuring unit to assess range.

The initial requirement was for an aid that could be used by an individual in an age care facility, who was vision impaired and used a walker to aid them with their limited mobility. It was desired that the user be provided with some form of indication of the proximity of walls and objects that might be in their path.

Because the environment was an age care facility it was not permissible to have audio indications of proximity and so it was decided to use haptic feedback by the form of a vibrating motor coupled to the walking frame. The motor would be vibrated at varying levels to indicate proximity. The power to the motor is switched on and off in pulses and the pulses are progressively longer as the sensed distance to objects decreases. i.e. short duration bursts for longer ranges and almost on fully when the distance are short.

There was also a requirement that the use of the device should be 'automatic' and that there not be an on/off switch and that the unit not require daily charging (i.e. low maintenance).

The unit uses the Arduino sleep mode to enter a low power state when there is no activity. Wake up from ‘sleep’ is determined by a timer interrupt. Upon ‘waking’ the unit takes some samples of distance and if it has not changed since it was last placed in sleep, then it assumed that the unit is stationery and it goes back to sleep. If however the sampled distance has changed then the unit is mobile and it continues to take samples and provide feedback to the user of proximity to objects.

In this way, even if the unit is parked up against a wall, it will go into the low power sleep mode and will wake up when the unit is moved away from the wall.

In order to provide more reliable sensing of proximity the samples are summed and averaged and new samples are compared to the running average. This smooths out the jitter seen in the values returned by the ultrasonic sensor and when the difference between samples and the running average fall below a threshold then the unit is assumed to be stationary and can be placed in sleep mode. Values returned from the sensor are only considered if they fall with a range of maximum and minimum values that are considered valid for analysis.

The user environment is such that the speed of movement is relatively slow and so these smoothing and calculating periods will not impact the usefulness of the aid.

