## Dynamic engineering is a group dedicated to make autonomous flight drones for multiple uses. This project is made to be submitted to Hackomation 2022.

### pins:

#### MCPWM BLDC
- 2   front
- 4   right
- 9   back
- 10  left

#### HC-SR04
- 32  ECHO
- 33  TRIG

#### I2C
- 18  SDA
- 19  SCL

#### GPS
- 36

#### sim800L
- 34  TX
- 35  RX

#### HC-12 RF module
- TX  27
- RX  35

### commands
- 0-(kill value)              0 -> run motors, 1-> kill motors
- 1-(throttle in pwm_us)      MIN->1000, MAX -> 1800
