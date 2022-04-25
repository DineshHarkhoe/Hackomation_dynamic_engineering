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

#### HC-12 RF
- TX  27
- RX  35

### commands
- 0-(kill value)              0 -> run motors, 1-> kill motors
- 1-(throttle in pwm_us)      MIN->1000, MAX -> 1800
- 10-(kp)                     p value pitch
- 11-(ki)                     i value pitch
- 12-(kd)                     d value pitch
- 13-(kp)                     p value roll
- 14-(kp)                     i value roll
- 15-(kp)                     d value roll
- 16-(kp)                     p value yaw
- 17-(kp)                     i value yaw
- 18-(kp)                     d value yaw
- 19-(kp)                     p value altitude
- 20-(kp)                     i value altitude
- 21-(kp)                     d value altitude
