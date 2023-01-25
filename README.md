## Bill Ball

This is a robotics project that controls a body balanced on top of a sphere (ballbot or inverted pendulum). The project is written in C using the ESP-IDF framework to run on an ESP32. It uses a BNO-055 9-axis orientation sensor to determine its corrections in 3 axes, and it uses encoders on its motors to return to the position the robot is enabled at after it is disturbed. 

The BNO-055 integration was made possible with a "driver" file from Bosch and some example code they provided. There is also a partial implementation of quaternion-based math in my code (src/bno055/bno_helper.c) but I have the functionality I need so far from euler angles so the functions are unused.