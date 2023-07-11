# panTiltRobot
This repository of arduino C was used to power the pan tilt robot shown below.
This was a Mechatronics Final Project for my Masters Degree at Colorado School of Mines.
The robot was a proof-of-concept for high speed object tracking.
When it was functional, the robot took accelerometer and gyroscope input to keep the image vector pointing at its original target. All the vector transformation math is included in the repository code, but it should be noted that true image tracking is not included. This means you will have to write your image tracking code if you want to use the camera as an input sensor.
Read the [Final Paper.pdf](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/Final%20Paper.pdf) for detailed info.

If you want to use this code for your arduino project, you will have to download arduino libraries in addition to the code in this repository. You will need to install the [servoOrientation2.ino](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/servoOrientation2/servoOrientation2.ino) file onto your arduino board. Addtitionally you will need to source and build the physical components shown in the [BOM](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/BOM.png) of the final paper.

![alt text](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/Robot%20Render%20and%20Component%20Label.png)

![alt text](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/Wiring%20Diagram.png)

![alt text](https://github.com/RamonJustisOrtega/panTiltRobot/blob/main/BOM.png)
