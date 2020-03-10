# bno055_ros_driver

Modified ROS driver for BNO055 (CJMCU-055)

Based on bno055_linux_driver by skbmir https://github.com/skbmir/bno055_linux_driver

**IMU Calibration**

Though the sensor fusion software runs the calibration algorithm of all the three sensors (accelerometer, gyroscope and magnetometer) in the background to remove the offsets, some preliminary steps had to be ensured for this automatic calibration to take place. The accelerometer and the gyroscope are relatively less susceptible to external disturbances, as a result of which the offset is negligible. Whereas the magnetometer is susceptible to external magnetic field and therefore to ensure proper heading accuracy, the calibration steps described below have to be taken. Depending on the sensors been selected, the following simple steps had to be taken after every ‘Power on Reset’ for proper calibration of the device.

* **Accelerometer Calibration**
Place the device in 6 different stable positions for a period of few seconds to allow the accelerometer to calibrate. Make sure that there is slow movement between 2 stable positions. The 6 stable positions could be in any direction, but make sure that the device is lying at least once perpendicular to the x, y and z axis. The register CALIB_STAT can be read to see the calibration status of the accelerometer.

* **Gyroscope Calibration** 
Place the device in a single stable position for a period of few seconds to allow the gyroscope to calibrate. The register CALIB_STAT can be read to see the calibration status of the gyroscope.

* **Magnetometer Calibration**
Magnetometer in general are susceptible to both hard-iron and soft-iron distortions, but majority of the cases are rather due to the former. And the steps mentioned below are to calibrate the magnetometer for hard-iron distortions.
Nevertheless certain precautions need to be taken into account during the positioning of the sensor in the PCB which is described in our HSMI (Handling, Soldering and Mounting Instructions) application note to avoid unnecessary magnetic influences.
<br />***Compass, M4G & NDOF_FMC_OFF:***
<br />Make some random movements (for example: writing the number ‘8’ on air) until the CALIB_STAT register indicates fully calibrated.
It takes more calibration movements to get the magnetometer calibrated than in the NDOF mode. 
<br />***NDOF:*** 
<br />The same random movements have to be made to calibrate the sensor as in the FMC_OFF mode, but here it takes relatively less calibration movements (and slightly higher current consumption) to get the magnetometer calibrated. The register CALIB_STAT can be read to see the calibration status of the magnetometer.***
