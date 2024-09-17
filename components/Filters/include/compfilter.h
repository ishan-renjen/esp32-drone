/*complementary filter high passes filtered gyro data bc it will drift over time - goof for short term
low passes accelerometer data - less instantaneous response - good long term, low pass magnetometer and swap with accelerometer for yaw
balance these to give a reliable estimate

pitch, roll: high pass gyro low pass accelerometer
yaw: high pass gyro low pass magnetometer
*/

extern void complementary_filter(float accel[3], float gyro[3], float mag[3], float angular_vel[3]);