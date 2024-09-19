/*complementary filter high passes filtered gyro data bc it will drift over time - goof for short term
low passes accelerometer data - less instantaneous response - good long term, low pass magnetometer and swap with accelerometer for z
balance these to give a reliable estimate

x, y: high pass gyro low pass accelerometer
z: high pass gyro low pass magnetometer
*/

extern void complementary_filter(float accel[3], float gyro[3], float mag[3], float angular_vel[3]);