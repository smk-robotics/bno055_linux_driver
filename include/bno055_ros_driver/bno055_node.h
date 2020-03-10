#ifndef BNO055_NODE_H
#define BNO055_NODE_H

#include "bno055_driver.h"

using namespace std;

/// <<< Structs
// Struct for covariance matrix
typedef struct _covariance
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
} Covariance;
/// >>> Structs

/// <<< Variables
Covariance accel_covariance; // 24.06.19. Covariance for IMU acceleration data. Kirill.
Covariance mag_covariance;   // 24.06.19. Covariance for IMU magnetometer data. Kirill.
Covariance gyro_covariance;  // 24.06.19. Covariance for IMU gyroscope data. Kirill.

// Vectors with statistics for coveriance matrices calculation
vector<float> stat_quat_w;
vector<float> stat_quat_x;
vector<float> stat_quat_y;
vector<float> stat_quat_z;

vector<float> stat_acc_x;
vector<float> stat_acc_y;
vector<float> stat_acc_z;

vector<float> stat_ang_x;
vector<float> stat_ang_y;
vector<float> stat_ang_z;

// Covariance matrix (quat.x, quat.y, quat.z, acc.x, acc.y, acc.z, ang.x, ang.y, ang.z)
vector<float> covariance_matrix = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
/// >>> Variables

/// <<< Functions
// Function to calc covariance for value from vector with statistic
float calcStats(vector<float>& data);
/// >>> Functions

/// <<< Standart functions overloading
// Input function overload for ACCELERATION offset
istream& operator>>(istream& input, bno055_accel_offset_t& accel_offset) {
  input >> accel_offset.x >> accel_offset.y >> accel_offset.z >> accel_offset.r;
  return input;
}

// Output function overload for ACCELERATION offset
ostream& operator<<(ostream& output, bno055_accel_offset_t& accel_offset){
  output << accel_offset.x << "\t" << accel_offset.y << "\t" << accel_offset.z << "\t" << accel_offset.r;
  return output;
}

// Input function overload for MAGNETOMETER offset
istream& operator>>(istream& input, bno055_mag_offset_t& mag_offset) {
  input >> mag_offset.x >> mag_offset.y >> mag_offset.z >> mag_offset.r;
  return input;
}

// Output function overload for ACCELERATION offset
ostream& operator<<(ostream& output, bno055_mag_offset_t& mag_offset) {
  output << mag_offset.x << "\t" << mag_offset.y << "\t" << mag_offset.z << "\t" << mag_offset.r;
  return output;
}

// Input function overload for GYROSCOPE offset
istream& operator>>(istream& input, bno055_gyro_offset_t& gyro_offset) {
  input >> gyro_offset.x >> gyro_offset.y >> gyro_offset.z;
  return input;
}

// Output function overload for GYROSCOPE offset
ostream& operator<<(ostream& output, bno055_gyro_offset_t& gyro_offset) {
  output << gyro_offset.x << "\t" << gyro_offset.y << "\t" << gyro_offset.z;
  return output;
}

// Input function overload for COVARIANCE value
istream& operator>>(istream& input, Covariance& covar) {
  input >> covar.x >> covar.y >> covar.z;
  return input;
}

// Output function overload for COVARIANCE value
ostream& operator<<(ostream& output, Covariance& covar) {
  output << covar.x << "\t" << covar.y << "\t" << covar.z;
  return output;
}
/// >>> Standart functions overloading

#endif // BNO055_NODE_H
