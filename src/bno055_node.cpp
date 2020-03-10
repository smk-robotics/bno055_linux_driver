#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/KeyValue.h> // 17.06.19. Added for IMU calibration offsets publication. Kirill.
#include <diagnostic_msgs/DiagnosticArray.h> // 10.06.19. Added for IMU calibration status publication. Kirill.
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "bno055_node.h"

using namespace std;

int main(int argc, char *argv[]) {
  // set up ROS
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle node_h("~");
  ros::Publisher imu_publisher = node_h.advertise<sensor_msgs::Imu>("/imu/data", 1);
  ros::Publisher imu_status_publisher = node_h.advertise<diagnostic_msgs::DiagnosticArray>("/imu/status", 1); // 10.06.19. Publisher for IMU calibration status. Kirill.

  // Load general parameters
  string frame_id;
  node_h.param<string>("frame_id", frame_id, "default_link");
  string bno055_i2c_name;
  node_h.param<string>("i2c_name", bno055_i2c_name, "/dev/i2c-1");
  string imu_settings_path;
  node_h.param<string>("imu_settings_path", imu_settings_path, "/home/smk/local/workspace/src/bno055_ros_driver/config/imu_settings.txt");

  int bno055_addr;
  node_h.param("i2c_adress", bno055_addr, BNO055_I2C_ADDR2);
  int rate;
  node_h.param("rate", rate, 10);
  int max_data_count;
  node_h.param("max_data_count", max_data_count, 100);

  ros::Rate loop_rate(rate); // Set IMU message publishing rate

  // IMU Сalibration settings (offsets)
  bno055_accel_offset_t accel_offset; // 16.06.19. Variable for accelerometer calibration offset. Kirill.
  bno055_mag_offset_t mag_offset;     // 16.06.19. Variable for magnetometer calibration offset. Kirill.
  bno055_gyro_offset_t gyro_offset;   // 16.06.19. Variable for gyroscope calibration offset. Kirill.

  bool offsets_available = false;     // 16.06.19. "IMU calibration offsets available" flag. Kirill.
  bool covariance_calculated = false; // 21.06.19. "IMU covariances calculated" flag. Kirill.
  uint16_t count = 0; // 21.06.19. Current data count for covariance calculation. Kirill.

  // Checking that file with IMU calibration settings is exist
  if (boost::filesystem::exists(imu_settings_path)) {
    ROS_DEBUG_STREAM("[bno055_node] - Reading IMU offsets settings from file: " << imu_settings_path);

    ifstream settings_file(imu_settings_path);

    // 21.06.19. Read IMU settings from file. Kirill.
    if (settings_file.is_open()) {
      ROS_DEBUG_STREAM("[bno055_node] - Reading IMU offsets settings ...");
      // 21.06.19. Read IMU accelerometer calibration offset from file. Kirill.
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> accel_offset;
      // 21.06.19. Read IMU magnetometer calibration offset from file. Kirill.
      settings_file.ignore(1, '\n');
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> mag_offset;
      // 21.06.19. Read IMU gyroscope calibration offset from file. Kirill.
      settings_file.ignore(1, '\n');
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> gyro_offset;
      // 24.06.19. Read IMU accelerometer covariance values from file. Kirill.
      settings_file.ignore(1, '\n');
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> accel_covariance;
      // 24.06.19. Read IMU magnetometer covariance values from file. Kirill.
      settings_file.ignore(1, '\n');
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> mag_covariance;
      // 24.06.19. Read IMU gyroscope covariance values from file. Kirill.
      settings_file.ignore(1, '\n');
      settings_file.ignore(numeric_limits<streamsize>::max(), '\n');
      settings_file >> gyro_covariance;

      offsets_available = true;  // 17.06.19. Run up the "IMU calibration offsets available" flag. Kirill.

      settings_file.close();
      ROS_DEBUG_STREAM("[bno055_node] - Reading IMU offsets settings [DONE]");
    }
  }
  else {
    ROS_WARN_STREAM("[bno055_node] - No file with IMU offset settings");
  }

  /// INIT I2C DEVICE
  BNO055_I2C_init(bno055_i2c_name.c_str(), bno055_addr);

  /// INIT DRIVER
  struct bno055_t BNO_dev; // device handle
  BNO_dev.bus_read = BNO055_I2C_bus_read; // set user defined read function
  BNO_dev.bus_write = BNO055_I2C_bus_write; // set user defined write function
  BNO_dev.delay_msec = BNO055_delay_msec; // set user defined delay function
  BNO_dev.dev_addr = bno055_addr; // set bno055 address

  // init bno055
  bno055_init(&BNO_dev); // driver's basic init routine
  bno055_set_power_mode(BNO055_POWER_MODE_NORMAL); // normal power mode

  // Write IMU offset settings if available
  if (offsets_available) {
    ROS_DEBUG_STREAM("[bno055_node] - Set IMU offsets ...");
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG); // select "config" mode for writing offsets
    bno055_write_accel_offset(&accel_offset);
    bno055_write_mag_offset(&mag_offset);
    bno055_write_gyro_offset(&gyro_offset);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); // select "9dof sensor fusion" mode for normal work
    ROS_DEBUG_STREAM("[bno055_node] - Set IMU offsets [DONE]");
  }
  else
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); // select "9dof sensor fusion" mode for normal work

  // ROS messages declaration
  sensor_msgs::Imu imu_msg; // 10.06.19. IMU data message. Kirill.
  diagnostic_msgs::DiagnosticStatus accel_status; // 10.06.19. IMU accelerometer calibration status message. Kirill.
  diagnostic_msgs::DiagnosticStatus mag_status;   // 10.06.19. IMU magnetometer calibration status message. Kirill.
  diagnostic_msgs::DiagnosticStatus gyro_status;  // 10.06.19. IMU gyroscope calibration status message. Kirill.
  diagnostic_msgs::DiagnosticArray status_msg;    // 10.06.19. IMU calibration status array message message. Kirill.

  accel_status.name = "Accelerometer calibration status";
  accel_status.hardware_id = "IMU CJMCU-055";
  mag_status.name = "Magnetometer calibration status";
  mag_status.hardware_id = "IMU CJMCU-055";
  gyro_status.name = "Gyroscope calibration status";
  gyro_status.hardware_id = "IMU CJMCU-055";

  // IMU data variables declaration
  bno055_quaternion_double_t quat;
  bno055_linear_accel_double_t lin_acc;
  bno055_gyro_double_t ang_vel;

  // IMU Сalibration status parameters
  u8 acc_calib_stat;  // 10.06.19. Variable for IMU accelerometer calibration status. Kirill.
  u8 mag_calib_stat;  // 10.06.19. Variable for IMU magnetometer calibration status. Kirill.
  u8 gyro_calib_stat; // 10.06.19. Variable for IMU gyroscope calibration status. Kirill.

  while (ros::ok()) {
    // 10.06.19. Reading IMU calibration statuses. Kirill.
    bno055_get_accel_calib_stat(&acc_calib_stat); // 10.06.19. Get accelerometer calibration status. Kirill.
    bno055_get_mag_calib_stat(&mag_calib_stat);   // 10.06.19. Get magnetometer calibration status. Kirill.
    bno055_get_gyro_calib_stat(&gyro_calib_stat); // 10.06.19. Get gyroscope calibration status. Kirill.

    // 10.06.19. Reading IMU data. Kirill.
    bno055_convert_double_quaternion_wxyz(&quat);
    bno055_convert_double_linear_accel_xyz_msq(&lin_acc);
    bno055_convert_double_gyro_xyz_rps(&ang_vel);

    // 10.06.19. Checking calibration statuses. Kirill.
    if ((int)acc_calib_stat != 3)
      ROS_WARN_THROTTLE(5, "[bno055_node] - IMU accelerometer calibration ...");

    if ((int)mag_calib_stat != 3)
      ROS_WARN_THROTTLE(5, "[bno055_node] - IMU magnetometer calibration ...");

    if ((int)gyro_calib_stat != 3)
      ROS_WARN_THROTTLE(5, "[bno055_node] - IMU gyroscope calibration ...");

    // 21.06.19. If covariance was not updated. Kirill
    if (!covariance_calculated) {
      // 21.06.19. Get statictics for covariances update. Kirill.
      ROS_WARN_THROTTLE(5, "[bno055_node] - IMU covariance updating ...");
      if (count <= max_data_count) {
        if ((int)acc_calib_stat == 3 && (int)mag_calib_stat == 3 && (int)gyro_calib_stat == 3 &&
                (abs(ang_vel.x) < 0.01) && (abs(ang_vel.y) < 0.01) && (abs(ang_vel.z) < 0.01) &&
                    (abs(lin_acc.x) < 0.1) && (abs(lin_acc.y) < 0.1) && (abs(lin_acc.z) < 0.1)) {
          stat_quat_x.push_back(quat.x);
          stat_quat_y.push_back(quat.y);
          stat_quat_z.push_back(quat.z);

          stat_acc_x.push_back(lin_acc.x);
          stat_acc_y.push_back(lin_acc.y);
          stat_acc_z.push_back(lin_acc.z);

          stat_ang_x.push_back(ang_vel.x);
          stat_ang_y.push_back(ang_vel.y);
          stat_ang_z.push_back(ang_vel.z);
          count++;
        }
      }
      else {
        ROS_INFO_ONCE("[bno055_node] - IMU calibration [DONE]");
        // 21.06.19. Calculating covariances. Kirill.
        mag_covariance.x = calcStats(stat_quat_x);
        mag_covariance.y = calcStats(stat_quat_y);
        mag_covariance.z = calcStats(stat_quat_z);

        accel_covariance.x = calcStats(stat_acc_x);
        accel_covariance.y = calcStats(stat_acc_y);
        accel_covariance.z = calcStats(stat_acc_z);

        gyro_covariance.x = calcStats(stat_ang_x);
        gyro_covariance.y = calcStats(stat_ang_y);
        gyro_covariance.z = calcStats(stat_ang_z);
        covariance_calculated = true; // 21.06.19. Run up the "IMU covariances calculated" flag. Kirill.
        ROS_INFO_ONCE("[bno055_node] - IMU covariance updating [DONE]");
      }
    }

    status_msg.status.clear(); // 10.06.19. Clear status array message. Kirill.

    // 10.06.19. Accelerometer calibration status message population. Kirill.
    accel_status.message = to_string((int)acc_calib_stat); // 0 - Not calibrated, 3 - Fully calibrated.
    status_msg.status.push_back(accel_status); // 10.06.19. Add status message to status array. Kirill.
    // 10.06.19. Magnetometer calibration status message population. Kirill.
    mag_status.message = to_string((int)mag_calib_stat); // 0 - Not calibrated, 3 - Fully calibrated.
    status_msg.status.push_back(mag_status); // 10.06.19. Add status message to status array. Kirill.
    // 10.06.19. Gyroscope calibration status message population. Kirill.
    gyro_status.message = to_string((int)gyro_calib_stat); // 0 - Not calibrated, 3 - Fully calibrated.
    status_msg.status.push_back(gyro_status); // 10.06.19. Add status message to status array. Kirill.

    // Padding IMU message with accelerometer, magnetometer and gyroscope data
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_id;
    imu_msg.orientation.w = quat.w;
    imu_msg.orientation.x = quat.x;
    imu_msg.orientation.y = quat.y;
    imu_msg.orientation.z = quat.z;
    imu_msg.linear_acceleration.x = lin_acc.x;
    imu_msg.linear_acceleration.y = lin_acc.y;
    imu_msg.linear_acceleration.z = lin_acc.z;
    imu_msg.angular_velocity.x = ang_vel.x;
    imu_msg.angular_velocity.y = ang_vel.y;
    imu_msg.angular_velocity.z = ang_vel.z;

    // 21.06.19. Padding IMU message with covariance matrices. Kirill.
    imu_msg.orientation_covariance[0] = mag_covariance.x;
    imu_msg.orientation_covariance[4] = mag_covariance.y;
    imu_msg.orientation_covariance[8] = mag_covariance.z;
    imu_msg.linear_acceleration_covariance[0] = accel_covariance.x;
    imu_msg.linear_acceleration_covariance[4] = accel_covariance.y;
    imu_msg.linear_acceleration_covariance[8] = accel_covariance.z;
    imu_msg.angular_velocity_covariance[0] = gyro_covariance.x;
    imu_msg.angular_velocity_covariance[4] = gyro_covariance.y;
    imu_msg.angular_velocity_covariance[8] = gyro_covariance.z;

//    if ((int)acc_calib_stat == 3 && (int)mag_calib_stat == 3 && (int)gyro_calib_stat == 3) {
//      stat_quat_x.push_back(quat.x);
//      stat_quat_y.push_back(quat.y);
//      stat_quat_z.push_back(quat.z);

//      stat_acc_x.push_back(lin_acc.x);
//      stat_acc_y.push_back(lin_acc.y);
//      stat_acc_z.push_back(lin_acc.z);

//      stat_ang_x.push_back(ang_vel.x);
//      stat_ang_y.push_back(ang_vel.y);
//      stat_ang_z.push_back(ang_vel.z);

//      count++;

//      if (count > max_data_count) {
//        stat_quat_x.pop_front();
//        stat_quat_y.pop_front();
//        stat_quat_z.pop_front();

//        stat_acc_x.pop_front();
//        stat_acc_x.pop_front();
//        stat_acc_x.pop_front();

//        stat_ang_x.pop_front();
//        stat_ang_y.pop_front();
//        stat_ang_z.pop_front();

//        // 21.06.19. Calculating covariances. Kirill.
//        mag_covariance.x = calcStats(stat_quat_x);
//        mag_covariance.y = calcStats(stat_quat_y);
//        mag_covariance.z = calcStats(stat_quat_z);

//        accel_covariance.x = calcStats(stat_acc_x);
//        accel_covariance.y = calcStats(stat_acc_y);
//        accel_covariance.z = calcStats(stat_acc_z);

//        gyro_covariance.x = calcStats(stat_ang_x);
//        gyro_covariance.y = calcStats(stat_ang_y);
//        gyro_covariance.z = calcStats(stat_ang_z);

//        cout << "[bno055_node] - count == " << count << endl; // Diagnostic output

//        count--;
//      }
//    }

    imu_status_publisher.publish(status_msg); // 10.06.19. IMU calibration status publication. Kirill.
    imu_publisher.publish(imu_msg);

    loop_rate.sleep();
  }

  // 13.06.19. Store calibration data if IMU fully calibrated. Kirill.
  if ((int)acc_calib_stat == 3 && (int)mag_calib_stat == 3 && (int)gyro_calib_stat == 3) {
    // cout << "[bno055_node] - Saving IMU offset settings ..." << endl; // Diagnostic output
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG); // select "config" mode for reading offsets
    bno055_read_accel_offset(&accel_offset); // 17.06.19. Read current accelerometer offset. Kirill.
    bno055_read_mag_offset(&mag_offset);     // 17.06.19. Read current magnetometer offset. Kirill.
    bno055_read_gyro_offset(&gyro_offset);   // 17.06.19. Read current gyroscope offset. Kirill.
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); // select "9dof sensor fusion" mode for normal work

    ofstream settings_file(imu_settings_path, ios::out | ios::trunc);

    if (settings_file.is_open()) {
      // 17.06.19. Write current accelerometer offset to IMU settings file. Kirill.
      settings_file << "# Accelerometer calibration settings (x y z r) \n";
      settings_file << accel_offset << "\n";
      // 17.06.19. Write current magnetometer offset to IMU settings file. Kirill.
      settings_file << "# Magnetometer calibration settings (x y z r) \n";
      settings_file << mag_offset << "\n";
      // 17.06.19. Write current gyroscope offset to IMU settings file. Kirill.
      settings_file << "# Gyroscope calibration settings (x y z) \n";
      settings_file << gyro_offset << "\n";
      // 24.06.19. Write current accelerometer covariance to IMU settings file. Kirill.
      settings_file << "# Accelerometer covariance (x y z) \n";
      settings_file << accel_covariance << "\n";
      // 24.06.19. Write current magnetometer covariance to IMU settings file. Kirill.
      settings_file << "# Magnetometer covariance (x y z) \n";
      settings_file << mag_covariance << "\n";
      // 24.06.19. Write current gyroscope covariance to IMU settings file. Kirill.
      settings_file << "# Gyroscope covariance (x y z) \n";
      settings_file << gyro_covariance << "\n";

      settings_file.close();
      // cout << "[bno055_node] - Saving IMU offset settings [DONE]" << endl; // Diagnostic output
    }
  }

  close(bno055_file);

  return 0;
}

// Covariance calculation function
float calcStats(vector<float>& data)
{
  float sum = 0.0;
  float mean = 0.0;
  float sum_sq = 0.0;
  size_t count = data.size();

//  for (list<float>::iterator it = data.begin(); it != data.end(); ++it)
//  {
//     sum += it;
//  }

  for (size_t i = 0; i < count; i++) {
      sum += data[i];
  }

  mean = sum/count;

//  for (list<float>::iterator it = data.begin(); it != data.end(); ++it)
//  {
//     sum += (it-mean)*(it-mean) ;
//  }

  for (size_t i = 0; i < count; i++) {
      sum_sq += (data[i]-mean)*(data[i]-mean);
  }

  float sigma_sq = sum_sq/count;

  if (sigma_sq == 0.0)
    sigma_sq = 1e-12;

  return sigma_sq;
}
