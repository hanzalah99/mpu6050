#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"

void app_main(void)
{
    i2c_init();
    MPU6050_init();
    float ax, ay, az;
    float gx, gy, gz;
    double roll, pitch,yaw;
    double val = M_PI / 180;
    double qx, qy, qz, qw;

    // get Accelaration raw data
    ax = get_accX();
    ay = get_accY();
    az = get_accZ();
    // get Gyroscope raw data
    gx = get_gyro_X();
    gy = get_gyro_Y();
    gz = get_gyro_Z();

    // roll, pitch and yaw are the euler angles
    roll = 180 * atan (ay/sqrt(ax*ax + az*az))/M_PI;
    pitch = 180 * atan (ax/sqrt(ay*ay + az*az))/M_PI;
    yaw = 180 * atan (az/sqrt(ax*ax + az*az))/M_PI;

    // Converting the euler angles to radians
    roll=roll*val;
    pitch= pitch*val;
    yaw= yaw*val;

    // calculating the angles of quaterinons from the euler angles
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    }