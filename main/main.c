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
    float roll, pitch,yaw;
    float roll_r, pitch_r, yaw_r;
    float val = M_PI / 180;
    float qx, qy, qz, qw;
    float *array;

    while(1)
    {
    // get Accelaration raw data
    ax = get_accX();
    ay = get_accY();
    az = get_accZ();
    printf("ax : %f ", ax);
    printf("ay : %f ", ay);
    printf("az : %f ", az);

    // get Gyroscope raw data
    gx = get_gyro_X();
    gy = get_gyro_Y();
    gz = get_gyro_Z();
    printf("gx : %f \n", gx);
    printf("gy : %f \n", gy);
    printf("gz : %f \n", gz);

    // roll, pitch and yaw are the euler angles
    roll = roll_func(ax, ay, az);
    pitch = pitch_func(ax, ay, az);
    yaw = yaw_func(ax, ay, az);
    printf("roll : %f \n", roll);
    printf("pitch : %f \n", pitch);
    printf("yaw : %f \n", yaw);

    // Converting the euler angles to radians
    roll_r=roll*val;
    pitch_r= pitch*val;
    yaw_r= yaw*val;

    // calculating the angles of quaterinons from the euler angles
    array = quaternions(roll_r, pitch_r, yaw_r);

        for (int i = 0; i < 4;i++)
        {
            printf("q[%d] : %f \n",i, array[i]);
        }
    }
    }