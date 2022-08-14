#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"

void MPU6050_init ()
{
    slave_write(MPU6050_ADDR, PWR_MGMT_1, 0x00);     // Clear sleep mode bit (6), enable all sensors 
    slave_write(MPU6050_ADDR, SMPLRT_DIV, 0x00);    // Set the sampling rate to 1KHz
    slave_write(MPU6050_ADDR, CONFIG, 0x01);        // Set the Gyro Fs to 1KHz
    slave_write(MPU6050_ADDR, GYRO_CONFIG, 0x18);   // Set full scale range for the gyro 2000
    slave_write(MPU6050_ADDR, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2g
 
}
 
float get_accX()
{
     uint8_t r[0];
    slave_read(MPU6050_ADDR, ACCEL_XOUT_H, r, 2);
    short accx = r[0] << 8 | r[1];                     // left shift acc X H by 8 bits and adding the gyro Z L
    return (float)accx / AccAxis_Sensitive;
}

float get_accY()
{
    uint16_t acc_XH, acc_XL, acc_X;

    uint8_t r[0];
    slave_read(MPU6050_ADDR, ACCEL_YOUT_H, r, 2);
    short accy = r[0] << 8 | r[1];                     // left shift acc Y H by 8 bits and adding the gyro Z L
    return (float)accy / AccAxis_Sensitive;
    
}

float get_accZ()
{
    uint8_t r[0];
    slave_read(MPU6050_ADDR, ACCEL_ZOUT_H, r, 2);
    short accz = r[0] << 8 | r[1];                     // left shift acc Z H by 8 bits and adding the gyro Z L
    return (float)accz / AccAxis_Sensitive;
    
}

float get_gyro_X()
{

    uint8_t r[0];
    slave_read(MPU6050_ADDR, GYRO_XOUT_H, r, 2);
    short gyrox = r[0] << 8 | r[1];                     // left shift gyro X H by 8 bits and adding the gyro Z L
    return (float)gyrox / GyroAxis_Sensitive;
    uint16_t gyro_XH, gyro_XL, gyro_X;

    
}

float get_gyro_Y()
{

    uint8_t r[0];
    slave_read(MPU6050_ADDR, GYRO_YOUT_H, r, 2);
    short gyroy = r[0] << 8 | r[1];                     // left shift gyro X H by 8 bits and adding the gyro Z L
    return (float)gyroy / GyroAxis_Sensitive;
    uint16_t gyro_YH, gyro_YL, gyro_Y;

    
}

float get_gyro_Z()
{

    uint8_t r[0];
    slave_read(MPU6050_ADDR, GYRO_ZOUT_H, r, 2);
    short gyroz = r[0] << 8 | r[1];                     // left shift gyro Z H by 8 bits and adding the gyro Z L
    return (float)gyroz / GyroAxis_Sensitive;
    uint16_t gyro_ZH, gyro_ZL, gyro_Z;
}

float roll_func(float ax, float ay, float az)
{
    float calc_roll = 180 * atan (ay/sqrt(ax*ax + az*az))/M_PI;
    return calc_roll;
}

float pitch_func(float ax, float ay, float az)
{
    float calc_pitch = 180 * atan (ax/sqrt(ay*ay + az*az))/M_PI;
    return calc_pitch;
}

float yaw_func(float ax, float ay, float az)
{
    float calc_yaw = 180 * atan (az/sqrt(ax*ax+ az*az))/M_PI;
    return calc_yaw;
}

float *quaternions(float roll, float pitch, float yaw)
{
    static float q[4];
    q[0] = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q[1] = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q[2] = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q[3] = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
}

void i2c_init()
{
int i2c_master_port = I2C_NUM_0;
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,               // select the SDA Pin
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = GPIO_NUM_22,                  // select the CLK Pin
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 1000000,
};
    i2c_param_config(I2C_NUM_0, &conf );
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

}


void slave_write(uint8_t slave_add, uint8_t reg_add, uint8_t data)                      // write to slave
{
    i2c_cmd_handle_t write = i2c_cmd_link_create();
    i2c_master_start(write);
    i2c_master_write_byte(write,slave_add<<1| I2C_MASTER_WRITE,1);
    i2c_master_write_byte(write, reg_add, 1);
    i2c_master_write_byte(write, data, 1);
    i2c_master_stop(write);
    i2c_master_cmd_begin(I2C_NUM_0,write,10);
    i2c_cmd_link_delete(write);
}

void slave_read(uint8_t slave_addr, uint8_t data, uint8_t *buf, uint32_t len) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    i2c_cmd_link_delete(cmd);


    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | 1, 1);
    while(len) {
        i2c_master_read_byte(cmd, buf, (len == 1));
        buf++;
        len--;
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    i2c_cmd_link_delete(cmd);
}

uint8_t slave_read_byte(uint8_t slave_addr, uint8_t reg) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    i2c_cmd_link_delete(cmd);

    uint8_t buf;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | 1, 1);
    i2c_master_read_byte(cmd, &buf, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return buf;
}