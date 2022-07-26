#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"
static gpio_num_t i2c_gpio_sda = 1;

void MPU6050_init ()
{
    i2c_init();
    slave_write(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    slave_write(MPU6050_ADDR, SMPLRT_DIV, 0x07);    //0x07 to the sampling frequency division register
    slave_write(MPU6050_ADDR, CONFIG, 0x07);
    slave_write(MPU6050_ADDR, GYRO_CONFIG, 0x18);
    slave_write(MPU6050_ADDR, ACCEL_CONFIG, 0x01);
}
 
int get_accX()
{
    uint16_t acc_XH, acc_XL, acc_X;

    acc_XH = slave_read(MPU6050_ADDR, ACCEL_XOUT_H);
    acc_XL = slave_read(MPU6050_ADDR, ACCEL_XOUT_L);
    acc_X = (acc_XH << 8) + acc_XL;
    return (int)acc_X;
}

int get_accY()
{
    uint16_t acc_YH, acc_YL, acc_Y;

    acc_YH = slave_read(MPU6050_ADDR, ACCEL_YOUT_H);
    acc_YL = slave_read(MPU6050_ADDR, ACCEL_YOUT_L);
    acc_Y = (acc_YH << 8) + acc_YL;
    return (int)acc_Y;
    
}

int get_accZ()
{
    uint16_t acc_ZH, acc_ZL, acc_Z;

    acc_ZH = slave_read(MPU6050_ADDR, ACCEL_ZOUT_H);
    acc_ZL = slave_read(MPU6050_ADDR, ACCEL_ZOUT_L);
    acc_Z = (acc_ZH << 8) + acc_ZL;
    return (int)acc_Z;
    
}

int get_gyro_X()
{
    uint16_t gyro_XH, gyro_XL, gyro_X;

    gyro_XH = slave_read(MPU6050_ADDR, GYRO_XOUT_H);
    gyro_XL = slave_read(MPU6050_ADDR, GYRO_XOUT_L);
    gyro_X = (gyro_XH << 8) + gyro_XL;
    return (int)gyro_X;
}

int get_gyro_Y()
{
    uint16_t gyro_YH, gyro_YL, gyro_Y;

    gyro_YH = slave_read(MPU6050_ADDR, GYRO_YOUT_H);
    gyro_YL = slave_read(MPU6050_ADDR, GYRO_YOUT_L);
    gyro_Y = (gyro_YH << 8) + gyro_YL;
    return (int)gyro_Y;
}

int get_gyro_Z()
{
    uint16_t gyro_ZH, gyro_ZL, gyro_Z;

    gyro_ZH = slave_read(MPU6050_ADDR, GYRO_ZOUT_H);
    gyro_ZL = slave_read(MPU6050_ADDR, GYRO_ZOUT_L);
    gyro_Z = (gyro_ZH << 8) + gyro_ZL;
    return (int)gyro_Z;
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
    i2c_master_cmd_begin(I2C_NUM_0,write,100);
    i2c_cmd_link_delete(write);
}

uint8_t slave_read(uint8_t slave_add, uint8_t reg_add)
{
    uint8_t buf;
    i2c_cmd_handle_t read = i2c_cmd_link_create();
    i2c_master_start(read);
    i2c_master_write_byte(read, (slave_add << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(read, reg_add, 1);
    i2c_master_write_byte(read, (slave_add << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(read, &buf, 1);
    i2c_master_stop(read);
    i2c_master_cmd_begin(I2C_NUM_0,read,100);
    i2c_cmd_link_delete(read);
    return buf;
}