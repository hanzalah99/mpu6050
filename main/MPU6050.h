#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B	
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	
#define	MPU6050_ADDR	0x68

void MPU6050_init();
int get_accX();
int get_accY();
int get_accZ();
int get_gyro_X();
int get_gyro_Y();
int get_gyro_Z();

void i2c_init();
void slave_write(uint8_t slave_add, uint8_t reg_add, uint8_t data);
uint8_t slave_read(uint8_t slave_add, uint8_t reg_add);
