#define	SMPLRT_DIV		0x19	//Addr of SMPLRT_DIV Register
#define	CONFIG			0x1A    //Addr of CONFIG Register
#define	GYRO_CONFIG		0x1B	//Addr of GYRO_CONFIG Register
#define	ACCEL_CONFIG	0x1C	//Addr of ACCEL_CONFIG Register
#define	ACCEL_XOUT_H	0x3B	//Addr of ACCEL_XOUT_H Register
#define	ACCEL_XOUT_L	0x3C    //Addr of ACCEL_XOUT_L Register
#define	ACCEL_YOUT_H	0x3D    //Addr of ACCEL_YOUT_H Register
#define	ACCEL_YOUT_L	0x3E    //Addr of ACCEL_YOUT_L Register
#define	ACCEL_ZOUT_H	0x3F    //Addr of ACCEL_ZOUT_H Register
#define	ACCEL_ZOUT_L	0x40    //Addr of ACCEL_ZOUT_L Register
#define	GYRO_XOUT_H		0x43    //Addr of GYRO_XOUT_H Register
#define	GYRO_XOUT_L		0x44	//Addr of GYRO_XOUT_L Register
#define	GYRO_YOUT_H		0x45    //Addr of GYRO_YOUT_H Register
#define	GYRO_YOUT_L		0x46    //Addr of GYRO_YOUT_L Register
#define	GYRO_ZOUT_H		0x47    //Addr of GYRO_ZOUT_H Register
#define	GYRO_ZOUT_L		0x48    //Addr of GYRO_ZOUT_L Register
#define	PWR_MGMT_1		0x6B	//Addr of PWR_MGMT_1 Register
#define	MPU6050_ADDR	0x68    //Addr of MPU6050_ADDR Register

#define AccAxis_Sensitive 16384
#define GyroAxis_Sensitive (float)16.4

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
