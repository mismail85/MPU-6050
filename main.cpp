/*
 * main.cpp
 *
 *  Created on: Apr 20, 2018
 *      Author: mismail
 */
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#define MPU6050_SLAVE_ADDRESS 0x68
#define MPU6050_POWER_REG 0x6B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACC_X_HIGH          0x3B
#define MPU6050_REG_GYRO_X_HIGH         0x43

#define ACC_FS_SENSITIVITY_0					16384
#define ACC_FS_SENSITIVITY_1		            8192
#define ACC_FS_SENSITIVITY_2		            4096
#define ACC_FS_SENSITIVITY_3		            2048

#define GYR_FS_SENSITIVITY_0					 131
#define GYR_FS_SENSITIVITY_1					 65.5
#define GYR_FS_SENSITIVITY_2					 32.8
#define GYR_FS_SENSITIVITY_3				 	 16.4

#define GYRO_SENSITIVITY GYR_FS_SENSITIVITY_0

#define I2C_DEVICE_FILE "/dev/i2c-2"

int i2c_file_device;

int mpu6050_write(uint8_t address, uint8_t data)
{
	int ret;
	char buf[2];
	buf[0] = address;
	buf[1] = data;
	ret = write(i2c_file_device, buf, 2);
	return ret;
}

int mpu6050_read(uint8_t base_addr, char *pBuffer, uint32_t len)
{
	int ret;
	char buf[2];
	buf[0] = base_addr;
	ret = write(i2c_file_device, buf, 1);
	if(ret <= 0)
	{
		perror("write address failed\n");
		return -1;
	}

	ret = read(i2c_file_device, pBuffer,len);
	if(ret <= 0)
	{
		perror("read failed\n");
	}
	return 0;
}

void mpu6050_init()
{
	mpu6050_write(MPU6050_POWER_REG, 0x00);
	usleep(500);

	mpu6050_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
	usleep(500);

	mpu6050_write(MPU6050_REG_GYRO_CONFIG, 0x00);
	usleep(500);
}

void mpu6050_read_acc(short int * pBuffer)
{
	char acc_buffer[6];

	mpu6050_read(MPU6050_REG_ACC_X_HIGH, acc_buffer, 6);

	pBuffer[0] = (int) ( (acc_buffer[0] << 8) |  acc_buffer[1] );
	pBuffer[1] = (int) ( (acc_buffer[2] << 8) |  acc_buffer[3] );
	pBuffer[2] = (int) ( (acc_buffer[4] << 8) |  acc_buffer[5] );

}

void mpu6050_read_gyro(short *pBuffer)
{
    char gyro_buffer[6];

    //start reading from the base address of gyro values i.e MPU6050_REG_GYRO_X_HIGH
    mpu6050_read(MPU6050_REG_GYRO_X_HIGH, gyro_buffer,6);

    pBuffer[0] =  ( (gyro_buffer[0] << 8) +  gyro_buffer[1] );
    pBuffer[1] =  ( (gyro_buffer[2] << 8) +  gyro_buffer[3] );
    pBuffer[2] =  ( (gyro_buffer[4] << 8) +  gyro_buffer[5] );

}

int main()
{
	short gyro_value[3];
	short acc_value[3];

    double gyrox,gyroy,gyroz;
    double accx, accy, accz;

	if ((i2c_file_device = open(I2C_DEVICE_FILE, O_RDWR)) < 0)
	{
		perror("Failed to open I2C device File. \n");
		return -1;
	}

	if ( ioctl(i2c_file_device, I2C_SLAVE, MPU6050_SLAVE_ADDRESS ) < 0)
	{
		perror("Failed to set MPU6050 device address \n");
		close( i2c_file_device );
		return -1;
	}

	mpu6050_init();

	while(1)
	{
		mpu6050_read_acc(acc_value);
		mpu6050_read_gyro(gyro_value);

		/* Convert gyro raw values in to  "°/s" (deg/seconds) */
		gyrox = (double) gyro_value[0] / GYRO_SENSITIVITY;
		gyroy = (double) gyro_value[1] / GYRO_SENSITIVITY;
		gyroz = (double) gyro_value[2] / GYRO_SENSITIVITY;

		accx = (double) acc_value[0] / ACC_FS_SENSITIVITY_0 ;
		accy = (double) acc_value[1] / ACC_FS_SENSITIVITY_0 ;
		accz = (double) acc_value[2] / ACC_FS_SENSITIVITY_0 ;

		#if 0
		        /* print just the raw values read */
		       printf("Acc(raw)=> X:%d Y:%d Z:%d gyro(raw)=> X:%d Y:%d Z:%d \n", \
		                   acc_value[0],acc_value[1],acc_value[2],gyro_value[0],gyro_value[1],gyro_value[2]);

		       /* print the 'g' and '°/s' values */
		       printf("Acc(g)=> X:%.2f Y:%.2f Z:%.2f gyro(dps)=> X:%.2f Y:%.2f Z:%.2f \n", \
		               accx,accy,accz,gyrox,gyroy,gyroz);
		#endif

		#if 1

//		       printf("%0.2f %0.2f %0.2f\n",accx,accy,accz);

		       double x2 = accx * accx;
		       double z2 = accz * accz;
		       double y2 = accy * accy;

		       double x2z2 = x2 + z2;
		       double y2z2 = y2 + z2;

		       double rootxz = sqrt(x2z2);
		       double rootyz = sqrt(y2z2);

		       double xAngle= asin(accx / rootxz) * 180 / M_PI;
		       double yAngle = asin(accy / rootyz) * 180 / M_PI;
		       if(xAngle < 0)
		       {
		    	   xAngle *= -1;
		    	   xAngle = 360 - xAngle;
		       }
		       if(yAngle < 0)
		       {
		    	   yAngle *= -1;
		    	   yAngle = 360 - yAngle;
		       }

		       printf("%f %f\n",xAngle, yAngle);

		#endif
		usleep(50 * 1000);
	}

	return 0;
}

