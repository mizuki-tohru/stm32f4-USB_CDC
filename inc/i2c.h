#ifndef __i2c_h__
#define __i2c_h__

#ifdef __cplusplus
extern "C" {
#endif

void I2Cs_Initialize(void);		/* I2Cs の初期化 */
uint16_t I2Cs_read(uint16_t d_address,uint16_t address);
	// エラーだと 0xffff
uint16_t I2Cs_write(uint16_t d_address,uint16_t address,uint16_t data);
	// エラーだと 0xffff、正常だとエラーだと 0x0000

#define ADXL345_ADDRESS 0x53
#define L3G4200D_ADDRESS 0x69
#define HMC5883L_ADDRESS 0x1e
#define BMP085_ADDRESS 0x77



#define ITG3200_ADDRESS 0x68
#define HMC6352_ADDRESS 0x21
#define HMC6352_TIMER   200

#ifdef __cplusplus
}
#endif
#endif	/* __i2c_h__ */
