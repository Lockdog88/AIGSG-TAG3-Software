#ifndef LIS3DE_H_
#define LIS3DE_H_

#define LIS3DE_ID				0x33

//#define LIS3DE_READ_ADDR        0x53
#define LIS3DE_ADDR				0x29

#define LIS3DE_STATUS_REG_AUX	0x07
#define LIS3DE_OUT_ADC1_L     	0x08
#define LIS3DE_OUT_ADC1_H		0x09
#define LIS3DE_OUT_ADC2_L		0x0A
#define LIS3DE_OUT_ADC2_H		0x0B
#define LIS3DE_OUT_ADC3_L		0x0C
#define LIS3DE_OUT_ADC3_H		0x0D
#define LIS3DE_INT_COUNTER_REG	0x0E
#define LIS3DE_WHO_AM_I			0x0F

#define LIS3DE_TEMP_CFG_REG		0x1F
#define LIS3DE_CTRL_REG1		0x20
#define LIS3DE_CTRL_REG2		0x21
#define LIS3DE_CTRL_REG3		0x22
#define LIS3DE_CTRL_REG4		0x23
#define LIS3DE_CTRL_REG5		0x24
#define LIS3DE_CTRL_REG6		0x25
#define LIS3DE_REFERENCE		0x26
#define LIS3DE_STATUS_REG2		0x27
#define LIS3DE_OUT_X			0x29
#define LIS3DE_OUT_Y			0x2B
#define LIS3DE_OUT_Z			0x2D
#define LIS3DE_FIFO_CTRL_REG	0x2E
#define LIS3DE_FIFO_SRC_REG		0x2F
#define LIS3DE_IG1_CFG			0x30
#define LIS3DE_IG1_SOURCE		0x31
#define LIS3DE_IG1_THS			0x32
#define LIS3DE_IG1_DURATION		0x33
#define LIS3DE_IG2_CFG			0x34
#define LIS3DE_IG2_SOURCE		0x35
#define LIS3DE_IG2_THS			0x36
#define LIS3DE_IG2_DURATION		0x37
#define LIS3DE_CLICK_CFG		0x38
#define LIS3DE_CLICK_SRC		0x39
#define LIS3DE_CLICK_THS		0x3A
#define LIS3DE_TIME_LIMIT		0x3B
#define LIS3DE_TIME_LATENCY		0x3C
#define LIS3DE_TIME_WINDOW		0x3D
#define LIS3DE_ACT_THS			0x3E
#define LIS3DE_ACT_DUR			0x3F

#define EXTRACT_ACCEL(msb, lsb) (int16_t)((lsb) | (msb) << 8)

typedef struct {
	float x, y, z;
} lis3de_data_t;

typedef enum
{
  LIS3DH_DATARATE_400_HZ     = 0b0111, //  400Hz
  LIS3DH_DATARATE_200_HZ     = 0b0110, //  200Hz
  LIS3DH_DATARATE_100_HZ     = 0b0101, //  100Hz
  LIS3DH_DATARATE_50_HZ      = 0b0100, //   50Hz
  LIS3DH_DATARATE_25_HZ      = 0b0011, //   25Hz
  LIS3DH_DATARATE_10_HZ      = 0b0010, // 10 Hz
  LIS3DH_DATARATE_1_HZ       = 0b0001, // 1 Hz
  LIS3DH_DATARATE_POWERDOWN  = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ  =  0b1001,

} lis3de_dataRate_t;

typedef enum
{
  LIS3DE_RANGE_16_G     = 0b11,   // +/- 16g
  LIS3DE_RANGE_8_G      = 0b10,   // +/- 8g
  LIS3DE_RANGE_4_G      = 0b01,   // +/- 4g
  LIS3DE_RANGE_2_G      = 0b00,    // +/- 2g (default value)
} lis3de_range_t;

typedef enum
{
  LIS3DE_AXIS_X         = 0x0,
  LIS3DE_AXIS_Y         = 0x1,
  LIS3DE_AXIS_Z         = 0x2,
} lis3de_axis_t;

int lis3de_init(void);
uint8_t lis3de_selftest(void);
void lis3de_setRange(lis3de_range_t range);
lis3de_range_t lis3de_getRange(void);
void lis3de_setDataRate(lis3de_dataRate_t dataRate);
lis3de_dataRate_t lis3de_getDataRate(void);
int lis3de_read(lis3de_data_t *const data);

int i2c_init(void);
int i2c_write_register(uint16_t addr, uint8_t reg, uint8_t data);
int i2c_read_register(uint16_t addr, uint8_t reg, uint8_t *const data, uint32_t len);

#endif /* LIS3DE_H_ */
