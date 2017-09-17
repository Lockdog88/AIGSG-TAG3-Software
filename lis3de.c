#include "qm_i2c.h"
#include "qm_pinmux.h"
#include "clk.h"
#include "lis3de.h"

#define LIS3DE_I2C QM_I2C_0



int i2c_init(void)
{
	int rc;
	static const qm_i2c_config_t i2c_config = {
	    .address_mode = QM_I2C_7_BIT,
	    .mode = QM_I2C_MASTER,
	    .speed = QM_I2C_SPEED_STD,
	};

	rc = clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_M0_REGISTER);
	if (rc != 0) {
		return rc;
	}

	rc = qm_pmux_select(QM_PIN_ID_6, QM_PMUX_FN_2);
	if (rc != 0) {
		return rc;
	}

	rc = qm_pmux_select(QM_PIN_ID_7, QM_PMUX_FN_2);
	if (rc != 0) {
		return rc;
	}

	rc = qm_i2c_set_config(LIS3DE_I2C, &i2c_config);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

int i2c_read_register(uint16_t addr, uint8_t reg, uint8_t *const data,
			     uint32_t len)
{
	int rc;
	qm_i2c_status_t status;

	rc = qm_i2c_master_write(LIS3DE_I2C, addr, &reg, sizeof(reg), false,
				 &status);
	if (rc != 0) {
		return rc;
	}

	rc = qm_i2c_master_read(LIS3DE_I2C, addr, data, len, true, &status);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

int i2c_write_register(uint16_t addr, uint8_t reg, uint8_t data)
{
	int rc;
	qm_i2c_status_t status;

	rc = qm_i2c_master_write(LIS3DE_ADDR, addr, &reg, sizeof(reg), false,
				 &status);
	if (rc != 0) {
		return rc;
	}

	rc = qm_i2c_master_write(LIS3DE_ADDR, addr, &data, sizeof(data), true,
				 &status);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

int lis3de_init(void)
{
	i2c_init();
	i2c_write_register(LIS3DE_ADDR, LIS3DE_CTRL_REG1, 0x4F);
	return 0;
}

uint8_t lis3de_selftest(void)
{
	uint8_t data;
	i2c_read_register(LIS3DE_ADDR, LIS3DE_WHO_AM_I, &data, sizeof(data));
	return data;
}

int lis3de_read(lis3de_data_t *const data) {
  // read x y z at once
	int rc;
	uint8_t raw_accel[6];
	uint16_t x,y,z;

	/* Reading the 6 registers at once. */
	rc = i2c_read_register(LIS3DE_ADDR, LIS3DE_OUT_X | 0x80, raw_accel, sizeof(raw_accel));
	if (rc != 0) {
		return rc;
	}

    x = EXTRACT_ACCEL(raw_accel[1], raw_accel[0]);
	y = EXTRACT_ACCEL(raw_accel[3], raw_accel[2]);
	z = EXTRACT_ACCEL(raw_accel[5], raw_accel[4]);

    uint8_t range = lis3de_getRange();
    uint16_t divider = 1;
    if (range == LIS3DE_RANGE_16_G) divider = 1365; // different sensitivity at 16g
    if (range == LIS3DE_RANGE_8_G) divider = 4096;
    if (range == LIS3DE_RANGE_4_G) divider = 8190;
    if (range == LIS3DE_RANGE_2_G) divider = 16380;

    data->x = (float)x / divider;
    data->y = (float)y / divider;
    data->z = (float)z / divider;

    return 0;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void lis3de_setRange(lis3de_range_t range)
{
	uint8_t r;
	i2c_read_register(LIS3DE_ADDR, LIS3DE_CTRL_REG4, &r, 1);
	r &= ~(0x30);
	r |= range << 4;
	i2c_write_register(LIS3DE_ADDR, LIS3DE_CTRL_REG4, r);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
lis3de_range_t lis3de_getRange(void)
{
  /* Read the data format register to preserve bits */
	uint8_t r;
	i2c_read_register(LIS3DE_ADDR, LIS3DE_CTRL_REG4, &r, 1);
	r |= r >> 4;
	r &= 0x03;
	return (lis3de_range_t)r;
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DE (controls power consumption)
*/
/**************************************************************************/
void lis3de_setDataRate(lis3de_dataRate_t dataRate)
{
	uint8_t r;
	i2c_read_register(LIS3DE_ADDR, LIS3DE_CTRL_REG1, &r, 1);
	r &= ~(0xF0); // mask off bits
	r |= (dataRate << 4);
	i2c_write_register(LIS3DE_ADDR, LIS3DE_CTRL_REG1, r);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DE (controls power consumption)
*/
/**************************************************************************/
lis3de_dataRate_t lis3de_getDataRate(void)
{
	uint8_t r;
	i2c_read_register(LIS3DE_ADDR, LIS3DE_CTRL_REG1, &r, 1);
	r |= r >> 4;
	r &= 0x0F;
	return (lis3de_dataRate_t)r;
}



