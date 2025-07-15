#define DT_DRV_COMPAT silabns_si5351

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(si5351_driver, CONFIG_SI5351_LOG_LEVEL);

#include <zephyr/drivers/si5351.h>
#include <zephyr/drivers/i2c.h>
#include "si5351_regmap.h"
#include <stdlib.h>

struct si5351_data {
	si5351_crystal_freq_t crystal_freq; //!< Crystal frequency
	si5351_crystal_load_t crystal_load; //!< Crystal load capacitors
	uint32_t crystal_ppm;               //!< Frequency synthesis
	bool plla_configured;               //!< Phase-locked loop A configured
	uint32_t plla_freq;                 //!< Phase-locked loop A frequency
	bool pllb_configured;               //!< Phase-locked loop B configured
	uint32_t pllb_freq;                 //!< Phase-locked loop B frequency
	uint8_t last_rdiv_value[3];
};

static int si5351_i2c_write(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t data_len)
{
	const struct si5351_config *config = dev->config;

	struct i2c_msg msgs[] = {
		{
			.buf = &reg,
			.len = 1,
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = data,
			.len = data_len,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		},
	};
	return i2c_transfer_dt(&config->i2c, msgs, ARRAY_SIZE(msgs));
}

static int si5351_i2c_read(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t data_len)
{
	const struct si5351_config *config = dev->config;
	return i2c_write_read_dt(&config->i2c, &reg, 1, data, data_len);
}

static int si5351_i2c_write8(const struct device *dev, uint8_t reg, uint8_t data)
{
	return si5351_i2c_write(dev, reg, &data, 1);
}

static int si5351_i2c_read8(const struct device *dev, uint8_t reg, uint8_t *data)
{
	return si5351_i2c_read(dev, reg, data, 1);
}

static int si5351_setup_pll_int(const struct device *dev, si5351_pll_t pll, uint8_t mult);
{
	return si5351_setup_pll(dev, pll, mult, 0, 1);
}
static int si5351_setup_pll(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num,
			    uint32_t denom)
{
	const struct si5351_config *config = dev->config;
	struct si5351_data *data = dev->data;
	int err;

	uint32_t P1; /* PLL config register P1 */
	uint32_t P2; /* PLL config register P2 */
	uint32_t P3; /* PLL config register P3 */

	/* Basic validation */
	if (mult < 15 || mult > 90) {
		return EINVAL;
	}
	if (denom == 0) {
		return EINVAL;
	}
	if (num > 0xFFFFF) {
		return EINVAL;
	}
	if (denum > 0xFFFFF) {
		return EINVAL;
	}

	/* Feedback Multisynth Divider Equation
	 *
	 * where: a = mult, b = num and c = denom
	 *
	 * P1 register is an 18-bit value using following formula:
	 *
	 * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
	 *
	 * P2 register is a 20-bit value using the following formula:
	 *
	 * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
	 *
	 * P3 register is a 20-bit value using the following formula:
	 *
	 * 	P3[19:0] = denom
	 */

	/* Set the main PLL config registers */
	if (num == 0) {
		/* Integer mode */
		P1 = 128 * mult - 512;
		P2 = num;
		P3 = denom;
	} else {
		/* Fractional mode */
		P1 = (uint32_t)(128 * mult + floor(128 * ((float)num / (float)denom)) - 512);
		P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num / (float)denom)));
		P3 = denom;
	}

	/* Get the appropriate starting point for the PLL registers */
	uint8_t baseaddr = (pll == SI5351_PLL_A ? SI5351_REGISTER_26 : SI5351_REGISTER_34);

	/* The datasheet is a nightmare of typos and inconsistencies here! */
	uint8_t regs[] = {
		(P3 & 0x0000FF00) >> 8,  (P3 & 0x000000FF),
		(P1 & 0x00030000) >> 16, (P1 & 0x0000FF00) >> 8,
		(P1 & 0x000000FF),       ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16),
		(P2 & 0x0000FF00) >> 8,  (P2 & 0x000000FF),
	};

	err = si5351_i2c_write(baseaddr, regs, ARRAY_SIZE(regs));
	if (err < 0) {
		return err;
	}

	/* Reset both PLLs */
	err = si5351_i2c_si5351_i2c_write8(SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5));
	if (err < 0) {
		return err;
	}

	/* Store the frequency settings for use with the Multisynth helper */
	if (pll == SI5351_PLL_A) {
		float fvco = data->crystal_freq * (mult + ((float)num / (float)denom));
		data->plla_configured = true;
		data->plla_freq = (uint32_t)floor(fvco);
	} else {
		float fvco = data->crystal_freq * (mult + ((float)num / (float)denom));
		data->pllb_configured = true;
		data->pllb_freq = (uint32_t)floor(fvco);
	}

	return 0;
}

static int si5351_enable_spread_spectrum(bool enabled)
{
	uint8_t regval;
	int err = si5351_i2c_read8(SI5351_REGISTER_149_SPREAD_SPECTRUM_PARAMETERS, &regval);
	if (err < 0) {
		return err;
	}

	if (enabled) {
		regval |= 0x80;
	} else {
		regval &= ~0x80;
	}

	return si5351_i2c_write8(SI5351_REGISTER_149_SPREAD_SPECTRUM_PARAMETERS, regval);
}

static const struct si5351_driver_api si5351_api = {.si5351_setup_pll_int = &si5351_setup_pll_int,
						    .si5351_setup_pll = &si5351_setup_pll};

static int si5351_init(const struct device *dev)
{
	const struct si5351_config *config = dev->config;
	struct si5351_data *data = dev->data;
	int err;


	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Set initial data values */
	data->crystal_freq = SI5351_CRYSTAL_FREQ_25MHZ;
	data->crystal_load = SI5351_CRYSTAL_LOAD_10PF;
	data->crystal_ppm = 30;
	data->plla_configured = false;
	data->plla_freq = 0;
	data->pllb_configured = false;
	data->pllb_freq = 0;

	for (uint8_t i = 0; i < 3; i++) {
		data->last_rdiv_value[i] = 0;
	}

	uint8_t status;
	err = si5351_i2c_read8(SI5351_REGISTER_0_DEVICE_STATUS, &status);
	if (err < 0) {
		LOG_ERR("Unable to communicate with si5351 over i2c: %d", err);
		return err;
	} else {
		LOG_INF("Si5351 status: %x", status);
	}

	/* Disable all outputs setting CLKx_DIS high */
	err = si5351_i2c_write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);
	if (err < 0) {
		return err;
	}

	/* Power down all output drivers */
	uint8_t clk_control[8];
	for (size_t i = 0; i < 8; ++i) {
		clk_control[i] = 0x80;
	}
	err = si5351_i2c_write(SI5351_REGISTER_16_CLK0_CONTROL, clk_control, 8);
	if (err < 0) {
		return err;
	}

	/* Set the load capacitance for the XTAL */
	err = si5351_i2c_write8(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
				data->crystal_load);
	if (err < 0) {
		return err;
	}

	/* Disable spread spectrum output. */
	si5351_enable_spread_spectrum(false);

	// /* Set interrupt masks as required (see Register 2 description in AN619).
	//    By default, ClockBuilder Desktop sets this register to 0x18.
	//    Note that the least significant nibble must remain 0x8, but the most
	//    significant nibble may be modified to suit your needs. */

	return 0;
}

#define si5351_DEFINE(inst)                                                                        \
	static struct si5351_data si5351_data_##inst;                                              \
                                                                                                   \
	static const struct si5351_config si5351_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, si5351_init, NULL, si5351_data_##inst, &si5351_config_##inst,  \
			      POST_KERNEL, CONFIG_si5351_INIT_PRIORITY, &si5351_api);

DT_INST_FOREACH_STATUS_OKAY(si5351_DEFINE)
