/**************************************************************************
 * Based on the Adafruit Si5351 Library, adapted for Zephyr
 *
 * https://github.com/adafruit/Adafruit_Si5351_Library
 * by K. Townsend (Adafruit Industries)
 *
 * Si5351A/B/C Datasheet:
 * http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf
 *
 * Manually Generating an Si5351 Register Map:
 * http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
 *
 * 
 * Original copyright an license:
 * 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Adafruit Industries
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

#define DT_DRV_COMPAT silabs_si5351

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(si5351_driver, CONFIG_SI5351_LOG_LEVEL);

#include <zephyr/drivers/i2c.h>
#include <drivers/si5351.h>
#include "si5351_regmap.h"
#include <stdlib.h>
#include <math.h>

struct si5351_data {
	bool initialised;
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

	int err = i2c_write_dt(&config->i2c, &reg, 1);
	if (err < 0) {
		return err;
	}
	return i2c_read_dt(&config->i2c, data, data_len);
}

static int si5351_i2c_write8(const struct device *dev, uint8_t reg, uint8_t data)
{
	return si5351_i2c_write(dev, reg, &data, 1);
}

static int si5351_i2c_read8(const struct device *dev, uint8_t reg, uint8_t *data)
{
	return si5351_i2c_read(dev, reg, data, 1);
}


int si5351_enable_spread_spectrum(const struct device *dev, bool enabled);

int si5351_begin(const struct device *dev)
{
	struct si5351_data *data = dev->data;
	int err = 0;

	LOG_DBG("si5351_begin");

	uint8_t status;
	err = si5351_i2c_read8(dev, SI5351_REGISTER_0_DEVICE_STATUS, &status);
	if (err < 0) {
		LOG_ERR("Unable to communicate with si5351 over i2c: %d", err);
		return err;
	} else {
		LOG_INF("Si5351 status: %02x", status);
	}

	/* Disable all outputs setting CLKx_DIS high */
	err = si5351_i2c_write8(dev, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);
	if (err < 0) {
		goto on_error;
	}

	/* Power down all output drivers */
	uint8_t clk_control[8];
	for (size_t i = 0; i < 8; ++i) {
		clk_control[i] = 0x80;
	}
	err = si5351_i2c_write(dev, SI5351_REGISTER_16_CLK0_CONTROL, clk_control, 8);
	if (err < 0) {
		goto on_error;
	}

	/* Set the load capacitance for the XTAL */
	err = si5351_i2c_write8(dev, SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
				data->crystal_load);
	if (err < 0) {
		goto on_error;
	}

	/* Disable spread spectrum output. */
	si5351_enable_spread_spectrum(dev, false);

	// /* Set interrupt masks as required (see Register 2 description in AN619).
	//    By default, ClockBuilder Desktop sets this register to 0x18.
	//    Note that the least significant nibble must remain 0x8, but the most
	//    significant nibble may be modified to suit your needs. */

	/* Reset the PLL config fields just in case we call init again */
	data->plla_configured = false;
	data->plla_freq = 0;
	data->pllb_configured = false;
	data->pllb_freq = 0;

	/* All done! */
	data->initialised = true;

	return 0;

on_error:
	LOG_ERR("si5351_begin failed (%d)", err);
	return err;
}

int si5351_setup_pll(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num,
		     uint32_t denom)
{
	struct si5351_data *data = dev->data;
	int err = 0;

	uint32_t P1; /* PLL config register P1 */
	uint32_t P2; /* PLL config register P2 */
	uint32_t P3; /* PLL config register P3 */

	LOG_DBG("si5351_setup_pll(%d, %d, %d)", pll, mult, num);

	/* Basic validation */
	if (!data->initialised) {
		err = EPERM;
	}
	if (mult < 15 || mult > 90) {
		err = EINVAL;
	}
	if (denom == 0) {
		err = EINVAL;
	}
	if (num > 0xFFFFF) {
		err = EINVAL;
	}
	if (denom > 0xFFFFF) {
		err = EINVAL;
	}
	if (err < 0) {
		goto on_error;
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
	uint8_t reg_values[] = {
		(P3 & 0x0000FF00) >> 8,  (P3 & 0x000000FF),
		(P1 & 0x00030000) >> 16, (P1 & 0x0000FF00) >> 8,
		(P1 & 0x000000FF),       ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16),
		(P2 & 0x0000FF00) >> 8,  (P2 & 0x000000FF),
	};

	err = si5351_i2c_write(dev, baseaddr, reg_values, ARRAY_SIZE(reg_values));
	if (err < 0) {
		goto on_error;
	}

	/* Reset both PLLs */
	err = si5351_i2c_write8(dev, SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5));
	if (err < 0) {
		goto on_error;
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

on_error:
	LOG_ERR("si5351_setup_pll failed (%d)", err);
	return err;
}

int si5351_setup_multisynth(const struct device *dev, si5351_output_t output, si5351_pll_t pll, uint32_t div,
			   uint32_t num, uint32_t denom)
{
	struct si5351_data *data = dev->data;
	int err = 0;

	uint32_t P1; /* Multisynth config register P1 */
	uint32_t P2; /* Multisynth config register P2 */
	uint32_t P3; /* Multisynth config register P3 */

	LOG_DBG("si5351_setup_multisynth(%d, %d, %d, %d, %d)", output, pll, div, num, denom);

	/* Basic validation */
	if (!data->initialised) {
		err = EPERM;
	}
	if (output >= 3) {
		err = EINVAL;
	}
	if (div <= 3 || div > 2048) {
		err = EINVAL;
	}
	if (denom == 0) {
		err = EINVAL;
	}
	if (num > 0xFFFFF) {
		err = EINVAL;
	}
	if (denom > 0xFFFFF) {
		err = EINVAL;
	}

	/* Make sure the requested PLL has been initialised */
	if (pll == SI5351_PLL_A) {
		if (!data->plla_configured) {
			err = EINVAL;
		}
	} else {
		if (!data->pllb_configured) {
			err = EINVAL;
		}
	}

	if (err < 0) {
		goto on_error;
	}

	/* Output Multisynth Divider Equations
	 *
	 * where: a = div, b = num and c = denom
	 *
	 * P1 register is an 18-bit value using following formula:
	 *
	 * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
	 *
	 * P2 register is a 20-bit value using the following formula:
	 *
	 * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
	 *
	 * P3 register is a 20-bit value using the following formula:
	 *
	 * 	P3[19:0] = c
	 */

	/* Set the main PLL config registers */
	if (num == 0) {
		/* Integer mode */
		P1 = 128 * div - 512;
		P2 = 0;
		P3 = denom;
	} else if (denom == 1) {
		/* Fractional mode, simplified calculations */
		P1 = 128 * div + 128 * num - 512;
		P2 = 128 * num - 128;
		P3 = 1;
	} else {
		/* Fractional mode */
		P1 = (uint32_t)(128 * div + floor(128 * ((float)num / (float)denom)) - 512);
		P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num / (float)denom)));
		P3 = denom;
	}

	/* Get the appropriate starting point for the PLL registers */
	uint8_t baseaddr = 0;
	switch (output) {
	case 0:
		baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
		break;
	case 1:
		baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
		break;
	case 2:
		baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
		break;
	}

	/* Set the MSx config registers */
	/* Burst mode: register address auto-increases */
	uint8_t reg_values[] = {
		(P3 & 0xFF00) >> 8,
		P3 & 0xFF,
		((P1 & 0x30000) >> 16) | data->last_rdiv_value[output],
		(P1 & 0xFF00) >> 8,
		P1 & 0xFF,
		((P3 & 0xF0000) >> 12) | ((P2 & 0xF0000) >> 16),
		(P2 & 0xFF00) >> 8,
		P2 & 0xFF,
	};
	err = si5351_i2c_write(dev, baseaddr, reg_values, ARRAY_SIZE(reg_values));
	if (err < 0) {
		goto on_error;
	}

	/* Configure the clk control and enable the output */
	/* TODO: Check if the clk control byte needs to be updated. */
	uint8_t clkControlReg = 0x0F; /* 8mA drive strength, MS0 as CLK0 source, Clock
					 not inverted, powered up */
	if (pll == SI5351_PLL_B) {
		clkControlReg |= (1 << 5); /* Uses PLLB */
	}
	if (num == 0) {
		clkControlReg |= (1 << 6); /* Integer mode */
	}
	switch (output) {
	case 0:
		err = si5351_i2c_write8(dev, SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg);
	case 1:
		err = si5351_i2c_write8(dev, SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg);
	case 2:
		err = si5351_i2c_write8(dev, SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg);
	}
	if (err < 0) {
		goto on_error;
	}

	return 0;

on_error:
	LOG_ERR("si5351_setup_multisynth failed (%d)", err);
	return err;
}

int si5351_enable_spread_spectrum(const struct device *dev, bool enabled)
{
	uint8_t reg_value;
	LOG_DBG("si5351_enable_spread_spectrum(%d)", enabled);

	int err = si5351_i2c_read8(dev, SI5351_REGISTER_149_SPREAD_SPECTRUM_PARAMETERS, &reg_value);
	if (err < 0) {
		return err;
	}

	if (enabled) {
		reg_value |= 0x80;
	} else {
		reg_value &= ~0x80;
	}

	err = si5351_i2c_write8(dev, SI5351_REGISTER_149_SPREAD_SPECTRUM_PARAMETERS, reg_value);
	if (err < 0) {
		LOG_ERR("si5351_enable_spread_spectrum failed (%d)", err);
	}
	return err;
}

int si5351_enable_outputs(const struct device *dev, bool enabled)
{
	struct si5351_data *data = dev->data;
	int err = 0;

	if (!data->initialised) {
		return EPERM;
	}

	err =  si5351_i2c_write8(dev, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL,
				enabled ? 0x00 : 0xFF);
	if (err < 0) {
		LOG_ERR("si5351_enable_outputs failed (%d)", err);
	}
	return err;
}

int si5351_enable_selected_outputs(const struct device *dev, bool enabled_0, bool enabled_1, bool enabled_2)
{
	LOG_DBG("si5351_enable_selected_outputs(%d, %d, %d)", enabled_0, enabled_1, enabled_2);

	uint8_t flags = (enabled_0 ? 0b001 : 0) + (enabled_1 ? 0b010 : 0) + (enabled_2 ? 0b100 : 0);
	int err = si5351_i2c_write8(dev, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, ~flags);
	if (err < 0) {
		LOG_ERR("si5351_enable_selected_outputs failed (%d)", err);
	}
	return err;
}

int si5351_get_status(const struct device *dev, uint8_t *status)
{
	LOG_DBG("si5351_get_status()");

	int err = si5351_i2c_read8(dev, SI5351_REGISTER_0_DEVICE_STATUS, status);
	if (err < 0) {
		LOG_ERR("si5351_get_status failed (%d)", err);
	}
	return err;
}

int si5351_setup_rdiv(const struct device *dev, si5351_output_t output, si5351_rdiv_t div)
{
	struct si5351_data *data = dev->data;
	int err = 0;

	LOG_DBG("si5351_setup_rdiv(%d, %d)", output, div);

	if (output >= 3) {
		err = EINVAL;
		goto on_error;
	}

	uint8_t reg, reg_value;

	if (output == 0) {
		reg = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
	}
	if (output == 1) {
		reg = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
	}
	if (output == 2) {
		reg = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;
	}

	err = si5351_i2c_read8(dev, reg, &reg_value);
	if (err < 0) {
		goto on_error;
	}

	reg_value &= 0x0F;
	uint8_t divider = div;
	divider &= 0x07;
	divider <<= 4;
	reg_value |= divider;
	data->last_rdiv_value[output] = divider;

	err = si5351_i2c_write8(dev, reg, reg_value);
	if (err < 0) {
		goto on_error;
	}

	return 0;

on_error:
	LOG_DBG("si5351_setup_rdiv failed (%d)", err);
	return err;
}

static int init_pll(const struct device *dev, si5351_pll_t pll, const struct si5351_pll_config *config)
{
	if (config->enabled) {
		return si5351_setup_pll(dev, pll, config->mult, config->num, config->denom);
	}
	return 0;
}

static int init_output(const struct device *dev, si5351_output_t output, const struct si5351_output_config *config)
{
	int err = 0;
	if (config->enabled) {
		err = si5351_setup_multisynth(dev, output, config->pll, config->div, config->num, config->denom);
	}
	if (err >= 0 && config->rdiv >= 0) {
		err = si5351_setup_rdiv(dev, output, config->rdiv);
	}
	return err;
}

static int init_enable_outputs(const struct device *dev, bool enabled)
{
	const struct si5351_config *config = dev->config;
	if (enabled) {
		return si5351_enable_selected_outputs(dev, config->out_0.enabled, config->out_1.enabled, config->out_2.enabled);
	}
	return 0;
}

static int si5351_init(const struct device *dev)
{
	const struct si5351_config *config = dev->config;
	struct si5351_data *data = dev->data;
	int err = 0;

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

	if (config->pll_a.enabled || config->pll_b.enabled)
	{
		err = si5351_begin(dev);
		if (err < 0) {
			return err;
		}
	}

	err = init_pll(dev, SI5351_PLL_A, &config->pll_a);
	if (err < 0) {
		return err;
	}

	err = init_pll(dev, SI5351_PLL_B, &config->pll_b);
	if (err < 0) {
		return err;
	}

	err = init_output(dev, SI5351_OUTPUT_0, &config->out_0);
	if (err < 0) {
		return err;
	}

	err = init_output(dev, SI5351_OUTPUT_1, &config->out_1);
	if (err < 0) {
		return err;
	}

	err = init_output(dev, SI5351_OUTPUT_2, &config->out_2);
	if (err < 0) {
		return err;
	}

	err = init_enable_outputs(dev, config->enable_outputs);
	return err;
}


#define si5351_DEFINE(inst)                                                                        \
	static struct si5351_data si5351_data_##inst;                                              \
                                                                                                   \
	static const struct si5351_config si5351_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.pll_a = { \
			.enabled = DT_NODE_HAS_PROP(DT_DRV_INST(inst), pll_a_mult), \
			.mult = DT_INST_PROP_OR(inst, pll_a_mult, -1), \
			.num = DT_INST_PROP(inst, pll_a_num), \
			.denom = DT_INST_PROP(inst, pll_a_denom), \
		}, \
		.pll_b = { \
			.enabled = DT_NODE_HAS_PROP(DT_DRV_INST(inst), pll_b_mult), \
			.mult = DT_INST_PROP_OR(inst, pll_b_mult, -1), \
			.num = DT_INST_PROP(inst, pll_b_num), \
			.denom = DT_INST_PROP(inst, pll_b_denom), \
		}, \
		.out_0 = { \
			.enabled = DT_NODE_HAS_PROP(DT_DRV_INST(inst), output_0_pll), \
			.pll = DT_INST_PROP_OR(inst, output_0_pll, -1), \
			.div = DT_INST_PROP_OR(inst, output_0_div, -1), \
			.num = DT_INST_PROP_OR(inst, output_0_num, -1), \
			.denom = DT_INST_PROP_OR(inst, output_0_denom, -1), \
			.rdiv = DT_INST_PROP_OR(inst, output_0_rdiv, -1), \
		}, \
		.out_1 = { \
			.enabled = DT_NODE_HAS_PROP(DT_DRV_INST(inst), output_1_pll), \
			.pll = DT_INST_PROP_OR(inst, output_1_pll, -1), \
			.div = DT_INST_PROP_OR(inst, output_1_div, -1), \
			.num = DT_INST_PROP_OR(inst, output_1_num, -1), \
			.denom = DT_INST_PROP_OR(inst, output_1_denom, -1), \
			.rdiv = DT_INST_PROP_OR(inst, output_1_rdiv, -1), \
		}, \
		.out_2 = { \
			.enabled = DT_NODE_HAS_PROP(DT_DRV_INST(inst), output_2_pll), \
			.pll = DT_INST_PROP_OR(inst, output_2_pll, -1), \
			.div = DT_INST_PROP_OR(inst, output_2_div, -1), \
			.num = DT_INST_PROP_OR(inst, output_2_num, -1), \
			.denom = DT_INST_PROP_OR(inst, output_2_denom, -1), \
			.rdiv = DT_INST_PROP_OR(inst, output_2_rdiv, -1), \
		}, \
		.enable_outputs = DT_INST_PROP(inst, enable_outputs) \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, si5351_init, NULL, &si5351_data_##inst, &si5351_config_##inst, \
			      POST_KERNEL, CONFIG_SI5351_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(si5351_DEFINE)
