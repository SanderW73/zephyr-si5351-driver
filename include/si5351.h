#ifndef __SI5351_H__
#define __SI5351_H__
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

struct si5351_config {
	struct i2c_dt_spec i2c;
};

typedef enum {
  SI5351_PLL_A = 0,
  SI5351_PLL_B,
} si5351_pll_t;

typedef enum {
  SI5351_CRYSTAL_LOAD_6PF = (1 << 6),
  SI5351_CRYSTAL_LOAD_8PF = (2 << 6),
  SI5351_CRYSTAL_LOAD_10PF = (3 << 6)
} si5351_crystal_load_t;

typedef enum {
  SI5351_CRYSTAL_FREQ_25MHZ = (25000000),
  SI5351_CRYSTAL_FREQ_27MHZ = (27000000)
} si5351_crystal_freq_t;

typedef enum {
  SI5351_MULTISYNTH_DIV_4 = 4,
  SI5351_MULTISYNTH_DIV_6 = 6,
  SI5351_MULTISYNTH_DIV_8 = 8
} si5351_multisynth_div_t;

typedef enum {
  SI5351_R_DIV_1 = 0,
  SI5351_R_DIV_2 = 1,
  SI5351_R_DIV_4 = 2,
  SI5351_R_DIV_8 = 3,
  SI5351_R_DIV_16 = 4,
  SI5351_R_DIV_32 = 5,
  SI5351_R_DIV_64 = 6,
  SI5351_R_DIV_128 = 7,
} si5351_rdiv_t;


typedef int (*si5351_cmd_t)(const struct device *dev);
typedef int (*si5351_setup_pll_int_t)(const struct device *dev, si5351_pll_t pll, uint8_t mult);
typedef int (*si5351_setup_pll_t)(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num, uint32_t denomr);


__subsystem struct si5351_driver_api {
	si5351_setup_pll_int_t si5351_setup_pll_int;
	si5351_setup_pll_t si5351_setup_pll;
};


__syscall int si5351_setup_pll_int(const struct device *dev, si5351_pll_t pll, uint8_t mult);

static inline int z_impl_si5351_setup_pll_int(const struct device *dev, si5351_pll_t pll, uint8_t mult)
{
	const struct si5351_driver_api *api = (const struct si5351_driver_api *)dev->api;
	if (api->si5351_setup_pll_int == NULL) {
		return -ENOSYS;
	}
	return api->si5351_setup_pll_int(dev, mult);
}


__syscall int si5351_setup_pll(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num, uint32_t denom);

static inline int z_impl_si5351_setup_pll(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
	const struct si5351_driver_api *api = (const struct si5351_driver_api *)dev->api;
	if (api->si5351_setup_pll == NULL) {
		return -ENOSYS;
	}
	return api->si5351_setup_pll(dev, mult, num, denom);
}

#include <syscalls/libostentus.h>

#endif