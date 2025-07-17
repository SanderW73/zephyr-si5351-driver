#ifndef __SI5351_H__
#define __SI5351_H__

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>


struct si5351_pll_config {
  bool enabled;
  int mult;
  int num;
  int denom;
};

struct si5351_output_config {
  bool enabled;
  int pll;
  int div;
  int num;
  int denom;
  int rdiv;
};

struct si5351_config {
	struct i2c_dt_spec i2c;

  struct si5351_pll_config pll_a;
  struct si5351_pll_config pll_b;
  struct si5351_output_config out_0;
  struct si5351_output_config out_1;
  struct si5351_output_config out_2;
  bool enable_outputs;
};


typedef enum {
  SI5351_OUTPUT_0 = 0,
  SI5351_OUTPUT_1,
  SI5351_OUTPUT_2
} si5351_output_t;


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


/**
 *  @brief  Initializes I2C and configures the breakout (call this function
 *  before doing anything else)
 */
int si5351_begin(const struct device *dev);


int si5351_get_status(const struct device *dev, uint8_t *status);

/**
 *  @brief  Sets the multiplier for the specified PLL
 *
 *  @param  pll   The PLL to configure, which must be one of the following:
 *                - SI5351_PLL_A
 *                - SI5351_PLL_B
 *  @param  mult  The PLL integer multiplier (must be between 15 and 90)
 *  @param  num   The 20-bit numerator for fractional output (0..1,048,575).
 *                Set this to '0' for integer output.
 *  @param  denom The 20-bit denominator for fractional output (1..1,048,575).
 *                Set this to '1' or higher to avoid divider by zero errors.
 *
 *  @section PLL Configuration
 *
 *  fVCO is the PLL output, and must be between 600..900MHz, where:
 *
 *      fVCO = fXTAL * (a+(b/c))
 *
 *  fXTAL = the crystal input frequency
 *  a     = an integer between 15 and 90
 *  b     = the fractional numerator (0..1,048,575)
 *  c     = the fractional denominator (1..1,048,575)
 *
 *  NOTE: Try to use integers whenever possible to avoid clock jitter
 *  (only use the a part, setting b to '0' and c to '1').
 *
 *  See: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
*/
int si5351_setup_pll(const struct device *dev, si5351_pll_t pll, uint8_t mult, uint32_t num, uint32_t denom);

/**
 * @brief  Sets the multiplier for the specified PLL using integer values
 *
 * @param  pll   The PLL to configure, which must be one of the following:
 *               - SI5351_PLL_A
 *               - SI5351_PLL_B
 * @param  mult  The PLL integer multiplier (must be between 15 and 90)
 */
static inline int si5351_setup_pll_int(const struct device *dev, si5351_pll_t pll, uint8_t mult)
{
	return si5351_setup_pll(dev, pll, mult, 0, 1);
}

/**
 *  @brief  Configures the Multisynth divider, which determines the
 *          output clock frequency based on the specified PLL input.
 *
 *  @param  output    The output channel to use (0..2)
 *  @param  pllSource	The PLL input source to use, which must be one of:
 *                    - SI5351_PLL_A
 *                    - SI5351_PLL_B
 *  @param  div       The integer divider for the Multisynth output.
 *                    If pure integer values are used, this value must
 *                    be one of:
 *                    - SI5351_MULTISYNTH_DIV_4
 *                    - SI5351_MULTISYNTH_DIV_6
 *                    - SI5351_MULTISYNTH_DIV_8
 *                    If fractional output is used, this value must be
 *                    between 8 and 900.
 *  @param  num       The 20-bit numerator for fractional output
 *                    (0..1,048,575). Set this to '0' for integer output.
 *  @param  denom     The 20-bit denominator for fractional output
 *                    (1..1,048,575). Set this to '1' or higher to
 *                    avoid divide by zero errors.
 *
 *  @section Output Clock Configuration
 *
 *  The multisynth dividers are applied to the specified PLL output,
 *  and are used to reduce the PLL output to a valid range (500kHz
 *  to 160MHz). The relationship can be seen in this formula, where
 *  fVCO is the PLL output frequency and MSx is the multisynth
 *  divider:
 *
 *      fOUT = fVCO / MSx
 *
 *  Valid multisynth dividers are 4, 6, or 8 when using integers,
 *  or any fractional values between 8 + 1/1,048,575 and 900 + 0/1
 *
 *  The following formula is used for the fractional mode divider:
 *
 *      a + b / c
 *
 *  a = The integer value, which must be 4, 6 or 8 in integer mode (MSx_INT=1)
 *      or 8..900 in fractional mode (MSx_INT=0).
 *  b = The fractional numerator (0..1,048,575)
 *  c = The fractional denominator (1..1,048,575)
 *
 *  @note   Try to use integers whenever possible to avoid clock jitter
 *
 *  @note   For output frequencies > 150MHz, you must set the divider
 *          to 4 and adjust to PLL to generate the frequency (for example
 *          a PLL of 640 to generate a 160MHz output clock). This is not
 *          yet supported in the driver, which limits frequencies to
 *          500kHz .. 150MHz.
 *
 *  @note   For frequencies below 500kHz (down to 8kHz) Rx_DIV must be
 *          used, but this isn't currently implemented in the driver.
 */
int si5351_setup_multisynth(const struct device *dev, si5351_output_t output, si5351_pll_t pll, uint32_t div, uint32_t num, uint32_t denom);

/**
 *  @brief  Configures the Multisynth divider using integer output.
 *
 *  @param  output    The output channel to use (0..2)
 *  @param  pllSource	The PLL input source to use, which must be one of:
 *                    - SI5351_PLL_A
 *                    - SI5351_PLL_B
 *  @param  div       The integer divider for the Multisynth output,
 *                    which must be one of the following values:
 *                    - SI5351_MULTISYNTH_DIV_4
 *                    - SI5351_MULTISYNTH_DIV_6
 *                    - SI5351_MULTISYNTH_DIV_8
 */
static inline int si5351_setup_multisynth_int(const struct device *dev, si5351_output_t output, si5351_pll_t pll, uint32_t div)
{
	return si5351_setup_multisynth(dev, output, pll, div, 0, 1);
}

/**
 *  @brief  Enables or disables spread spectrum
 *  @param  enabled Whether spread spectrum output is enabled
 */
int si5351_enable_spread_spectrum(const struct device *dev, bool enabled);

/**
 *  @brief  Enables or disables all clock outputs
 *  @param  enabled Whether output is enabled
 */
int si5351_enable_outputs(const struct device *dev, bool enabled);

int si5351_setup_rdiv(const struct device *dev, si5351_output_t output, si5351_rdiv_t div);

#endif
