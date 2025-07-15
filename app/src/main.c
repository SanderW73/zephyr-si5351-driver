#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <si5351.h>


static const struct device *si5351 = DEVICE_DT_GET_ANY(silabs_si5351);

int main(void)
{
	int err   = 0;

	/* Configure to set Console output to USB Serial */
	const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (usb_enable(NULL) != 0) {
		return 0;
	}

	/* Wait for USB connected
	uint32_t dtr = 0;
	while (!dtr) {
		uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
	*/

	if (!device_is_ready(si5351)) {
		printk("Error: Si5351 is not available.\r\n");
		return 1;
	}

	/* Initialise the sensor */
	err = si5351_begin(si5351);
	if (err < 0) {
		printk("Error %d: failed to begin.\n", err);
		return 1;
	}

	/* INTEGER ONLY MODE --> most accurate output */
	/* Setup PLLA to integer only mode @ 900MHz (must be 600..900MHz) */
	/* Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8) */
	/* 25MHz * 36 = 900 MHz, then 900 MHz / 8 = 112.5 MHz */
	printk("Set PLLA to 900MHz\n");
	err = si5351_setup_pll_int(si5351, SI5351_PLL_A, 36);
	if (err < 0) {
		printk("Error %d: failed to setup PLL A.\n", err);
		return 1;
	}

	printk("Set Output #0 to 112.5MHz\n");
	err = si5351_setup_multisynth_int(si5351, SI5351_OUTPUT_0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);
	if (err < 0) {
		printk("Error %d: failed to setup multisynth #0.\n", err);
		return 1;
	}

	/* FRACTIONAL MODE --> More flexible but introduce clock jitter */
	/* Setup PLLB to fractional mode @616.66667MHz (XTAL * 24 + 2/3) */
	/* Setup Multisynth 1 to 13.55311MHz (PLLB/45.5) */
	err = si5351_setup_pll(si5351, SI5351_PLL_B, 24, 2, 3);
	if (err < 0) {
		printk("Error %d: failed to setup PLL B.\n", err);
		return 1;
	}

	printk("Set Output #1 to 13.553115MHz\n");
	err = si5351_setup_multisynth(si5351, SI5351_OUTPUT_1, SI5351_PLL_B, 45, 1, 2);
	if (err < 0) {
		printk("Error %d: failed to setup multisynth #1.\n", err);
		return 1;
	}

	/* Multisynth 2 is not yet used and won't be enabled, but can be */
	/* Use PLLB @ 616.66667MHz, then divide by 900 -> 685.185 KHz */
	/* then divide by 64 for 10.706 KHz */
	/* configured using either PLL in either integer or fractional mode */
	printk("Set Output #2 to 10.706 KHz\n");
	err = si5351_setup_multisynth(si5351, SI5351_OUTPUT_2, SI5351_PLL_B, 900, 0, 1);
	if (err < 0) {
		printk("Error %d: failed to setup multisynth #2.\n", err);
		return 1;
	}

	err = si5351_setup_rdiv(si5351, SI5351_OUTPUT_2, SI5351_R_DIV_64);
	if (err < 0) {
		printk("Error %d: failed to setup rdiv.\n", err);
		return 1;
	}

	/* Enable the clocks */
	err = si5351_enable_outputs(si5351, true);
	if (err < 0) {
		printk("Error %d: failed to enable outputs.\n", err);
		return 1;
	}

	k_sleep(K_MSEC(100));

	uint8_t status = 0;
	err = si5351_get_status(si5351, &status);
	if (err < 0) {
		printk("Error %d: failed to get status.\n", err);
		return 1;
	}
	else {
		printk("Si5351 status: 0x%02x\n", status);
	}

	while (1) {}
}
