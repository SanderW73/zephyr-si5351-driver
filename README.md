# Zephyr driver for Adafruit's Si5351 Clockgen Breakout Board

A zephyr driver based on the [Adafruit_Si5351_Library](https://github.com/adafruit/Adafruit_Si5351_Library.git)

The driver is tested with [Adafruit Si5351A Clock Generator Breakout Board - 8KHz to 160MHz](https://www.adafruit.com/product/2045), 
but should work for any board with the Si5351A chip (10-MSOP with 3 outputs) attached to the I2C bus.

Datasheet: https://cdn-shop.adafruit.com/datasheets/Si5351.pdf

Note: This has been created as an exercise in writing a Zephyr driver. It may need some extra work for serious applications,
and certainly more testing.

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``zephyr-si5351-driver`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the zephyr-si5351-driver (master branch)
west init -m https://github.com/SanderW73/zephyr-si5351-driver --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

### Building and running

To build the example application, run the following command:

```shell
cd zephyr-si5351-driver
west build -b $BOARD app
```

where `$BOARD` is the target board. Only the `rpi_pico` has an
appropriate overlay (see `app/boards`).

Once you have built the application, run the following command to flash it:

```shell
west flash
```

### Connecting and running

Connect the Si5351 breakout board to the Raspberry Pi Pico board:

| Si5351 | Pico |
|--------|------|
| GND    | GND  |
| VIN    | 3V3 (OUT) |
| SDA    | I2C0 SDA (P6) |
| SCL    | I2C0 SCL (P7) |

Add pull-up resistors (4K7) to SDA and SCL.

Expected output on the console after reset:

```
*** Booting Zephyr OS build v4.2.0-rc3-15-g66acb364b190 ***
Set PLLA to 900MHz
Set Output #0 to 112.5MHz
Set Output #1 to 13.553115MHz
Set Output #2 to 10.706 KHz
Si5351 status: 0x11
```