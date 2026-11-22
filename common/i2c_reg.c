// higher level utilities for I2C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "common/i2c_reg.h"
#include "common/pinsel.h"
#include "common/utils.h"

#include "FreeRTOSConfig.h" // configCPU_CLOCK_HZ for DELAY_US
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#endif // USE_FREERTOS

// This array helps us simplify the use of different I2C devices in the board.
const uint32_t I2C_BASE[] = {I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2C4_BASE,
                             I2C5_BASE, I2C6_BASE, I2C7_BASE, I2C8_BASE, I2C9_BASE};

// initialize I2C module 0
// Slightly modified version of TI's example code
void initI2C0(const uint32_t sysclockfreq)
{
  // enable I2C module 0
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  //
  // Wait for the I2C0 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    ;

  // Enable the module as a slave
  uint8_t slave_addr = 0x50;
  MAP_I2CSlaveInit(I2C0_BASE, slave_addr);
  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C0 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  // MAP_I2CMasterInitExpClk(I2C0_BASE, sysclockfreq, false);
  // while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

  // clear I2C FIFOs
  // HWREG(I2C0_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C0_BASE);
  MAP_I2CTxFIFOFlush(I2C0_BASE);

  // no resets here
}

// initialize I2C module 1
// Slightly modified version of TI's example code
void initI2C1(const uint32_t sysclockfreq)
{
  // enable I2C module 1
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  //
  // Wait for the I2C1 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C1 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C1_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    ;

  // clear I2C FIFOs
  // HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C1_BASE);
  MAP_I2CTxFIFOFlush(I2C1_BASE);

  // toggle relevant reset
  write_gpio_pin(_PWR_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_PWR_I2C_RESET, 0x1); // active low
}
// i2c 2 is for the clock chips
void initI2C2(const uint32_t sysclockfreq)
{
  // enable I2C module 2
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
  //
  // Wait for the I2C1 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C1 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C2_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2))
    ;

  // clear I2C FIFOs
  // HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C2_BASE);
  MAP_I2CTxFIFOFlush(I2C2_BASE);

  // toggle relevant reset
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x1); // active low
}

// I2C controller 3 is for V_OPTICS on the CM Rev 1
void initI2C3(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  //
  // Wait for the I2C3 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C3);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C3 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C3_BASE, sysclockfreq, true);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3))
    ;

  // clear I2C FIFOs
  // HWREG(I2C3_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C3_BASE);
  MAP_I2CTxFIFOFlush(I2C3_BASE);

  // toggle relevant reset
  write_gpio_pin(_F2_OPTICS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_F2_OPTICS_I2C_RESET, 0x1); // active low
}

// I2C controller 4 is for F1_OPTICS on the CM
void initI2C4(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
  //
  // Wait for the I2C4 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C4);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C4);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C4 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C4_BASE, sysclockfreq, true);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4))
    ;

  // clear I2C FIFOs
  // HWREG(I2C4_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C4_BASE);
  MAP_I2CTxFIFOFlush(I2C4_BASE);

  // toggle relevant reset
  write_gpio_pin(_F1_OPTICS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_F1_OPTICS_I2C_RESET, 0x1); // active low
}

#if defined(REV2) || defined(REV3)
// I2C controller 5 is for FPGAs on the CM
void initI2C5(const uint32_t sysclockfreq)
{
  // enable I2C module 5
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C5);
  //
  // Wait for the I2C5 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C5);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C5);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C5);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C5 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C5_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5))
    ;

  // clear I2C FIFOs
  // HWREG(I2C5_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C5_BASE);
  MAP_I2CTxFIFOFlush(I2C5_BASE);

  // toggle relevant reset
  write_gpio_pin(_FPGA_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_FPGA_I2C_RESET, 0x1); // active low
}
#elif defined(REV1)
// I2C controller 6 is for FPGAs on the CM
void initI2C6(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);
  //
  // Wait for the I2C6 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C6);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C6);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C6 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C6_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6))
    ;

  // clear I2C FIFOs
  // HWREG(I2C6_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C6_BASE);
  MAP_I2CTxFIFOFlush(I2C6_BASE);

  // toggle relevant reset
  write_gpio_pin(_FPGA_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_FPGA_I2C_RESET, 0x1); // active low
}
#endif

// ---------------------------------------------------------------------------
// I2C bus recovery (I2C spec UM10204 3.1.16 "Bus clear").
// A slave interrupted mid-read can hold SDA low indefinitely. Always clock SCL
// I2C_RECOVER_MIN_CLOCKS times, more while SDA is still low (at most
// I2C_RECOVER_MAX_CLOCKS), then issue a STOP.
//
// The pins are run as open-drain GPIOs with the output latch at 0, and the
// direction register does the work: input = released (external pull-up takes the
// line high, and the read is the pad), output = driven low. The I2C module has no
// hardware bus-clear (nothing in inc/hw_i2c.h), hence the bit-bang.
//
// Behind a TCA9548 mux only the currently selected channel(s) see these clocks.
struct i2c_recover_pins_t {
  uint32_t i2c_base;
  uint32_t gpio_base;
  uint8_t scl_pin;
  uint8_t sda_pin;
  uint32_t scl_cfg;
  uint32_t sda_cfg;
};

// indexed by bus number; pin assignments as in pinout_rev1.c / pinout_rev2.c
static const struct i2c_recover_pins_t i2c_recover_pins[] = {
    {0, 0, 0, 0, 0, 0}, // bus 0 is the I2C slave, not recoverable from here
    {I2C1_BASE, GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_1, GPIO_PG0_I2C1SCL, GPIO_PG1_I2C1SDA},
    {I2C2_BASE, GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_3, GPIO_PG2_I2C2SCL, GPIO_PG3_I2C2SDA},
    {I2C3_BASE, GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_PIN_5, GPIO_PG4_I2C3SCL, GPIO_PG5_I2C3SDA},
    {I2C4_BASE, GPIO_PORTG_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_PG6_I2C4SCL, GPIO_PG7_I2C4SDA},
#ifdef REV1
    {0, 0, 0, 0, 0, 0}, // bus 5 unused on REV1
    {I2C6_BASE, GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_7, GPIO_PA6_I2C6SCL, GPIO_PA7_I2C6SDA},
#else
    {I2C5_BASE, GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_1, GPIO_PB0_I2C5SCL, GPIO_PB1_I2C5SDA},
#endif
};
#define N_RECOVER_BUSES (sizeof(i2c_recover_pins) / sizeof(i2c_recover_pins[0]))

#define I2C_RECOVER_MIN_CLOCKS  9
#define I2C_RECOVER_MAX_CLOCKS  16
#define I2C_RECOVER_STRETCH_US  1000 // max wait for SCL to rise (clock stretching)
#define I2C_RECOVER_HALF_PER_US 5    // ~100 kHz

// release SCL and wait (bounded) for it to go high; false if it stays low
static bool scl_release_wait(const struct i2c_recover_pins_t *p)
{
  MAP_GPIODirModeSet(p->gpio_base, p->scl_pin, GPIO_DIR_MODE_IN);
  for (uint32_t us = 0; us < I2C_RECOVER_STRETCH_US; ++us) {
    if (MAP_GPIOPinRead(p->gpio_base, p->scl_pin))
      return true;
    DELAY_US(1);
  }
  return false;
}

int i2c_bus_recover(uint8_t bus, struct i2c_recover_info *info)
{
  info->pre = info->post = info->nclocks = 0;
  info->status = I2C_RECOVER_BAD_BUS;
  if (bus >= N_RECOVER_BUSES || i2c_recover_pins[bus].i2c_base == 0)
    return I2C_RECOVER_BAD_BUS;
  const struct i2c_recover_pins_t *p = &i2c_recover_pins[bus];
  const uint32_t base = p->gpio_base;
  const uint32_t mbmon_mask = I2C_MBMON_SCL | I2C_MBMON_SDA;

  info->pre = HWREG(p->i2c_base + I2C_O_MBMON) & mbmon_mask;

  // Take the pins away from the controller without a glitch: latch 0, then
  // direction-in (which also clears AFSEL). The pads are already open-drain from
  // GPIOPinTypeI2C/I2CSCL, so direction-out pulls down and never drives high.
  MAP_GPIOPinWrite(base, p->scl_pin | p->sda_pin, 0);
  MAP_GPIODirModeSet(base, p->scl_pin | p->sda_pin, GPIO_DIR_MODE_IN);

  int status = I2C_RECOVER_OK;
  if (!scl_release_wait(p)) {
    status = I2C_RECOVER_SCL_STUCK;
  }
  else {
    // 9 clocks unconditionally, then more until the slave releases SDA
    while ((info->nclocks < I2C_RECOVER_MIN_CLOCKS || !MAP_GPIOPinRead(base, p->sda_pin)) &&
           info->nclocks < I2C_RECOVER_MAX_CLOCKS) {
      MAP_GPIODirModeSet(base, p->scl_pin, GPIO_DIR_MODE_OUT);
      DELAY_US(I2C_RECOVER_HALF_PER_US);
      ++info->nclocks;
      if (!scl_release_wait(p)) {
        status = I2C_RECOVER_SCL_STUCK;
        break;
      }
      DELAY_US(I2C_RECOVER_HALF_PER_US);
    }
    if (status == I2C_RECOVER_OK) {
      // STOP: SDA rises while SCL is high
      MAP_GPIODirModeSet(base, p->scl_pin, GPIO_DIR_MODE_OUT);
      DELAY_US(I2C_RECOVER_HALF_PER_US);
      MAP_GPIODirModeSet(base, p->sda_pin, GPIO_DIR_MODE_OUT);
      DELAY_US(I2C_RECOVER_HALF_PER_US);
      (void)scl_release_wait(p);
      DELAY_US(I2C_RECOVER_HALF_PER_US);
      MAP_GPIODirModeSet(base, p->sda_pin, GPIO_DIR_MODE_IN);
      DELAY_US(I2C_RECOVER_HALF_PER_US);
    }
  }

  // hand the pins back to the controller, as in pinout_rev*.c
  MAP_GPIODirModeSet(base, p->scl_pin | p->sda_pin, GPIO_DIR_MODE_IN);
  MAP_GPIOPinConfigure(p->scl_cfg);
  MAP_GPIOPinTypeI2CSCL(base, p->scl_pin);
  MAP_GPIOPinConfigure(p->sda_cfg);
  MAP_GPIOPinTypeI2C(base, p->sda_pin);

  info->post = HWREG(p->i2c_base + I2C_O_MBMON) & mbmon_mask;
  if (status == I2C_RECOVER_OK && info->post != mbmon_mask)
    status = (info->post & I2C_MBMON_SCL) ? I2C_RECOVER_SDA_STUCK : I2C_RECOVER_SCL_STUCK;
  info->status = status;
  return status;
}

const char *i2c_recover_status_str(int status)
{
  static const char *const str[] = {"OK", "bad bus", "SCL stuck low", "SDA stuck low", "mux select failed"};
  return (status >= 0 && status < (int)(sizeof(str) / sizeof(str[0]))) ? str[status] : "unknown";
}
