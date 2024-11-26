//*****************************************************************************
// production testing firmware 
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

// TI includes
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

// local includes
#include "common/pinout.h"
#include "prod_test.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
  while(1);
}
#endif


uint32_t g_ui32SysClock;

void SystemInit(void)
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  // This function does not exist in the ROM
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320),
                                          configCPU_CLOCK_HZ);

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();
}


//*****************************************************************************
//
// main entry point
//
//*****************************************************************************
__attribute__((noreturn)) int main(void)
{
    SystemInit();


  //
  // Loop forever.
  //
  while(1) {
    // do something.     
    // Delay for a bit.
    //
    for(uint32_t ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }

    
  }
}
