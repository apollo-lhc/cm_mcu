//*****************************************************************************
//
// FreeRTOS v2 -- wittich 3/2019
// initially based on the demo CORTEX_M4F_CEC_MEC_17xx_51xx_Keil_GCC
// chosen as one that has a CORTEX M4F and GCC
// that file had no makefile, so ...
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "board_specific/pinout.h"
#include "board_specific/pinsel.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

#define USER_LED1_PIN  GPIO_PIN_0
#define USER_LED2_PIN  GPIO_PIN_1
#define USER_LED12_PORT GPIO_PORTJ_BASE // same port

#define USER_LED3_PIN  GPIO_PIN_0
#define USER_LED3_PORT GPIO_PORTP_BASE


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// write by pin number or name
static inline
void write_gpio_pin(int pin, uint8_t value)
{
  uint32_t gport;
  uint8_t  gpin;
  pinsel(pin, &gport, &gpin);
  MAP_GPIOPinWrite(gport, gpin, value);
  return;
}

// write by pin number or name
static inline
uint8_t read_gpio_pin(int pin)
{
  uint32_t gport;
  uint8_t  gpin;
  uint8_t value;
  pinsel(pin, &gport, &gpin);
  value = MAP_GPIOPinRead(gport, gpin);
  return value;
}

uint32_t g_ui32SysClock = 0;

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
  uint32_t ui32Status;

  //
  // Get the interrupt status.
  //
  ui32Status = ROM_UARTIntStatus(UART4_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(UART4_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  while(ROM_UARTCharsAvail(UART4_BASE))
    {
      //
      // Read the next character from the UART and write it back to the UART.
      //
      ROM_UARTCharPutNonBlocking(UART4_BASE,
				 ROM_UARTCharGetNonBlocking(UART4_BASE));

      //
      // Blink the LED to show a character transfer is occurring.
      //
      MAP_GPIOPinWrite(USER_LED12_PORT, USER_LED2_PIN, USER_LED2_PIN);

      //
      // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
      //
      SysCtlDelay(g_ui32SysClock / (1000 * 3));

      //
      // Turn off the LED
      //
      MAP_GPIOPinWrite(USER_LED12_PORT, USER_LED2_PIN, 0x0);

    }
}


// Initialize the UART 
// based on uart_echo demo project
// we use UART4 (front panel) 
void 
UartInit(uint32_t ui32SysClock)
{
  //
  // Set relevant GPIO pins as UART pins.
  //
  //
  // Configure the GPIO Pin Mux for PA2
  // for U4RX
  //
  MAP_GPIOPinConfigure(GPIO_PA2_U4RX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2);

  //
  // Configure the GPIO Pin Mux for PA3
  // for U4TX
  //
  MAP_GPIOPinConfigure(GPIO_PA3_U4TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART4_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
  MAP_IntEnable(INT_UART4);
  MAP_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);
  return;
}

  


//*****************************************************************************
//
// Send a string to the UART. From uart_echo example.
//
//*****************************************************************************
void
UARTSend(const uint32_t base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  //
  // Loop while there are more characters to send.
  //
  while(ui32Count--)
    {
      //
      // Write the next character to the UART.
      //
      MAP_UARTCharPutNonBlocking(base, *pui8Buffer++);
    }
}



// data structures to hold GPIO PIN information 
struct gpio_pin_t {
  int name;
  int priority;
};

struct gpio_pin_t enables[] = {
  {  CTRL_K_VCCINT_PWR_EN, 1},
  {  CTRL_V_VCCINT_PWR_EN, 1},
  {  CTRL_VCC_1V8_PWR_EN,  2},
  {  CTRL_VCC_3V3_PWR_EN,  3},
  {  CTRL_V_MGTY1_VCCAUX_PWR_EN, 4},
  {  CTRL_V_MGTY2_VCCAUX_PWR_EN, 4},
  {  CTRL_K_MGTY_VCCAUX_PWR_EN,  4},
  {  CTRL_K_MGTH_VCCAUX_PWR_EN,  4},
  {  CTRL_V_MGTY1_AVCC_PWR_EN, 5},
  {  CTRL_V_MGTY2_AVCC_PWR_EN, 5},
  {  CTRL_K_MGTY_AVCC_PWR_EN,  5},
  {  CTRL_K_MGTH_AVCC_PWR_EN,  5},
  {  CTRL_K_MGTY_AVTT_PWR_EN,  6},
  {  CTRL_K_MGTH_AVTT_PWR_EN,  6},
  {  CTRL_V_MGTY1_AVTT_PWR_EN, 6},
  {  CTRL_V_MGTY2_AVTT_PWR_EN, 6}
};
const int nenables = sizeof(enables)/sizeof(enables[0]);

struct gpio_pin_t oks[] = {
  { K_VCCINT_PG_A, 1},
  { K_VCCINT_PG_B, 1},
  { V_VCCINT_PG_A, 1},
  { V_VCCINT_PG_B, 1},
  { VCC_1V8_PG,    2},
  { VCC_3V3_PG,    3},
  { V_MGTY1_AVCC_OK, 5},
  { V_MGTY2_AVCC_OK, 5},
  { K_MGTY_AVCC_OK,  5},
  { K_MGTH_AVCC_OK,  5},
  { K_MGTY_AVTT_OK,  6},
  { K_MGTH_AVTT_OK,  6},
  { V_MGTY1_AVTT_OK, 6},
  { V_MGTY2_AVTT_OK, 6}
};
const int noks = sizeof(oks)/sizeof(oks[0]);
const int num_priorities = 6;



// 
// check the power supplies and turn them on one by one
// 
bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2)
{
  bool success = true; // return value

  // data structure to turn on various power supplies. This should be ordered
  // such that the priority increases, though it's not necessary
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    for ( int e = 0; e < nenables; ++e ) {
      if ( enables[e].priority == prio )
        write_gpio_pin(enables[e].name, 0x1);
    }
    //
    // Delay for a bit
    //
    SysCtlDelay(g_ui32SysClock/6);
    // check power good at this level or higher priority (lower number)
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < noks; ++o ) {
      if ( enables[o].priority <= prio ) {
        int8_t val = read_gpio_pin(oks[o].name);
        if ( val == 0 ) {
          all_good = false;
          break;
        }
      }
    } // loop over 'ok' bits
    if (  ! all_good ) { 
      // o tells you which one died. should I print something on UART?
      // turn off all supplies at current priority level or lower
      // that is probably overkill since they should not all be 
      for ( int e = 0; e < nenables; ++e ) {
        if ( enables[e].priority >= prio ) 
          write_gpio_pin(enables[e].name, 0x0);
      }
      success = false;
      break;
    }
  } // loop over priorities

  return success;
  
}

bool
check_ps(void)
{
  bool success = true;
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < noks; ++o ) {
      if ( enables[o].priority <= prio ) {
        int8_t val = read_gpio_pin(oks[o].name);
        if ( val == 0 ) {
          all_good = false;
          break;
        }
      }
    } // loop over 'ok' bits
    if (  ! all_good ) { 
      // o tells you which one died. should I print something on UART?
      // turn off all supplies at current priority level or lower
      for ( int e = 0; e < nenables; ++e ) {
        if ( enables[e].priority >= prio ) {
          write_gpio_pin(enables[e].name, 0x0);
        }
        success = false;
        break;
      }
    }
  } // loop over priorities

  return success;
}

void SystemInit()
{
  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();

  //
  // Run from the PLL at 120 MHz.
  //
  g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
      SYSCTL_OSC_MAIN |
      SYSCTL_USE_PLL |
      SYSCTL_CFG_VCO_480), 120000000);
  UartInit(g_ui32SysClock);

  // SYSTICK timer
  MAP_SysTickPeriodSet(100); // period in Hz
  MAP_SysTickIntEnable(); // enable the interrupt
  MAP_SysTickEnable();
  return;
}

volatile uint32_t g_ui32SysTickCount;




// Holds the handle of the created queue.
static QueueHandle_t xLedQueue = NULL;

#define PS_BAD 0x03;
#define PS_GOOD 0x01;

// control the LED
void LedTask(void *parameters)
{
  uint32_t result;
  // this function never returns
  for ( ;; ) {
      // wait for a new item in the queue
      xQueueReceive(xLedQueue, &result, portMAX_DELAY);
      switch (result ) {
      case 0x00:
	break;
      case 0x01:
	// do something
	break;
      default:
	break;
      }

  }
}
// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool lastStateGood = false;

  // this function never returns
  for ( ;; ) {
      uint32_t message;
      bool good = check_ps();
      if ( ! good ) {
	  message = PS_BAD;
	  lastStateGood = false;
      }
      else if ( !lastStateGood ) { // good now, was bad
	  message = PS_GOOD;
	  lastStateGood = true;
      }
      xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  }
}

void CommandLineTask(void *parameters)
{
  //
  for ( ;; ) {
      ;
  }
}



// 
int main( void )
{
  // Set up the hardware ready to run the demo. 
  SystemInit();
  UARTSend(UART4_BASE, (const uint8_t*)"Starting\r\n",10 );

  // semaphore for the UART

  // queue for the LED
  xLedQueue = xQueueCreate(5, // The maximum number of items the queue can hold.
			   sizeof( uint32_t )); 		// The size of each item.


  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", 256, NULL, 5, NULL);
  xTaskCreate(LedTask,         "LED", 256, NULL, 2, NULL);
  xTaskCreate(CommandLineTask, "CON", 256, NULL, 1, NULL);

  // start the scheduler -- this function should not return
  vTaskStartScheduler();

  // should never get here 
  for( ;; );
}



//static void SetupHardware( void )
//{
//  SystemInit();// these are CMSIS names
//  SystemCoreClockUpdate();
//}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  /* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
     function will automatically get called if a task overflows its stack. */
  ( void ) pxTask;
  ( void ) pcTaskName;
  for( ;; );
}
/*-----------------------------------------------------------*/



