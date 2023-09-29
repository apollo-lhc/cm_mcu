//*****************************************************************************
//
// FreeRTOS v1 -- wittich 3/2019
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
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/uart.h"

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
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  //
  // Loop while there are more characters to send.
  //
  while(ui32Count--)
    {
      //
      // Write the next character to the UART.
      //
      MAP_UARTCharPutNonBlocking(UART4_BASE, *pui8Buffer++);
    }
}

uint32_t g_ui32SysClock = 0;


// data structures to hold GPIO PIN information 
struct gpio_pin_t {
  int name;
  int priority;
};

struct gpio_pin_t enables[] = {
  {  CTRL_F1_VCCINT_PWR_EN, 1},
  {  CTRL_F2_VCCINT_PWR_EN, 1},
  {  CTRL_VCC_1V8_PWR_EN,  2},
  {  CTRL_VCC_3V3_PWR_EN,  3},
  {  CTRL_F2_MGTY1_VCCAUX_PWR_EN, 4},
  {  CTRL_F2_MGTY2_VCCAUX_PWR_EN, 4},
  {  CTRL_F1_MGTY_VCCAUX_PWR_EN,  4},
  {  CTRL_F1_MGTH_VCCAUX_PWR_EN,  4},
  {  CTRL_F2_MGTY1_AVCC_PWR_EN, 5},
  {  CTRL_F2_MGTY2_AVCC_PWR_EN, 5},
  {  CTRL_F1_MGTY_AVCC_PWR_EN,  5},
  {  CTRL_F1_MGTH_AVCC_PWR_EN,  5},
  {  CTRL_F1_MGTY_AVTT_PWR_EN,  6},
  {  CTRL_F1_MGTH_AVTT_PWR_EN,  6},
  {  CTRL_F2_MGTY1_AVTT_PWR_EN, 6},
  {  CTRL_F2_MGTY2_AVTT_PWR_EN, 6}
};
const int nenables = sizeof(enables)/sizeof(enables[0]);

struct gpio_pin_t oks[] = {
  { F1_VCCINT_PG_A, 1},
  { F1_VCCINT_PG_B, 1},
  { F2_VCCINT_PG_A, 1},
  { F2_VCCINT_PG_B, 1},
  { VCC_1V8_PG,    2},
  { VCC_3V3_PG,    3},
  { F2_MGTY1_AVCC_OK, 5},
  { F2_MGTY2_AVCC_OK, 5},
  { F1_MGTY_AVCC_OK,  5},
  { F1_MGTH_AVCC_OK,  5},
  { F1_MGTY_AVTT_OK,  6},
  { F1_MGTH_AVTT_OK,  6},
  { F2_MGTY1_AVTT_OK, 6},
  { F2_MGTY2_AVTT_OK, 6}
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
  g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 120000000);
  UartInit(g_ui32SysClock);
  return;
}

void SystemCoreClockUpdate()
{
  return;
}

/*
 * This is a simple example that creates two tasks and one queue.  One task
 * periodically sends a value to the other, which then prints out a message.
 * Normally such a simple example would toggle an LED, so the message that is
 * printed out is "toggle".
 *
 * The demo configures the kernel to be as simple as possible; FreeRTOSConfig.h
 * excludes most features, including dynamic memory allocation.
 */



/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
   to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( pdMS_TO_TICKS( 1000UL ) )

/* The number of items the queue can hold.  This is 1 as the receive task
   will remove items as they are added, meaning the send task should always find
   the queue empty. */
#define mainQUEUE_LENGTH_IN_ITEMS			( 1 )

/*-----------------------------------------------------------*/

/*
 * Configures the clocks ready to run the demo.
 */
static void prvSetupHardware( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*-----------------------------------------------------------*/


/* Holds the handle of the created queue. */
static QueueHandle_t xQueue = NULL;


/*-----------------------------------------------------------*/

int main( void )
{
  /* Set up the hardware ready to run the demo. */
  prvSetupHardware();
  UARTSend( (const uint8_t*)"Starting\r\n",10 );

  /* Create the queue.  xQueueCreateStatic() has two more parameters than the
     xQueueCreate() function.  The first new parameter is a pointer to the
     pre-allocated queue storage area.  The second new parameter is a pointer to
     the StaticQueue_t structure that will hold the queue state information in
     an anonymous way. */
  xQueue = xQueueCreate( mainQUEUE_LENGTH_IN_ITEMS, /* The maximum number of items the queue can hold. */
			 sizeof( uint32_t )); 		/* The size of each item. */

  /* Create the two tasks as described in the comments at the top of this
     file. */
  xTaskCreate(    	prvQueueReceiveTask, 			/* Function that implements the task. */
			"Rx",							/* Human readable name for the task. */
			configMINIMAL_STACK_SIZE,		/* Task's stack size, in words (not bytes!). */
			NULL,							/* Parameter to pass into the task. */
			mainQUEUE_RECEIVE_TASK_PRIORITY,/* The priority of the task. */
			NULL);

  xTaskCreate(  	prvQueueSendTask, 				/* Function that implements the task. */
			"Tx",							/* Human readable name for the task. */
			configMINIMAL_STACK_SIZE,		/* Task's stack size, in words (not bytes!). */
			NULL,							/* Parameter to pass into the task. */
			mainQUEUE_SEND_TASK_PRIORITY,	/* The priority of the task. */
			NULL);
  /* Start the scheduler. */
  vTaskStartScheduler();

  /* If dynamic memory allocation was used then the following code line would
     be reached if there was insufficient heap memory available to create either
     the timer or idle tasks.  As this project is using static memory allocation
     then the following line should never be reached. */
  for( ;; );
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  const unsigned long ulValueToSend = 100UL;

  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();

  for( ;; )
    {
      /* Place this task in the blocked state until it is time to run again.
	 The block time is specified in ticks, the constant used converts ticks
	 to ms.  While in the Blocked state this task will not consume any CPU
	 time. */
      vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

      /* Send to the queue - causing the queue receive task to unblock and
	 toggle the LED.  0 is used as the block time so the sending operation
	 will not block - it shouldn't need to block as the queue should always
	 be empty at this point in the code. */
      xQueueSend( xQueue, &ulValueToSend, 0U );
    }
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
  unsigned long ulReceivedValue;

  for( ;; )
    {
      /* Wait until something arrives in the queue - this task will block
	 indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
	 FreeRTOSConfig.h. */
      xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

      /*  To get here something must have been received from the queue, but
	  is it the expected value?  If it is, toggle the LED. */
      if( ulReceivedValue == 100UL )
	{
	  /* Output a string in lieu of using an LED. */
	  UARTSend((const uint8_t*)"Toggle!\r\n",9 );
	}
    }
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  SystemInit();// these are CMSIS names
  SystemCoreClockUpdate();
}
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



/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
   application must provide an implementation of vApplicationGetTimerTaskMemory()
   to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  /* If the buffers to be provided to the Timer task are declared inside this
     function then they must be declared static - otherwise they will be allocated on
     the stack and so not exists after this function exits. */
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

  /* Pass out a pointer to the StaticTask_t structure in which the Timer
     task's state will be stored. */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  /* Pass out the array that will be used as the Timer task's stack. */
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     Note that, as the array is necessarily of type StackType_t,
     configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/


