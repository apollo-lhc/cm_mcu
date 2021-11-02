#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "pinsel.h"

#ifdef DEBUG
extern void __error__(char *pcFilename, uint32_t ui32Line);
#endif // DEBUG

#if defined(REV1)
void pinsel(int pin, uint32_t * x_gpio_port, uint8_t * x_gpio_pin )
{
  uint8_t gpio_pin = -1;
  uint32_t gpio_port = -1;
  
  switch(pin) {
  case  TM4C_TO_VU7P_0 :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  TM4C_FROM_VU7P_0 :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  TM4C_DIP_SW_2 :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  F2_MGTY1_AVTT_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  F2_MGTY1_AVCC_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  F2_MGTY2_AVCC_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  F2_MGTY2_AVTT_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  F2_FPGA_PROGRAM :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  VCC_1V8_PG :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  VCC_3V3_PG :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  F2_VCCINT_PG_B :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  F2_VCCINT_PG_A :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_F2_VCCINT_PWR_EN :
    {
      gpio_port = GPIO_PORTA_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_VCC_1V8_PWR_EN :
    {
      gpio_port = GPIO_PORTA_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  _FPGA_I2C_RESET :
    {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  CTRL_VCC_3V3_PWR_EN :
    {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  CTRL_F1_VCCINT_PWR_EN :
    {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  _PWR_I2C_RESET :
    {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  TM4C_TP1 :
    {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  _CLOCKS_I2C_RESET :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  _F2_OPTICS_I2C_RESET :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  _F1_OPTICS_I2C_RESET :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  F1_VCCINT_PG_B :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  F1_VCCINT_PG_A :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  BLADE_POWER_OK :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  BLADE_POWER_EN :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  BLADE_ZYNQ_GPIO1 :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  BLADE_ZYNQ_GPIO2 :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  F1_MGTH_AVTT_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  F1_MGTH_AVCC_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  F1_MGTY_AVCC_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  F1_MGTY_AVTT_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  _F1_FPGA_DONE :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  F1_FPGA_PROGRAM :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  TM4C_DIP_SW_1 :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  CTRL_F1_MGTY_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  CTRL_F1_MGTH_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_F1_MGTY_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_F1_MGTH_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  TM4C_FROM_KU15P_0 :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  TM4C_TO_KU15P_0 :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  CTRL_F1_MGTY_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_F1_MGTH_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  TM4C_TP2 :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  TM4C_TP3 :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  ID_EEPROM_WP :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  CTRL_F2_MGTY2_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  CTRL_F2_MGTY1_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  CTRL_F2_MGTY2_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  CTRL_F2_MGTY1_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_F2_MGTY2_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_F2_MGTY1_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  MCU_LED_BLUE :
    {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  MCU_LED_GREEN :
    {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  MCU_LED_RED :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  _F2_FPGA_DONE :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  default:
    gpio_port = -1;
    gpio_pin  = -1;
#ifdef DEBUG
    __error__(__FILE__, __LINE__);
#endif // DEBUG
    break;
  }

  *x_gpio_port = gpio_port;
  *x_gpio_pin = gpio_pin;
}

const char *const pin_names[] = {
    "Dummy",                      // 0
    "VCC_M1V8",                   // 1
    "VCC_M3V3",                   // 2
    "VCC_2V5",                    // 3
    "VCC_12V",                    // 4
    "TM4C_TO_VU7P_0",             // 5
    "TM4C_FROM_VU7P_0",           // 6
    "VCC_M3V3",                   // 7
    "VCC_M3V3",                   // 8
    "VREF_2.5V",                  // 9
    "GND",                        // 10
    "TM4C_DIP_SW_2",              // 11
    "F2_MGTY1_AVTT",              // 12
    "F2_MGTY1_AVCC",              // 13
    "F2_MGTY1_VCCAUX",            // 14
    "F2_VCCINT",                  // 15
    "VCC_M3V3",                   // 16
    "GND",                        // 17
    "VCC_3V3",                    // 18
    "F2_MGTY2_VCCAUX",            // 19
    "F2_MGTY2_AVCC",              // 20
    "F2_MGTY2_AVTT",              // 21
    "F2_MGTY1_AVTT_OK",           // 22
    "F2_MGTY1_AVCC_OK",           // 23
    "F2_MGTY2_AVCC_OK",           // 24
    "F2_MGTY2_AVTT_OK",           // 25
    "VCC_M3V3",                   // 26
    "F2_FPGA_PROGRAM",            // 27
    "VCC_M3V3",                   // 28
    "VCC_1V8_PG",                 // 29
    "VCC_3V3_PG",                 // 30
    "F2_VCCINT_PG_B",             // 31
    "F2_VCCINT_PG_A",             // 32
    "BLADE_IPMC_TX",              // 33
    "BLADE_IPMC_RX",              // 34
    "FRONT_PANEL_UART_RX",        // 35
    "FRONT_PANEL_UART_TX",        // 36
    "CTRL_F2_VCCINT_PWR_EN",      // 37
    "CTRL_VCC_1V8_PWR_EN",        // 38
    "VCC_M3V3",                   // 39
    "FPGA_I2C_SCL",               // 40
    "FPGA_I2C_SDA",               // 41
    "_FPGA_I2C_RESET",            // 42
    "CTRL_VCC_3V3_PWR_EN",        // 43
    "CTRL_F1_VCCINT_PWR_EN",      // 44
    "_PWR_I2C_RESET",             // 45
    "TM4C_TP1",                   // 46
    "VCC_M3V3",                   // 47
    "GND",                        // 48
    "PWR_I2C_SCL",                // 49
    "PWR_I2C_SDA",                // 50
    "CLOCKS_I2C_SCL",             // 51
    "CLOCKS_I2C_SDA",             // 52
    "F2_OPTICS_I2C_SCL",          // 53
    "F2_OPTICS_I2C_SDA",          // 54
    "F1_OPTICS_I2C_SCL",          // 55
    "F1_OPTICS_I2C_SDA",          // 56
    "_CLOCKS_I2C_RESET",          // 57
    "_F2_OPTICS_I2C_RESET",       // 58
    "_F1_OPTICS_I2C_RESET",       // 59
    "F1_VCCINT_PG_B",             // 60
    "F1_VCCINT_PG_A",             // 61
    "BLADE_POWER_OK",             // 62
    "VCC_M3V3",                   // 63
    "NC",                         // "Tied to GND. This function is not used.",// 64
    "NC",                         // 65
    "Tied to GND. ",              // 66
    "NC",                         // 67
    "VCC_M3V3",                   // 68
    "VCC_M3V3",                   // 69
    "TM4C_ENABLE",                // 70
    "BLADE_POWER_EN",             // 71
    "BLADE_ZYNQ_GPIO1",           // 72
    "BLADE_ZYNQ_GPIO2",           // 73
    "F1_MGTH_AVTT_OK",            // 74
    "F1_MGTH_AVCC_OK",            // 75
    "F1_MGTY_AVCC_OK",            // 76
    "F1_MGTY_AVTT_OK",            // 77
    "_F1_FPGA_DONE",              // 78
    "VCC_M3V3",                   // 79
    "GND",                        // 80
    "F1_FPGA_PROGRAM",            // 81
    "TM4C_DIP_SW_1",              // 82
    "CTRL_F1_MGTY_VCCAUX_PWR_EN", // 83
    "CTRL_F1_MGTH_VCCAUX_PWR_EN", // 84
    "CTRL_F1_MGTY_AVCC_PWR_EN",   // 85
    "CTRL_F1_MGTH_AVCC_PWR_EN",   // 86
    "TM4C_VDDC",                  // 87
    "NC",       // Tied to GND. This design uses the 16 MHz internal oscillator PIOSC. 88
    "NC",       // 89
    "VCC_M3V3", // 90
    "BLADE_MASTER_I2C_SCL",        // 91
    "BLADE_MASTER_I2C_SDA",        // 92
    "TM4C_FROM_KU15P_0",           // 93
    "TM4C_TO_KU15P_0",             // 94
    "BLADE_ZYNQ_TX",               // 95
    "BLADE_ZYNQ_RX",               // 96
    "JTAG_TDO",                    // 97
    "JTAG_TDI",                    // 98
    "JTAG_TMS",                    // 99
    "JTAG_TCK",                    // 100
    "VCC_M3V3",                    // 101
    "CTRL_F1_MGTY_AVTT_PWR_EN",    // 102
    "CTRL_F1_MGTH_AVTT_PWR_EN",    // 103
    "TM4C_TP2",                    // 104
    "TM4C_TP3",                    // 105
    "ID_EEPROM_WP",                // 106
    "CTRL_F2_MGTY2_AVTT_PWR_EN",   // 107
    "CTRL_F2_MGTY1_AVTT_PWR_EN",   // 108
    "CTRL_F2_MGTY2_AVCC_PWR_EN",   // 109
    "CTRL_F2_MGTY1_AVCC_PWR_EN",   // 110
    "CTRL_F2_MGTY2_VCCAUX_PWR_EN", // 111
    "CTRL_F2_MGTY1_VCCAUX_PWR_EN", // 112
    "VCC_M3V3",                    // 113
    "GND",                         // 114
    "TM4C_VDDC",                   // 115
    "MCU_LED_BLUE",               // 116
    "MCU_LED_GREEN",              // 117
    "MCU_LED_RED",                // 118
    "_F2_FPGA_DONE",               // 119
    "F1_MGTH_AVTT",                // 120
    "F1_MGTH_AVCC",                // 121
    "VCC_M3V3",                    // 122
    "F1_MGTH_VCCAUX",              // 123
    "F1_VCCINT",                   // 124
    "VCC_1V8",                     // 125
    "F1_MGTY_VCCAUX",              // 126
    "F1_MGTY_AVCC",                // 127
    "F1_MGTY_AVTT"                 // 128
};

#elif defined(REV2)
void pinsel(int pin, uint32_t *x_gpio_port, uint8_t *x_gpio_pin)
{
  uint8_t gpio_pin = -1;
  uint32_t gpio_port = -1;

  switch (pin) {
    case _PWR_I2C_RESET: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case _CLOCKS_I2C_RESET: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case _F2_OPTICS_I2C_RESET: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case PG_4V0: {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin = GPIO_PIN_7;
      break;
    }
    case F2_FPGA_PROGRAM: {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin = GPIO_PIN_6;
      break;
    }
    case _F1_OPTICS_I2C_RESET: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case _FPGA_I2C_RESET: {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case BLADE_POWER_EN: {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case PG_3V3: {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case PG_1V8: {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case PG_F1_INT_A: {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case PG_F1_INT_B: {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case PG_F1_AVCC: {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case PG_F1_AVTT: {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case PG_F1_VCCAUX: {
      gpio_port = GPIO_PORTF_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case PG_F2_INT_A: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case PG_F2_INT_B: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_6;
      break;
    }
    case PG_F2_AVCC: {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin = GPIO_PIN_7;
      break;
    }
    case PG_F2_AVTT: {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin = GPIO_PIN_6;
      break;
    }
    case PG_F2_VCCAUX: {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case _F1_FPGA_DONE: {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case _F2_FPGA_DONE: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_7;
      break;
    }
    case F1_TO_MCU: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_6;
      break;
    }
    case F2_TO_MCU: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case MCU_TO_F1: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case MCU_TO_F2: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case BLADE_POWER_OK: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case EN_3V3: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case EN_1V8: {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case EN_F1_INT: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case EN_F1_AVCC: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case EN_F1_AVTT: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case EN_F1_VCCAUX: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case EN_F2_INT: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case EN_F2_AVCC: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case EN_F2_AVTT: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_7;
      break;
    }
    case EN_F2_VCCAUX: {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin = GPIO_PIN_6;
      break;
    }
    case FPGA_CFG_FROM_FLASH: {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case F1_FPGA_PROGRAM: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case MCU_LED_RED: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case MCU_LED_GREEN: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case MCU_LED_BLUE: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case ID_EEPROM_WP: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case JTAG_FROM_SM: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case _F1_JTAG_BYPASS: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_2;
      break;
    }
    case _F2_JTAG_BYPASS: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_3;
      break;
    }
    case SPARE_GPIO0: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_4;
      break;
    }
    case SPARE_GPIO1: {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin = GPIO_PIN_5;
      break;
    }
    case F1_C2C_OK: {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case F2_C2C_OK: {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    case _F1_INSTALLED: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_0;
      break;
    }
    case _F2_INSTALLED: {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin = GPIO_PIN_1;
      break;
    }
    default: {
      gpio_port = -1;
      gpio_pin = -1;
#ifdef DEBUG
      __error__(__FILE__, __LINE__);
#endif // DEBUG
      break;
    }
  }
  *x_gpio_port = gpio_port;
  *x_gpio_pin = gpio_pin;

}
const char *const pin_names[] = {
    "Dummy",                // 0
    "CUR_V_4V0",            // 1
    "CUR_V_M3V3",           // 2
    "CUR_V_12V",            // 3
    "V_F2_VCCAUX",          // 4
    "_I2C_RESET_PWR",       // 5
    "_I2C_RESET_CLOCKS",    // 6
    "V_M3V3",               // 7
    "V_M3V3",               // 8
    "MCU_VREFA",            // 9
    "GND",                  // 10
    "_I2C_RESET_F2_OPTICS", // 11
    "V_12V",                // 12
    "V_M3V3",               // 13
    "V_3V3",                // 14
    "V_4V0",                // 15
    "V_M3V3",               // 16
    "GND",                  // 17
    "CUR_F1_VCCAUX",        // 18
    "CUR_F2_VCCAUX",        // 19
    "F1_TEMP_DIODE",        // 20
    "F2_TEMP_DIODE",        // 21
    "PG_4V0",               // 22
    "F2_CFG_START",         // 23
    "SM_ZYNQ_GPIO2",        // 24
    "SM_ZYNQ_GPIO1",        // 25
    "V_M3V3",               // 26
    "_I2C_RESET_F1_OPTICS", // 27
    "V_M3V3",               // 28
    "_I2C_RESET_FPGAS",     // 29
    "SM_SOFT_PWR_EN",       // 30
    "PG_3V3",               // 31
    "PG_1V8",               // 32
    "SM_ZYNQ_TX",           // 33
    "SM_ZYNQ_RX",           // 34
    "SM_ZYNQ_I2C_SCL",      // 35
    "SM_ZYNQ_I2C_SDA",      // 36
    "SM_IPMC_TX",           // 37
    "SM_IPMC_RX",           // 38
    "V_M3V3",               // 39
    "SM_CPLD_GPIO0",        // 40
    "SM_CPLD_GPIO1",        // 41
    "PG_F1_INT_A",          // 42
    "PG_F1_INT_B",          // 43
    "PG_F1_AVCC",           // 44
    "PG_F1_AVTT",           // 45
    "PG_F1_VCCAUX",         // 46
    "V_M3V3",               // 47
    "GND",                  // 48
    "I2C_SCL_POWER",        // 49
    "I2C_SDA_POWER",        // 50
    "I2C_SCL_CLOCKS",       // 51
    "I2C_SDA_CLOCKS",       // 52
    "I2C_SCL_F2_OPTICS",    // 53
    "I2C_SDA_F2_OPTICS",    // 54
    "I2C_SCL_F1_OPTICS",    // 55
    "I2C_SDA_F1_OPTICS",    // 56
    "PG_F2_INT_A",          // 57
    "PG_F2_INT_B",          // 58
    "PG_F2_AVCC",           // 59
    "PG_F2_AVTT",           // 60
    "PG_F2_VCCAUX",         // 61
    "_F1_CFG_DONE",         // 62
    "V_M3V3",               // 63
    "GND",                  // 64
    "NC",                   // 65
    "XOSC0",                // 66
    "XOSC1",                // 67
    "V_M3V3",               // 68
    "V_M3V3",               // 69
    "MCU_ENABLED",          // 70
    "_F2_CFG_DONE",         // 71
    "F1_TO_MCU",            // 72
    "F2_TO_MCU",            // 73
    "MCU_TO_F1",            // 74
    "MCU_TO_F2",            // 75
    "CM_TO_SM_PWR_OK",      // 76
    "EN_3V3",               // 77
    "EN_1V8",               // 78
    "V_M3V3",               // 79
    "GND",                  // 80
    "EN_F1_INT",            // 81
    "EN_F1_AVCC",           // 82
    "EN_F1_AVTT",           // 83
    "EN_F1_VCCAUX",         // 84
    "EN_F2_INT",            // 85
    "EN_F2_AVCC",           // 86
    "TM4C_VDDC",            // 87
    "OSC0",                 // 88
    "OSC1",                 // 89
    "V_M3V3",               // 90
    "SM_IPMC_I2C_SCL",      // 91
    "SM_IPMC_I2C_SDA",      // 92
    "EN_F2_AVTT",           // 93
    "EN_F2_VCCAUX",         // 94
    "I2C_SCL_FPGAS",        // 95
    "I2C_SDA_FPGAS",        // 96
    "FP_MCU_TDO",           // 97
    "FP_MCU_TDI",           // 98
    "FP_MCU_TMS",           // 99
    "FP_MCU_TCK",           // 100
    "V_M3V3",               // 101
    "FPGA_CFG_FROM_FLASH",  // 102
    "F1_CFG_START",         // 103
    "MCU_LED_RED",          // 104
    "MCU_LED_GREEN",        // 105
    "MCU_LED_BLUE",         // 106
    "ID_EEPROM_WP",         // 107
    "JTAG_FROM_SM",         // 108
    "_F1_JTAG_BYPASS",      // 109
    "_F2_JTAG_BYPASS",      // 110
    "SPARE_GPIO0",          // 111
    "SPARE_GPIO1",          // 112
    "V_M3V3",               // 113
    "GND",                  // 114
    "TM4C_VDDC",            // 115
    "F1_C2C_OK",            // 116
    "F2_C2C_OK",            // 117
    "_F1_INSTALLED",        // 118
    "_F2_INSTALLED",        // 119
    "V_F2_AVTT",            // 120
    "V_F2_AVCC",            // 121
    "V_M3V3",               // 122
    "V_F2_INT",             // 123
    "V_F1_VCCAUX",          // 124
    "V_F1_AVTT",            // 125
    "V_F1_AVCC",            // 126
    "V_F1_INT",             // 127
    "V_1V8",                // 128
};


#else // no valid revision defined
#error "No board defined"
#endif

