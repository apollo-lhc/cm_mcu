#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "pinsel.h"

#ifdef DEBUG
extern void __error__(char *pcFilename, uint32_t ui32Line);
#endif // DEBUG

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
  case  V_MGTY1_AVTT_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  V_MGTY1_AVCC_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  V_MGTY2_AVCC_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  V_MGTY2_AVTT_OK :
    {
      gpio_port = GPIO_PORTC_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  V_FPGA_PROGRAM :
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
  case  V_VCCINT_PG_B :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  V_VCCINT_PG_A :
    {
      gpio_port = GPIO_PORTH_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_V_VCCINT_PWR_EN :
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
  case  CTRL_K_VCCINT_PWR_EN :
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
  case  _V_OPTICS_I2C_RESET :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  _K_OPTICS_I2C_RESET :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_7;
      break;
    }
  case  K_VCCINT_PG_B :
    {
      gpio_port = GPIO_PORTK_BASE;
      gpio_pin  = GPIO_PIN_6;
      break;
    }
  case  K_VCCINT_PG_A :
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
  case  BLADE_ZYNQ_GPIO0 :
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
  case  K_MGTH_AVTT_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  K_MGTH_AVCC_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  K_MGTY_AVCC_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  K_MGTY_AVTT_OK :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  _K_FPGA_DONE :
    {
      gpio_port = GPIO_PORTM_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  K_FPGA_PROGRAM :
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
  case  CTRL_K_MGTY_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  CTRL_K_MGTH_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_K_MGTY_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTL_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_K_MGTH_AVCC_PWR_EN :
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
  case  CTRL_K_MGTY_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTQ_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_K_MGTH_AVTT_PWR_EN :
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
  case  CTRL_V_MGTY2_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  CTRL_V_MGTY1_AVTT_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  CTRL_V_MGTY2_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_2;
      break;
    }
  case  CTRL_V_MGTY1_AVCC_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_3;
      break;
    }
  case  CTRL_V_MGTY2_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_4;
      break;
    }
  case  CTRL_V_MGTY1_VCCAUX_PWR_EN :
    {
      gpio_port = GPIO_PORTN_BASE;
      gpio_pin  = GPIO_PIN_5;
      break;
    }
  case  TM4C_LED_BLUE :
    {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  TM4C_LED_GREEN :
    {
      gpio_port = GPIO_PORTJ_BASE;
      gpio_pin  = GPIO_PIN_1;
      break;
    }
  case  TM4C_LED_RED :
    {
      gpio_port = GPIO_PORTP_BASE;
      gpio_pin  = GPIO_PIN_0;
      break;
    }
  case  _V_FPGA_DONE :
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

const char* const pin_names[] =
{
    "VCC_M1V8",// 1
    "VCC_M3V3",// 2
    "VCC_2V5",// 3
    "VCC_12V",// 4
    "TM4C_TO_VU7P_0",// 5
    "TM4C_FROM_VU7P_0",// 6
    "VCC_M3V3",// 7
    "VCC_M3V3",// 8
    "VREF_2.5V",// 9
    "GND",// 10
    "TM4C_DIP_SW_2",// 11
    "V_MGTY1_AVTT",// 12
    "V_MGTY1_AVCC",// 13
    "V_MGTY1_VCCAUX",// 14
    "V_VCCINT",// 15
    "VCC_M3V3",// 16
    "GND",// 17
    "VCC_3V3",// 18
    "V_MGTY2_VCCAUX",// 19
    "V_MGTY2_AVCC",// 20
    "V_MGTY2_AVTT",// 21
    "V_MGTY1_AVTT_OK",// 22
    "V_MGTY1_AVCC_OK",// 23
    "V_MGTY2_AVCC_OK",// 24
    "V_MGTY2_AVTT_OK",// 25
    "VCC_M3V3",// 26
    "V_FPGA_PROGRAM",// 27
    "VCC_M3V3",// 28
    "VCC_1V8_PG",// 29
    "VCC_3V3_PG",// 30
    "V_VCCINT_PG_B",// 31
    "V_VCCINT_PG_A",// 32
    "BLADE_IPMC_TX",// 33
    "BLADE_IPMC_RX",// 34
    "FRONT_PANEL_UART_RX",// 35
    "FRONT_PANEL_UART_TX",// 36
    "CTRL_V_VCCINT_PWR_EN",// 37
    "CTRL_VCC_1V8_PWR_EN",// 38
    "VCC_M3V3",// 39
    "FPGA_I2C_SCL",// 40
    "FPGA_I2C_SDA",// 41
    "_FPGA_I2C_RESET",// 42
    "CTRL_VCC_3V3_PWR_EN",// 43
    "CTRL_K_VCCINT_PWR_EN",// 44
    "_PWR_I2C_RESET",// 45
    "TM4C_TP1",// 46
    "VCC_M3V3",// 47
    "GND",// 48
    "PWR_I2C_SCL",// 49
    "PWR_I2C_SDA",// 50
    "CLOCKS_I2C_SCL",// 51
    "CLOCKS_I2C_SDA",// 52
    "V_OPTICS_I2C_SCL",// 53
    "V_OPTICS_I2C_SDA",// 54
    "K_OPTICS_I2C_SCL",// 55
    "K_OPTICS_I2C_SDA",// 56
    "_CLOCKS_I2C_RESET",// 57
    "_V_OPTICS_I2C_RESET",// 58
    "_K_OPTICS_I2C_RESET",// 59
    "K_VCCINT_PG_B",// 60
    "K_VCCINT_PG_A",// 61
    "BLADE_POWER_OK",// 62
    "VCC_M3V3",// 63
    "Tied to GND. This function is not used.",// 64
    "No connect",// 65
    "Tied to GND. ",// 66
    "No connect",// 67
    "VCC_M3V3",// 68
    "VCC_M3V3",// 69
    "TM4C_ENABLE",// 70
    "BLADE_ZYNQ_GPIO0",// 71
    "BLADE_ZYNQ_GPIO1",// 72
    "BLADE_ZYNQ_GPIO2",// 73
    "K_MGTH_AVTT_OK",// 74
    "K_MGTH_AVCC_OK",// 75
    "K_MGTY_AVCC_OK",// 76
    "K_MGTY_AVTT_OK",// 77
    "_K_FPGA_DONE",// 78
    "VCC_M3V3",// 79
    "GND",// 80
    "K_FPGA_PROGRAM",// 81
    "TM4C_DIP_SW_1",// 82
    "CTRL_K_MGTY_VCCAUX_PWR_EN",// 83
    "CTRL_K_MGTH_VCCAUX_PWR_EN",// 84
    "CTRL_K_MGTY_AVCC_PWR_EN",// 85
    "CTRL_K_MGTH_AVCC_PWR_EN",// 86
    "TM4C_VDDC",// 87
    "TIed to GND. This design uses the 16 MHz internal oscillator PIOSC.",// 88
    "No connect",// 89
    "VCC_M3V3",// 90
    "BLADE_MASTER_I2C_SCL",// 91
    "BLADE_MASTER_I2C_SDA",// 92
    "TM4C_FROM_KU15P_0",// 93
    "TM4C_TO_KU15P_0",// 94
    "BLADE_ZYNQ_TX",// 95
    "BLADE_ZYNQ_RX",// 96
    "JTAG_TDO",// 97
    "JTAG_TDI",// 98
    "JTAG_TMS",// 99
    "JTAG_TCK",// 100
    "VCC_M3V3",// 101
    "CTRL_K_MGTY_AVTT_PWR_EN",// 102
    "CTRL_K_MGTH_AVTT_PWR_EN",// 103
    "TM4C_TP2",// 104
    "TM4C_TP3",// 105
    "ID_EEPROM_WP",// 106
    "CTRL_V_MGTY2_AVTT_PWR_EN",// 107
    "CTRL_V_MGTY1_AVTT_PWR_EN",// 108
    "CTRL_V_MGTY2_AVCC_PWR_EN",// 109
    "CTRL_V_MGTY1_AVCC_PWR_EN",// 110
    "CTRL_V_MGTY2_VCCAUX_PWR_EN",// 111
    "CTRL_V_MGTY1_VCCAUX_PWR_EN",// 112
    "VCC_M3V3",// 113
    "GND",// 114
    "TM4C_VDDC",// 115
    "TM4C_LED_BLUE",// 116
    "TM4C_LED_GREEN",// 117
    "TM4C_LED_RED",// 118
    "_V_FPGA_DONE",// 119
    "K_MGTH_AVTT",// 120
    "K_MGTH_AVCC",// 121
    "VCC_M3V3",// 122
    "K_MGTH_VCCAUX",// 123
    "K_VCCINT",// 124
    "VCC_1V8",// 125
    "K_MGTY_VCCAUX",// 126
    "K_MGTY_AVCC",// 127
    "K_MGTY_AVTT"// 128
};
