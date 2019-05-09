#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "pinsel.h"

extern void __error__(char *pcFilename, uint32_t ui32Line);


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
    __error__(__FILE__, __LINE__);
    break;
  }

  *x_gpio_port = gpio_port;
  *x_gpio_pin = gpio_pin;
}
