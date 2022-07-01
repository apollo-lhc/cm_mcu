#ifndef PINSEL_H
#define PINSEL_H

#include <stdint.h>
#ifndef PART_TM4C1290NCPDT
#define PART_TM4C1290NCPDT
#endif // PART_TM4C1290NCPDT

// data structures to hold GPIO PIN information
struct gpio_pin_t {
  uint8_t pin_number;
  char *name;
  int priority;
};

#if defined(REV1)
#define TM4C_TO_VU7P_0              5   // port, local_pin = Q 0
#define TM4C_FROM_VU7P_0            6   // port, local_pin = Q 1
#define TM4C_DIP_SW_2               11  // port, local_pin = Q 2
#define F2_MGTY1_AVTT_OK            22  // port, local_pin = C 7
#define F2_MGTY1_AVCC_OK            23  // port, local_pin = C 6
#define F2_MGTY2_AVCC_OK            24  // port, local_pin = C 5
#define F2_MGTY2_AVTT_OK            25  // port, local_pin = C 4
#define F2_FPGA_PROGRAM             27  // port, local_pin = Q 3
#define VCC_1V8_PG                  29  // port, local_pin = H 0
#define VCC_3V3_PG                  30  // port, local_pin = H 1
#define F2_VCCINT_PG_B              31  // port, local_pin = H 2
#define F2_VCCINT_PG_A              32  // port, local_pin = H 3
#define CTRL_F2_VCCINT_PWR_EN       37  // port, local_pin = A 4
#define CTRL_VCC_1V8_PWR_EN         38  // port, local_pin = A 5
#define _FPGA_I2C_RESET             42  // port, local_pin = F 0
#define CTRL_VCC_3V3_PWR_EN         43  // port, local_pin = F 1
#define CTRL_F1_VCCINT_PWR_EN       44  // port, local_pin = F 2
#define _PWR_I2C_RESET              45  // port, local_pin = F 3
#define TM4C_TP1                    46  // port, local_pin = F 4
#define _CLOCKS_I2C_RESET           57  // port, local_pin = Q 5
#define _F2_OPTICS_I2C_RESET        58  // port, local_pin = Q 6
#define _F1_OPTICS_I2C_RESET        59  // port, local_pin = K 7
#define F1_VCCINT_PG_B              60  // port, local_pin = K 6
#define F1_VCCINT_PG_A              61  // port, local_pin = K 5
#define BLADE_POWER_OK              62  // port, local_pin = K 4
#define BLADE_POWER_EN              71  // port, local_pin = M 7
#define BLADE_ZYNQ_GPIO1            72  // port, local_pin = M 6
#define BLADE_ZYNQ_GPIO2            73  // port, local_pin = M 5
#define F1_MGTH_AVTT_OK             74  // port, local_pin = M 4
#define F1_MGTH_AVCC_OK             75  // port, local_pin = M 3
#define F1_MGTY_AVCC_OK             76  // port, local_pin = M 2
#define F1_MGTY_AVTT_OK             77  // port, local_pin = M 1
#define _F1_FPGA_DONE               78  // port, local_pin = M 0
#define F1_FPGA_PROGRAM             81  // port, local_pin = L 0
#define TM4C_DIP_SW_1               82  // port, local_pin = L 1
#define CTRL_F1_MGTY_VCCAUX_PWR_EN  83  // port, local_pin = L 2
#define CTRL_F1_MGTH_VCCAUX_PWR_EN  84  // port, local_pin = L 3
#define CTRL_F1_MGTY_AVCC_PWR_EN    85  // port, local_pin = L 4
#define CTRL_F1_MGTH_AVCC_PWR_EN    86  // port, local_pin = L 5
#define TM4C_FROM_KU15P_0           93  // port, local_pin = L 7
#define TM4C_TO_KU15P_0             94  // port, local_pin = L 6
#define CTRL_F1_MGTY_AVTT_PWR_EN    102 // port, local_pin = Q 4
#define CTRL_F1_MGTH_AVTT_PWR_EN    103 // port, local_pin = P 2
#define TM4C_TP2                    104 // port, local_pin = P 3
#define TM4C_TP3                    105 // port, local_pin = P 4
#define ID_EEPROM_WP                106 // port, local_pin = P 5
#define CTRL_F2_MGTY2_AVTT_PWR_EN   107 // port, local_pin = N 0
#define CTRL_F2_MGTY1_AVTT_PWR_EN   108 // port, local_pin = N 1
#define CTRL_F2_MGTY2_AVCC_PWR_EN   109 // port, local_pin = N 2
#define CTRL_F2_MGTY1_AVCC_PWR_EN   110 // port, local_pin = N 3
#define CTRL_F2_MGTY2_VCCAUX_PWR_EN 111 // port, local_pin = N 4
#define CTRL_F2_MGTY1_VCCAUX_PWR_EN 112 // port, local_pin = N 5
#define MCU_LED_BLUE                116 // port, local_pin = J 0
#define MCU_LED_GREEN               117 // port, local_pin = J 1
#define MCU_LED_RED                 118 // port, local_pin = P 0
#define _F2_FPGA_DONE               119 // port, local_pin = P 1

#define isFPGAF1_PRESENT() (read_gpio_pin(TM4C_DIP_SW_1) == 1)
#define isFPGAF2_PRESENT() (read_gpio_pin(TM4C_DIP_SW_2) == 1)

#elif defined(REV2)

#define X(name, pin, port, localpin, input) \
  name = pin,
enum pins {
#include "gpio_pins.def"
};

#define isFPGAF1_PRESENT() (read_gpio_pin(_F1_INSTALLED) == 0)
#define isFPGAF2_PRESENT() (read_gpio_pin(_F2_INSTALLED) == 0)

#else
#error "Unknown board revision"
#endif

void pinsel(int pin, uint32_t *x_gpio_port, uint8_t *x_gpio_pin);

extern const char *const pin_names[];

#endif // PINSEL_H
