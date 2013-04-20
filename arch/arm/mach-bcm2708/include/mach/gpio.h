/*
 * arch/arm/mach-bcm2708/include/mach/gpio.h
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#define ARCH_NR_GPIOS 54 // number of gpio lines

#define gpio_to_irq(x)	((x) + GPIO_IRQ_START)
#define irq_to_gpio(x)	((x) - GPIO_IRQ_START)

/* to allow other drivers to control alternate functions of the GPIO pins */
enum {
	GPIO_FSEL_INPUT = 0x00,
	GPIO_FSEL_OUTPUT = 0x01,
	GPIO_FSEL_ALT5 = 0x02,
	GPIO_FSEL_ALT4 = 0x03,
	GPIO_FSEL_ALT0 = 0x04,
	GPIO_FSEL_ALT1 = 0x05,
	GPIO_FSEL_ALT2 = 0x06,
	GPIO_FSEL_ALT3 = 0x07
};

/* FIXME: these rely on some global data in the GPIO driver */
extern int bcm2708_gpio_platform_setfcn (unsigned offset, int function);
extern int bcm2708_gpio_platform_getfcn (unsigned offset);


#endif

