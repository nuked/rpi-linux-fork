/*
 *	pwm-bcm2708.c -- PWM driver for BCM2708/2835 (Raspberry Pi).
 *	Copyright (C) 2012-2013  Fred Barnes (University of Kent) and Omer Kilic (Erlang Solutions Ltd).
 *
 *	Author: Fred Barnes <F.R.M.Barnes@kent.ac.uk>
 *	Author: Omer Kilic <Omer.Kilic@erlang-solutions.com>
 *	License terms: GNU General Public Licence (GPL) version 2
 */

/*
 *	Using for reference, Broadcom BCM2835 ARM Peripherals datasheet, plus
 *	code by Frank Boss (for the PWM clock source).
 */

/*
 *	Note: this is fixed for the Raspberry Pi, in that we only expose one PWM
 *	controller (though the device has at least two), no support for FIFO
 *	stuff and bit-spread-PWM is disabled (e.g. for RC-servos).
 *
 *	The pin exposed on the Raspberry Pi that is PWM-able is GPIO18, P1-12 on
 *	the board header (rev 1&2).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

// #define DEBUG

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/of_device.h>

#include <mach/platform.h>
#include <mach/gpio.h>
#include <linux/gpio.h>


/* for the whole PWM controller */
struct bcm2708_pwm_chip {
	struct pwm_chip chip;
	void __iomem *pwm_base;
	void __iomem *clk_base;
};

/* for the exposed port (attached to pwm_device) */
struct bcm2708_pwm {
	int pin, pin_fcn;
	int clk_div;		/* 12-bit clock divisor */
	u32 range;		/* period in clocks */
	u32 data;		/* duty in clocks */
};

#define to_bcm2708_pwm_chip(_chip) \
	container_of(_chip, struct bcm2708_pwm_chip, chip)

#define RPI_PWM_GPIO_PIN	18
#define RPI_PWM_GPIO_PIN_FSEL	GPIO_FSEL_ALT5

/*{{{  offsets into the CLOCK area (CPRMAN_BASE)*/

/* XXX: defined in various places, not seen in datasheet yet */

// #define CPRMAN_BASE	(BCM2708_PERI_BASE + 0x101000)

#define PWMCLK_CNTL	160
#define PWMCLK_DIV	164	/* bits 23-12 */

#define PWMCLK_MAGIC	0x5a000000

#define PWMCLK_CNTL_STOP	0x00000001
#define PWMCLK_CNTL_START	0x00000011

#define PWMCLK_CNTL_BUSY	0x00000080


/*}}}*/
/*{{{  word offsets into the PWM area (PWM_BASE)*/

#define PWM_REG_CTL	0		/* control */
#define PWM_REG_STA	4		/* status */
#define PWM_REG_DMAC	8		/* DMA control */
#define PWM_REG_RNG1	16		/* channel 1 range */
#define PWM_REG_DAT1	20		/* channel 1 data */
#define PWM_REG_FIF1	24		/* FIFO */
#define PWM_REG_RNG2	32		/* channel 2 range */
#define PWM_REG_DAT2	36		/* channel 2 data */

/* control register bits */
#define PWM_CTL_MSEN2	0x8000		/* channel 2 m/s enable (0=PWM, 1=M/S) */
#define PWM_CTL_USEF2	0x2000		/* channel 2 uses FIFO */
#define PWM_CTL_POLA2	0x1000		/* channel 2 polarity (0=norm, 1=inv) */
#define PWM_CTL_SBIT2	0x0800		/* channel 2 silence bit */
#define PWM_CTL_RPTL2	0x0400		/* channel 2 FIFO repeat */
#define PWM_CTL_MODE2	0x0200		/* channel 2 mode (0=PWM, 1=serialiser) */
#define PWM_CTL_PWEN2	0x0100		/* channel 2 enable */
#define PWM_CTL_MSEN1	0x0080		/* channel 1 m/s enable (0=PWM, 1=M/S) */
#define PWM_CTL_CLRF1	0x0040		/* clear fifo by writing 1 */
#define PWM_CTL_USEF1	0x0020		/* channel 1 uses FIFO */
#define PWM_CTL_POLA1	0x0010		/* channel 1 polarity (0=norm, 1=inv) */
#define PWM_CTL_SBIT1	0x0008		/* channel 1 silence bit */
#define PWM_CTL_RPTL1	0x0004		/* channel 1 FIFO repeat */
#define PWM_CTL_MODE1	0x0002		/* channel 1 mode (0=PWM, 1=serialiser) */
#define PWM_CTL_PWEN1	0x0001		/* channel 1 enable */

/* status register bits */
#define PWM_STA_STA4	0x1000		/* channel 4 state */
#define PWM_STA_STA3	0x0800		/* channel 3 state */
#define PWM_STA_STA2	0x0400		/* channel 2 state */
#define PWM_STA_STA1	0x0200		/* channel 1 state */
#define PWM_STA_BERR	0x0100		/* bus error */
#define PWM_STA_GAPO4	0x0080		/* channel 4 gap occured */
#define PWM_STA_GAPO3	0x0040		/* channel 3 gap occured */
#define PWM_STA_GAPO2	0x0020		/* channel 2 gap occured */
#define PWM_STA_GAPO1	0x0010		/* channel 1 gap occured */
#define PWM_STA_RERR1	0x0008		/* fifo read error */
#define PWM_STA_WERR1	0x0004		/* fifo write error */
#define PWM_STA_EMPT1	0x0002		/* fifo empty */
#define PWM_STA_FULL1	0x0001		/* fifo full */

/* DMA control register bits (not used by this implementation) */
#define PWM_DMAC_ENAB	0x80000000	/* DMA enable */
#define PWM_DMAC_PANIC_MASK	0x0000ff00	/* DMA threshold for PANIC signal */
#define PWM_DMAC_PANIC_SHIFT	8
#define PWM_DMAC_DREQ_MASK	0x000000ff	/* DMA threshold for DREQ signal */
#define PWM_DMAC_DREQ_SHIFT	0

/* In PWM mode:
 *    RNG1/RNG2 defines the period (S).
 *    DAT1/DAT2 defines the duty-cycle (M).
 */

/*}}}*/


/*{{{  static int bcm2708_pwm_request (struct pwm_chip *chip, struct pwm_device *pwm)*/
/*
 *	called when the PWM device is requested (i.e. opened/etc.)
 */
static int bcm2708_pwm_request (struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2708_pwm *bc_pwm;
	int err = 0;

	bc_pwm = kzalloc (sizeof (struct bcm2708_pwm), GFP_KERNEL);
	if (!bc_pwm) {
		return -ENOMEM;
	}

	/* get the current GPIO alternative function, so we can put it back later */
	bc_pwm->pin = RPI_PWM_GPIO_PIN;
	bc_pwm->pin_fcn = bcm2708_gpio_platform_getfcn (bc_pwm->pin);
	/* setup some defaults */
	bc_pwm->clk_div = 1;		/* full-speed approx ~52ns ticks at 19.2 MHz */
	bc_pwm->range = 19200000;	/* 1 second */
	bc_pwm->data = 960000;		/* 0.5 seconds */

	if (bc_pwm->pin_fcn < 0) {
		/* means something went wrong */
		err = bc_pwm->pin_fcn;
		kfree (bc_pwm);
		bc_pwm = NULL;
	} else {
		/* bit grim: set this directly, since function not exported to modules */
		pwm->chip_data = bc_pwm;
		// pwm_set_chip_data (pwm, bc_pwm);
	}

	return err;
}
/*}}}*/
/*{{{  static void bcm2708_pwm_free (struct pwm_chip *chip, struct pwm_device *pwm)*/
/*
 *	called when the PWM device is released (i.e. closed)
 */
static void bcm2708_pwm_free (struct pwm_chip *chip, struct pwm_device *pwm)
{
	// struct bcm2708_pwm *bc_pwm = pwm_get_chip_data (pwm);
	struct bcm2708_pwm *bc_pwm = pwm->chip_data;

	if (bc_pwm) {
		/* restore previous pin state */
		int cstate = bcm2708_gpio_platform_getfcn (bc_pwm->pin);

		if (cstate != RPI_PWM_GPIO_PIN_FSEL) {
			dev_warn (chip->dev, "GPIO PWM pin function is now %d, so not resetting to %d\n", cstate, bc_pwm->pin_fcn);
		} else if (cstate != bc_pwm->pin_fcn) {
			/* put it back */
			bcm2708_gpio_platform_setfcn (bc_pwm->pin, bc_pwm->pin_fcn);
		}
		kfree (bc_pwm);
		bc_pwm = NULL;
	}
	return;
}
/*}}}*/
/*{{{  static int bcm2708_pwm_config (struct pwm_device *pwm, int duty_ns, int period_ns)*/
/*
 *	called to configure PWM for specified chip/device
 */
static int bcm2708_pwm_config (struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct bcm2708_pwm_chip *bc_chip = to_bcm2708_pwm_chip (chip);
	struct bcm2708_pwm *bc_pwm = pwm->chip_data;
	u64 act_period, tmp;

	/* note: kind of limited in what we can set these things too (integer nanosecond params on a 32-bit architecture) */
	dev_dbg (chip->dev, "configure!: duty_ns=%d, period_ns=%d\n", duty_ns, period_ns);
	dev_dbg (chip->dev, "chip pin is %d, old-fcn is %d\n", bc_pwm->pin, bc_pwm->pin_fcn);

	if (period_ns == 0) {
		/* hasn't been set yet */
		return 0;
	}
	/* got one or both */
	if (period_ns < 52) {
		dev_err (chip->dev, "cannot set period less than 52ns (19.2 MHz clock)\n");
		return -EINVAL;
	}
	bc_pwm->clk_div = 32;
	tmp = (u64)period_ns * (192 / bc_pwm->clk_div);
	do_div (tmp, 10000);
	bc_pwm->range = (u32)tmp;

	act_period = (u64)bc_pwm->range * 10000;
	do_div (act_period, (192 / bc_pwm->clk_div));

	/* Note: act_period <= period_ns, because of rounding */
	tmp = act_period * 100;
	do_div (tmp, (u32)period_ns);
	if (tmp < 99) {
		dev_warn (chip->dev, "more than 1%% inaccuracy in period (requested %d, actual %d)\n",
				period_ns, (int)act_period);
	}

	if (duty_ns == 0) {
		bc_pwm->data = 0;
	} else if (duty_ns < 52) {
		dev_err (chip->dev, "cannot set duty less than 52ns (19.2 MHz clock)\n");
		return -EINVAL;
	} else {
		u64 act_duty;

		tmp = (u64)duty_ns * (192 / bc_pwm->clk_div);
		do_div (tmp, 10000);
		bc_pwm->data = (u32)tmp;

		act_duty = (u64)bc_pwm->data * 10000;
		do_div (act_duty, (192 / bc_pwm->clk_div));

		/* Note: act_duty <= duty_ns, because of rounding */
		tmp = act_period * 100;
		do_div (tmp, (u32)duty_ns);
		if (tmp < 99) {
			dev_warn (chip->dev, "more than 1%% inaccuracy in duty-cycle (requested %d, actual %d)\n",
					duty_ns, (int)act_duty);
		}

		if (act_duty > act_period) {
			dev_err (chip->dev, "actual duty-cycle (%d) more than period (%d), requested (%d,%d)\n",
					(int)act_duty, (int)act_period, duty_ns, period_ns);
			return -EINVAL;
		}
	}

	dev_dbg (chip->dev, "configured, clk_div = %d, range = %u, data = %u\n", bc_pwm->clk_div, bc_pwm->range, bc_pwm->data);

	return 0;
}
/*}}}*/
/*{{{  static int bcm2708_pwm_enable (struct pwm_chip *chip, struct pwm_device *pwm)*/
/*
 *	called to enable specified PWM device
 */
static int bcm2708_pwm_enable (struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2708_pwm_chip *bc_chip = to_bcm2708_pwm_chip (chip);
	struct bcm2708_pwm *bc_pwm = pwm->chip_data;
	int cur_fcn, err = 0;
	int busy, waiting;
	u32 pwmreg;

	/* refresh the current state of the PWM pin */
	cur_fcn = bcm2708_gpio_platform_getfcn (bc_pwm->pin);
	if (cur_fcn < 0) {
		return -EINVAL;
	}

	bc_pwm->pin_fcn = cur_fcn;

	/* before anything else, make sure we're disabled already */
	pwmreg = readl (bc_chip->pwm_base + PWM_REG_CTL);
	pwmreg &= ~PWM_CTL_PWEN1;
	writel (pwmreg, bc_chip->pwm_base + PWM_REG_CTL);

	/* firstly, put the pin into PWM mode */
	err = bcm2708_gpio_platform_setfcn (bc_pwm->pin, RPI_PWM_GPIO_PIN_FSEL);
	if (err < 0) {
		dev_err (chip->dev, "failed to set pin %d mode to %d (error %d)\n", bc_pwm->pin, RPI_PWM_GPIO_PIN_FSEL, err);
		goto l_err;
	}
	udelay (110);

	/* then, configure PWM clock */
	/* FIXME: racey;  ought to have some infrastructure or lock for this */
	writel (PWMCLK_MAGIC | PWMCLK_CNTL_STOP, bc_chip->clk_base + PWMCLK_CNTL);		/* stop */
	udelay (110);

	waiting = 10000;
	do {
		busy = readl (bc_chip->clk_base + PWMCLK_CNTL) & PWMCLK_CNTL_BUSY;
		if (busy) {
			udelay (10);
			waiting--;
		}
	} while (busy && waiting);

	if (!waiting) {
		/* means we timed out (~100ms) */
		dev_err (chip->dev, "gave up waiting for PWM clock to stop being busy after 100ms.. (may be broken now)\n");
		err = -EIO;
		goto l_err;
	}

	writel (PWMCLK_MAGIC | ((bc_pwm->clk_div & 0xfff) << 12), bc_chip->clk_base + PWMCLK_DIV);
	writel (PWMCLK_MAGIC | PWMCLK_CNTL_START, bc_chip->clk_base + PWMCLK_CNTL);		/* start */

	/* next up, program range and data registers (code up top makes sure we're disabled) */
	writel (bc_pwm->range, bc_chip->pwm_base + PWM_REG_RNG1);
	udelay (20);
	writel (bc_pwm->range, bc_chip->pwm_base + PWM_REG_RNG2);
	udelay (20);
	writel (bc_pwm->data, bc_chip->pwm_base + PWM_REG_DAT1);

	/* put suitable settings into 'pwmreg' (currently PWM control word) */
	pwmreg |= PWM_CTL_MSEN1 | PWM_CTL_MSEN2;
	pwmreg &= ~(PWM_CTL_USEF1 | PWM_CTL_POLA1 | PWM_CTL_SBIT1 | PWM_CTL_RPTL1 | PWM_CTL_MODE1);
	pwmreg &= ~(PWM_CTL_USEF2 | PWM_CTL_POLA2 | PWM_CTL_SBIT2 | PWM_CTL_RPTL2 | PWM_CTL_MODE2);
	pwmreg |= PWM_CTL_PWEN1 | PWM_CTL_PWEN2;
	writel (pwmreg, bc_chip->pwm_base + PWM_REG_CTL);

	dev_dbg (chip->dev, "enabled!\n");
	err = 0;

l_err:
	return err;
}
/*}}}*/
/*{{{  static void bcm2708_pwm_disable (struct pwm_chip *chip, struct pwm_device *pwm)*/
/*
 *	called to disable the specified PWM device
 */
static void bcm2708_pwm_disable (struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2708_pwm_chip *bc_chip = to_bcm2708_pwm_chip (chip);
	struct bcm2708_pwm *bc_pwm = pwm->chip_data;
	u32 pwmreg;
	int err;

	/* stop the PWM generator */
	pwmreg = readl (bc_chip->pwm_base + PWM_REG_CTL);
	pwmreg &= ~PWM_CTL_PWEN1;
	writel (pwmreg, bc_chip->pwm_base + PWM_REG_CTL);

	/* then stop the PWM clock */
	/* FIXME: racey;  ought to have some infrastructure or lock for this */
	writel (PWMCLK_MAGIC | PWMCLK_CNTL_STOP, bc_chip->clk_base + PWMCLK_CNTL);		/* stop */

	/* finally, put the pin back into its previous mode */
	err = bcm2708_gpio_platform_setfcn (bc_pwm->pin, bc_pwm->pin_fcn);
	if (err < 0) {
		dev_err (chip->dev, "failed to set pin %d mode back to %d (error %d)\n", bc_pwm->pin, bc_pwm->pin_fcn, err);
	}

	dev_dbg (chip->dev, "disabled!\n");

	return;
}
/*}}}*/

/*{{{  bcm2708_pwm_ops (struct pwm_ops): PWM operations*/

static const struct pwm_ops bcm2708_pwm_ops = {
	.request = bcm2708_pwm_request,
	.free = bcm2708_pwm_free,
	.config = bcm2708_pwm_config,
	.enable = bcm2708_pwm_enable,
	.disable = bcm2708_pwm_disable,
	.owner = THIS_MODULE,
};

/*}}}*/



/*{{{  static int bcm2708_pwm_probe (struct platform_device *pdev)*/
/*
 *	Initialise the PWM device (basic).  Return 0 on success, non-zero on failure.
 */
static int bcm2708_pwm_probe (struct platform_device *pdev)
{
	struct bcm2708_pwm_chip *bc_chip;
	struct resource *res;
	int ret;

	printk (KERN_INFO"  bcm2708-pwm: bcm2708_pwm_probe(): here!\n");

	bc_chip = devm_kzalloc (&pdev->dev, sizeof (struct bcm2708_pwm_chip), GFP_KERNEL);
	if (!bc_chip) {
		dev_err (&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	res = platform_get_resource (pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err (&pdev->dev, "failed to get PWM IO-memory resource\n");
		return -EINVAL;
	}
	res = platform_get_resource (pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err (&pdev->dev, "failed to get CPRMAN IO-memory resource\n");
		return -EINVAL;
	}

	platform_set_drvdata (pdev, bc_chip);

	bc_chip->pwm_base = __io_address (PWM_BASE);
	bc_chip->clk_base = __io_address (CPRMAN_BASE);

	bc_chip->chip.dev = &pdev->dev;
	bc_chip->chip.ops = &bcm2708_pwm_ops;
	bc_chip->chip.base = -1;
	bc_chip->chip.npwm = 1;				/* though we have 2, only 1 visible on R-Pi */

	ret = pwmchip_add (&bc_chip->chip);
	if (ret < 0) {
		dev_err (&pdev->dev, "failed to add PWM chip, error %d\n", ret);
		return ret;
	}

	dev_dbg (&pdev->dev, "probe successful :)\n");

	return 0;
}
/*}}}*/
/*{{{  static int bcm2708_pwm_remove (struct platform_device *pdev)*/
/*
 *	Cleans up stuff here
 */
static int bcm2708_pwm_remove (struct platform_device *pdev)
{
	struct bcm2708_pwm_chip *bc_chip = platform_get_drvdata (pdev);

	return pwmchip_remove (&bc_chip->chip);
}
/*}}}*/

/*{{{  bcm2708_pwm_driver (struct platform_driver): platform driver structure*/

static struct platform_driver bcm2708_pwm_driver = {
	.driver = {
		.name = "bcm2708-pwm",
	},
	.probe = bcm2708_pwm_probe,
	.remove = bcm2708_pwm_remove,
};

module_platform_driver(bcm2708_pwm_driver);

/*}}}*/


MODULE_AUTHOR ("Fred Barnes <F.R.M.Barnes@kent.ac.uk>, Omer Kilic <Omer.Kilic@erlang-solutions.com>");
MODULE_DESCRIPTION ("PWM driver for BCM2708 (Raspberry Pi)");
MODULE_ALIAS ("platform:bcm2708-pwm");
MODULE_LICENSE ("GPL v2");


