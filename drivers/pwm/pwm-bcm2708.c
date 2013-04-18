/*
 *	pwm-bcm2708.c -- PWM driver for BCM2708/2835 (Raspberry Pi).
 *	Copyright (C) 2012-2013  Fred Barnes (University of Kent) and Omer Kilic (Erlang Solutions Ltd).
 *
 *	Author: Fred Barnes <F.R.M.Barnes@kent.ac.uk>
 *	Author: Omer Kilic <Omer.Kilic@erlang-solutions.com>
 *	License terms: GNU General Public Licence (GPL) version 2
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/of_device.h>

#include <mach/platform.h>

struct bcm2708_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;
};

#define to_bcm2708_pwm_chip(_chip) \
	container_of(_chip, struct bcm2708_pwm_chip, chip)


/*{{{  static int bcm2708_pwm_config (struct pwm_device *pwm, int duty_ns, int period_ns)*/
/*
 *	called to configure PWM for specified chip/device
 */
static int bcm2708_pwm_config (struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct bcm2708_pwm_chip *bc_chip = to_bcm2708_pwm_chip (chip);

	printk (KERN_INFO"  bcm2708-pwm: duty_ns=%d, period_ns=%d\n", duty_ns, period_ns);

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

	printk (KERN_INFO"  bcm2708-pwm: enable!\n");

	return 0;
}
/*}}}*/
/*{{{  static void bcm2708_pwm_disable (struct pwm_chip *chip, struct pwm_device *pwm)*/
/*
 *	called to disable the specified PWM device
 */
static void bcm2708_pwm_disable (struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2708_pwm_chip *bc_chip = to_bcm2708_pwm_chip (chip);

	printk (KERN_INFO"  bcm2708-pwm: disable!\n");

	return;
}
/*}}}*/

/*{{{  bcm2708_pwm_ops (struct pwm_ops): PWM operations*/

static const struct pwm_ops bcm2708_pwm_ops = {
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
		dev_err (&pdev->dev, "failed to get IO-memory resource\n");
		return -EINVAL;
	}

	platform_set_drvdata (pdev, bc_chip);

	bc_chip->base = __io_address (PWM_BASE);
	bc_chip->chip.dev = &pdev->dev;
	bc_chip->chip.ops = &bcm2708_pwm_ops;
	bc_chip->chip.base = -1;
	bc_chip->chip.npwm = 1;

	ret = pwmchip_add (&bc_chip->chip);
	if (ret < 0) {
		dev_err (&pdev->dev, "failed to add PWM chip, error %d\n", ret);
		return ret;
	}

	printk (KERN_INFO"  bcm2708-pwm: probe successful\n");

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


