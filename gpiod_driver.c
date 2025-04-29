#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>


static struct gpio_desc *button_gpio;
static unsigned int irq_number;
static ktime_t last_time;
static volatile s64 last_period_ns;  //nanoseconds

// sysfs attribute to show last measured period
static ssize_t speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Last year code resets to 0 when idle but idt we have to do that 
    return sprintf(buf, "%lld\n", last_period_ns);
}

static DEVICE_ATTR_RO(speed);

// Interrupt handler
static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    ktime_t now = ktime_get();
    last_period_ns = ktime_to_ns(ktime_sub(now, last_time));
    last_time = now;
    pr_info("gpiod_driver: Detected encoder pulse, period = %lld ns\n", last_period_ns);
    return IRQ_HANDLED;
}

// Probe function
static int encoder_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;

    dev_info(dev, "gpiod_driver: Probing device...\n");

    // Get GPIO from device tree
    button_gpio = devm_gpiod_get(dev, "button", GPIOD_IN);
    if (IS_ERR(button_gpio)) {
        dev_err(dev, "gpiod_driver: Failed to get button GPIO\n");
        return PTR_ERR(button_gpio);
    }

    // Get IRQ number from GPIO
    irq_number = gpiod_to_irq(button_gpio);
    if (irq_number < 0) {
        dev_err(dev, "gpiod_driver: Failed to get IRQ number\n");
        return irq_number;
    }

    // Request IRQ
    // Use IRQF_TRIGGER_FALLING to detect falling edge of the signal
    ret = devm_request_irq(dev, irq_number, button_irq_handler, IRQF_TRIGGER_FALLING, "encoder_irq", NULL);
    if (ret) {
        dev_err(dev, "gpiod_driver: Failed to request IRQ\n");
        return ret;
    }

    // Initialize last_time to current time
    last_time = ktime_get();

    // Create sysfs entry for speed
    ret = device_create_file(dev, &dev_attr_speed);
    if (ret) {
        dev_err(dev, "gpiod_driver: Failed to create sysfs entry\n");
        return ret;
    }

    dev_info(dev, "gpiod_driver: Probe successful\n");
    return 0;
}

// Remove function
static void encoder_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    device_remove_file(dev, &dev_attr_speed);
    free_irq(irq_number, NULL);

    dev_info(dev, "gpiod_driver: Driver removed\n");
}

// Device tree match table
static const struct of_device_id encoder_of_match[] = {
    { .compatible = "fun_overlay" },
    {},
};
MODULE_DEVICE_TABLE(of, encoder_of_match);

// Platform driver structure
static struct platform_driver encoder_driver = {
    .probe = encoder_probe,
    .remove = encoder_remove,
    .driver = {
        .name = "slay_device_driver",
        .of_match_table = encoder_of_match,
        .owner = THIS_MODULE,
    },
};

module_platform_driver(encoder_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ELEC 424 Team 15");
MODULE_DESCRIPTION("Driver for RC Car");

