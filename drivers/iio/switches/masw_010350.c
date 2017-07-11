/*
 * masw_010350 RF switch
 *
 * Copyright 2015 GR UPM
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


struct masw_010350_platform_data{
	int gpios[3];
};

struct masw_010350_state {
	struct masw_010350_platform_data* pdata;
	int gpio_states[3];
	int selected;
};

	

static ssize_t masw_010350_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct masw_010350_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;
	if(((u32)private)!=0)
		return   -EINVAL;
	if(readin<0 || readin >4)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	st->selected =readin;
	switch (st->selected){
		case 1:
			st->gpio_states[0]=1;
			st->gpio_states[1]=0;
			st->gpio_states[2]=0;
			break;
		case 2:
			st->gpio_states[0]=0;
			st->gpio_states[1]=1;
			st->gpio_states[2]=0;
			break;
		case 3:
			st->gpio_states[0]=1;
			st->gpio_states[1]=1;
			st->gpio_states[2]=0;
			break;
		case 4:
			st->gpio_states[0]=0;
			st->gpio_states[1]=0;
			st->gpio_states[2]=1;
			break;
		default:
			st->gpio_states[0]=1;
			st->gpio_states[1]=1;
			st->gpio_states[2]=1;
			break;
	}

	gpio_set_value(st->pdata->gpios[0],st->gpio_states[0]);
	gpio_set_value(st->pdata->gpios[1],st->gpio_states[1]);
	gpio_set_value(st->pdata->gpios[2],st->gpio_states[2]);

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}


static ssize_t masw_010350_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct masw_010350_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;
	if(((u32)private)!=0)
			return   -EINVAL;
	val = st->selected;
	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}



#define _MASW_010350_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = masw_010350_read, \
	.write = masw_010350_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info masw_010350_ext_info[] = {
	_MASW_010350_EXT_INFO("selected", 0),
	{ },
};

static const struct iio_chan_spec masw_010350_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.ext_info = masw_010350_ext_info,
};

static const struct iio_info masw010350_info = {
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static struct masw_010350_platform_data *masw_010350_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct masw_010350_platform_data *pdata;
	int i;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}
	for(i=0; i<3; i++){
		pdata->gpios[i] = of_get_gpio(np, i);
	}

	return pdata;
}
#else
static
struct masw_010350_platform_data *masw_010350_parse_dt(struct device *dev)
{
	struct masw_010350_platform_data *pdata;
	pdata -> gpios[0]=-1;
	pdata -> gpios[1]=-1;
	pdata -> gpios[2]=-1;
	return pdata;
}
#endif

static int masw_010350_probe(struct platform_device *pdev)
{
	struct masw_010350_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct masw_010350_state *st;
	int ret;
	int i;
	dev_err(&pdev->dev, "PROBING MASW...\n");
	if (&pdev->dev.of_node) {
		pdata = masw_010350_parse_dt(&pdev->dev);
		if (pdata == NULL)
			dev_err(&pdev->dev, "fail to get pdata\n");
			return -EINVAL;
	} else {
			dev_err(&pdev->dev, "No OF node\n");
			return -EINVAL;
	}

	if(pdata -> gpios[0] < 0 || pdata -> gpios[1] <0 || pdata -> gpios[2]<0){
		dev_err(&pdev->dev, "Bad pdata gpios %d %d %d\n",pdata -> gpios[0],pdata -> gpios[1],pdata -> gpios[2]);
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret =  -ENOMEM;
		dev_err(&pdev->dev, "fail to allocate indio_dev\n");
		return ret;
	}

	st = iio_priv(indio_dev);
	platform_set_drvdata(pdev,indio_dev);
	for(i=0;i<3;i++){
		if (gpio_is_valid(pdata->gpios[i])) {
		ret = devm_gpio_request(&pdev->dev, pdata->gpios[i],
					indio_dev->name);
		if (ret) {
			dev_err(&pdev->dev, "fail to request GPIO %d for V%d",
				pdata->gpios[i],i);
			return ret;
		}
		st->gpio_states[i]=1;
		gpio_direction_output(pdata->gpios[i],st->gpio_states[i]);
		}
	}

	return ret;
}

static int masw_010350_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id masw_010350_dt_match[] = {
	{ .compatible = "macom,masw_010350" },
	{},
};
MODULE_DEVICE_TABLE(of, masw_010350_dt_match);
#endif

static struct platform_driver masw_010350_driver = {
	.driver = {
		.name	= "masw010350",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(masw_010350_dt_match),
	},
	.probe		= masw_010350_probe,
	.remove		= masw_010350_remove,
};



module_platform_driver(masw_010350_driver);

MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_DESCRIPTION("MACOM MASW_010350 RF SWITCH");
MODULE_LICENSE("GPL v2");

