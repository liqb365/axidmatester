// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/task.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/workqueue.h>
#include <asm/checksum.h>

#define DRIVER_NAME "axidmatester"
#define BD_COUNT 13

#define BUFFER_SIZE 16384
#define BUFFER_CHECKSUM_SIZE 2
#define BUFFER_DATA_SIZE (BUFFER_SIZE - BUFFER_CHECKSUM_SIZE)

/**
 * struct driver_data
 */
struct driver_data {
	struct work_struct *work;
	struct workqueue_struct *workqueue;
	struct dma_chan *tx_chan;
	struct dma_chan *rx_chan;
	u8 **srcs;
	u8 **dsts;
	int counter;
	struct platform_device *pdev;
};

/* Global data. */
struct driver_data gd;

/* struct buffer_format */
struct buffer_format {
	u8 random_data[BUFFER_DATA_SIZE];
	u16 checksum;
} __packed;

/**
 * slave_tx_callback
 */
static void slave_tx_callback(void *completion)
{
	complete(completion);
}

/**
 * slave_rx_callback
 */
static void slave_rx_callback(void *completion)
{
	complete(completion);
}

/**
 * work_func
 */
static void work_func(struct work_struct *work)
{
	struct dma_device *tx_dev = gd.tx_chan->device;
	struct dma_device *rx_dev = gd.rx_chan->device;
	struct scatterlist tx_sg[BD_COUNT];
	struct scatterlist rx_sg[BD_COUNT];
	int i;
	dma_addr_t dma_srcs[BD_COUNT];
	dma_addr_t dma_dsts[BD_COUNT];
	struct dma_async_tx_descriptor *txd = NULL;
	struct dma_async_tx_descriptor *rxd = NULL;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct completion rx_cmp;
	struct completion tx_cmp;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;
	unsigned long rx_tmo = msecs_to_jiffies(300000);
	unsigned long tx_tmo = msecs_to_jiffies(30000);
	enum dma_status status;
	u16 csum;
	struct buffer_format *buffer_format;
	int pass = 0, fail = 0;

	/* Write content to source buffers. */
	for (i = 0; i < BD_COUNT; i++) {
		buffer_format = (struct buffer_format *)gd.srcs[i];
		/* Fill in content. */
		get_random_bytes(buffer_format->random_data, BUFFER_DATA_SIZE);
		/* Calculate checksum. */
		csum = ip_compute_csum(buffer_format->random_data, BUFFER_DATA_SIZE);
		/* Write checksum to source buffer. */
		buffer_format->checksum = csum;
	}

	/* Set source DMA addresses as NULL. */
	memset(dma_srcs, 0, sizeof(dma_srcs));

	/* Map the source buffers from virtual address space to bus address space. */
	for (i = 0; i < BD_COUNT; i++) {
		dma_srcs[i] = dma_map_single(tx_dev->dev, gd.srcs[i],
									BUFFER_SIZE, DMA_MEM_TO_DEV);
		if (dma_mapping_error(tx_dev->dev, dma_srcs[i])) {
			dev_err(&gd.pdev->dev, "Map source buffer (%d) failed!\n", i);
			goto unmap_source;
		}
	}

	/* Map the destination buffers from virtual address space to bus address space. */
	for (i = 0; i < BD_COUNT; i++) {
		dma_dsts[i] = dma_map_single(rx_dev->dev, gd.dsts[i],
									BUFFER_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(rx_dev->dev, dma_dsts[i])) {
			dev_err(&gd.pdev->dev, "Map destination buffer (%d) failed!\n", i);
			goto unmap_destination;
		}
	}

	/* Init sglist. */
	sg_init_table(tx_sg, BD_COUNT);
	sg_init_table(rx_sg, BD_COUNT);

	for (i = 0; i < BD_COUNT; i++) {
		sg_dma_address(&tx_sg[i]) = dma_srcs[i];
		sg_dma_address(&rx_sg[i]) = dma_dsts[i];

		sg_dma_len(&tx_sg[i]) = BUFFER_SIZE;
		sg_dma_len(&rx_sg[i]) = BUFFER_SIZE;
	}

	/* Prepare slave sg (Dev -> Mem). */
	rxd = rx_dev->device_prep_slave_sg(gd.rx_chan, rx_sg, BD_COUNT,
									DMA_DEV_TO_MEM, flags, NULL);
	if (!rxd) {
		dev_err(&gd.pdev->dev, "Prepare slave sg (Dev -> Mem) failed!\n");
		goto unmap_destination;
	}

	/* Prepare slave sg (Mem -> Dev). */
	txd = rx_dev->device_prep_slave_sg(gd.tx_chan, tx_sg, BD_COUNT,
									DMA_MEM_TO_DEV, flags, NULL);
	if (!txd) {
		dev_err(&gd.pdev->dev, "Prepare slave sg (Mem -> Dev) failed!\n");
		goto unmap_destination;
	}

	/* Init rxd. */
	init_completion(&rx_cmp);
	rxd->callback_param = &rx_cmp;
	rxd->callback = slave_rx_callback;

	/* Submit rxd. */
	rx_cookie = rxd->tx_submit(rxd);
	if (dma_submit_error(rx_cookie)) {
		dev_err(&gd.pdev->dev, "Submit rxd failed!\n");
		goto unmap_destination;
	}

	/* Init txd. */
	init_completion(&tx_cmp);
	txd->callback_param = &tx_cmp;
	txd->callback = slave_tx_callback;

	/* Submit txd. */
	tx_cookie = rxd->tx_submit(txd);
	if (dma_submit_error(tx_cookie)) {
		dev_err(&gd.pdev->dev, "Submit txd failed!\n");
		goto unmap_destination;
	}

	/* Flush pending transactions to HW. */
	dma_async_issue_pending(gd.rx_chan);
	dma_async_issue_pending(gd.tx_chan);

	/* Wait for tx_cmp completion. */
	tx_tmo = wait_for_completion_timeout(&tx_cmp, tx_tmo);
	if (tx_tmo == 0) {
		dev_err(&gd.pdev->dev, "Wait for tx_cmp completion failed!\n");
		goto unmap_destination;
	}

	/* Poll for tx transaction completion. */
	status = dma_async_is_tx_complete(gd.tx_chan, tx_cookie,
									NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(&gd.pdev->dev, "tx got completion callback but status is \'%s\'!\n",
									status == DMA_ERROR ? "error" : "in progress");
		goto unmap_destination;
	}

	/* Wait for rx_cmp completion. */
	rx_tmo = wait_for_completion_timeout(&rx_cmp, rx_tmo);
	if (rx_tmo == 0) {
		dev_err(&gd.pdev->dev, "Wait for rx_cmp completion failed!\n");
		goto unmap_destination;
	}

	/* Poll for rx transaction completion. */
	status = dma_async_is_tx_complete(gd.rx_chan, rx_cookie,
									NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(&gd.pdev->dev, "tx got completion callback but status is \'%s\'!\n",
									status == DMA_ERROR ? "error" : "in progress");
		goto unmap_destination;
	}

	/* Check transaction result. */
	for (i = 0; i < BD_COUNT; i++) {
		buffer_format = (struct buffer_format *)gd.dsts[i];
		/* Calculate checksum. */
		csum = ip_compute_csum(buffer_format->random_data, BUFFER_DATA_SIZE);
		if (csum != buffer_format->checksum) {
			pr_err("No.%d buffer checking failed! calculated checksum: %x buffer checksum: %x\n",
							i, csum, buffer_format->checksum);
			fail++;
		} else
			pass++;
	}
	pr_info("Pass: %d Fail: %d\n", pass, fail);

unmap_destination:
	/* Unmap destination buffers. */
	for (i = 0; i < BD_COUNT; i++) {
		dma_unmap_single(rx_dev->dev, dma_dsts[i],
						BUFFER_SIZE, DMA_BIDIRECTIONAL);
	}

unmap_source:
	/* Unmap source buffers. */
	for (i = 0; dma_srcs[i]; i++)
		dma_unmap_single(tx_dev->dev, dma_srcs[i],
						BUFFER_SIZE, DMA_MEM_TO_DEV);
}

/**
 * do_work_show
 */
static ssize_t do_work_show(struct device *dev,
			struct device_attribute *attr,
				char *buf)
{
	/* Enqueue a work. */
	queue_work(gd.workqueue, gd.work);
	return 0;
}

static DEVICE_ATTR(do_work, 0664, do_work_show, NULL);

/**
 * axidmatester_attrs_array
 */
static struct attribute *axidmatester_attrs_array[] = {
	&dev_attr_do_work.attr,
	NULL,
};

/**
 * axidmatester_attr_group
 */
static struct attribute_group axidmatester_attr_group = {
	.name   = "attributes",
	.attrs  =  axidmatester_attrs_array,
};

/**
 * axidmatester_probe
 */
static int axidmatester_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	/* Request channel 'axidma0'. */
	gd.tx_chan = dma_request_chan(&pdev->dev, "axidma0");
	if (IS_ERR(gd.tx_chan)) {
		dev_err(&pdev->dev, "Request DMA channel 'axidma0' failed!\n");
		ret = -1;
		goto out;
	}

	/* Request channel 'axidma1'. */
	gd.rx_chan = dma_request_chan(&pdev->dev, "axidma1");
	if (IS_ERR(gd.rx_chan)) {
		dev_err(&pdev->dev, "Request DMA channel 'axidma1' failed!\n");
		goto release_axidma0;
	}

	/* Create workqueue. */
	gd.workqueue = create_workqueue("workqueue");
	if (!gd.workqueue) {
		dev_err(&pdev->dev, "Create workqueue failed!\n");
		goto release_axidma1;
	}

	/* Alloc work_struct. */
	gd.work = (struct work_struct *)kmalloc(sizeof(struct work_struct), GFP_KERNEL);
	if (!gd.work) {
		dev_err(&pdev->dev, "Alloc work_struct failed!\n");
		goto destroy_workqueue;
	}

	/* Init work. */
	INIT_WORK(gd.work, work_func);

	/* Alloc memory for the source buffer pointers. */
	gd.srcs = kcalloc(BD_COUNT + 1, sizeof(u8 *), GFP_KERNEL);
	if (!gd.srcs) {
		dev_err(&pdev->dev, "Alloc memroy for the source buffer pointers failed!\n");
		goto free_work;
	}

	/* Set all of the pointers as NULL. */
	memset(gd.srcs, 0, (BD_COUNT + 1) * sizeof(u8 *));

	/* Alloc memory as the source buffer. */
	for (i = 0; i < BD_COUNT; i++) {
		gd.srcs[i] = kmalloc(BUFFER_SIZE, GFP_KERNEL);
		if (!gd.srcs[i]) {
			dev_err(&pdev->dev, "Alloc source buffer (%d) failed!\n", i);
			goto free_source;
		}
	}

	/* Alloc memory for the destination buffer pointers. */
	gd.dsts = kcalloc(BD_COUNT + 1, sizeof(u8 *), GFP_KERNEL);
	if (!gd.dsts) {
		dev_err(&pdev->dev, "Alloc destinations failed!\n");
		goto free_source;
	}

	/* Set all of the pointers as NULL. */
	memset(gd.dsts, 0, (BD_COUNT + 1) * sizeof(u8 *));

	/* Alloc memory as the destination buffer. */
	for (i = 0; i < BD_COUNT; i++) {
		gd.dsts[i] = kmalloc(BUFFER_SIZE, GFP_KERNEL);
		if (!gd.dsts[i]) {
			dev_err(&pdev->dev, "Alloc source buffer (%d) failed!\n", i);
			goto free_destination;
		}
	}

	/* Init other global data fields. */
	gd.counter = 0;
	gd.pdev = pdev;

	/* sysfs_create_group. */
	ret = sysfs_create_group(&pdev->dev.kobj, &axidmatester_attr_group);
	if (ret != 0) {
		dev_err(&pdev->dev, "sysfs_create_group failed!\n");
		goto free_destination;
	}

	return ret;

free_destination:
	/* Free destination buffers. */
	for (i = 0; gd.dsts[i]; i++)
		kfree(gd.dsts[i]);
	/* Free destination buffer pointers. */
	kfree(gd.dsts);

free_source:
	/* Free source buffers. */
	for (i = 0; gd.srcs[i]; i++)
		kfree(gd.srcs[i]);
	/* Free source buffer pointers. */
	kfree(gd.srcs);

free_work:
	/* Free work. */
	kfree(gd.work);

destroy_workqueue:
	/* Destroy workqueue. */
	destroy_workqueue(gd.workqueue);

release_axidma1:
	/* Release channel 'axidma1'. */
	dma_release_channel(gd.rx_chan);

release_axidma0:
	/* Release channel 'axidma0'. */
	dma_release_channel(gd.tx_chan);

out:
	/* Out. */
	return ret;
}

/**
 * axidmatester_remove
 */
static int axidmatester_remove(struct platform_device *pdev)
{
	int i;

	/* Remove sysfs group. */
	sysfs_remove_group(&pdev->dev.kobj, &axidmatester_attr_group);

	/* Free destination buffers. */
	for (i = 0; gd.dsts[i]; i++)
		kfree(gd.dsts[i]);
	/* Free destination buffer pointers. */
	kfree(gd.dsts);

	/* Free source buffers. */
	for (i = 0; gd.srcs[i]; i++)
		kfree(gd.srcs[i]);
	/* Free source buffer pointers. */
	kfree(gd.srcs);

	/* Free work. */
	kfree(gd.work);

	/* Destroy workqueue. */
	destroy_workqueue(gd.workqueue);

	/* Release channel 'axidma1'. */
	dma_release_channel(gd.rx_chan);

	/* Release channel 'axidma0'. */
	dma_release_channel(gd.tx_chan);

	return 0;
}

/**
 * axidmatester_of_match
 */
static const struct of_device_id axidmatester_of_match[] = {
	{
		.compatible = "xlnx,axi-dma-test-1.00.a",
	},
	{

	},
};

MODULE_DEVICE_TABLE(of, axidmatester_of_match);

/**
 * axidmatester_driver
 */
static struct platform_driver axidmatester_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= axidmatester_of_match,
	},
	.probe = axidmatester_probe,
	.remove	= axidmatester_remove,
};

/**
 * axidmatester_init
 */
static int __init axidmatester_init(void)
{
	return platform_driver_register(&axidmatester_driver);
}

module_init(axidmatester_init);

/**
 * axidmatester_exit
 */
static void __exit axidmatester_exit(void)
{
	platform_driver_unregister(&axidmatester_driver);
}

module_exit(axidmatester_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chingbin Li");
MODULE_DESCRIPTION("axidmatester - AXIDMA IP test program.");
