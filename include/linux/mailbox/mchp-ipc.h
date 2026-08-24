/* SPDX-License-Identifier: GPL-2.0 */
/*
 *Copyright (c) 2024 Microchip Technology Inc. All rights reserved.
 */

#ifndef _LINUX_MCHP_IPC_H_
#define _LINUX_MCHP_IPC_H_

#include <linux/mailbox_controller.h>
#include <linux/types.h>
#include <linux/mailbox_client.h>

#define MAX_MSG_SIZE 64

struct mchp_ipc_msg {
	u32 *buf;
	u16 size;
};

enum ipc_hw {
	MIV_IHC,
	P64H_IPC,
	IPC_HW_NONE
};

struct mchp_ipc_sbi_mbox {
	int id;
	struct platform_device *pdev;
	struct device *dev;
	struct mbox_chan *chans;
	struct mchp_ipc_cluster_cfg *cluster_cfg;
	void *buf_base;
	unsigned long buf_base_addr;
	struct mbox_controller controller;
	enum ipc_hw hw_type;
	struct list_head list;
};

struct p64h_ipc_demo_request {
	int id;
	void *opaque;
};


/* IOCTL command definitions */
#define MCHP_IPC_IOC_MAGIC 'm'
#define MCHP_IPC_IOC_RECEIVER_START _IOW(MCHP_IPC_IOC_MAGIC, 1, void *)
#define MCHP_IPC_IOC_SENDER_START _IOW(MCHP_IPC_IOC_MAGIC, 6, void *)
#define MCHP_IPC_IOC_SHUTDOWN _IO(MCHP_IPC_IOC_MAGIC, 2)
#define MCHP_IPC_IOC_SEND_MSG _IOW(MCHP_IPC_IOC_MAGIC, 3, char *)
#define MCHP_IPC_IOC_RECV_MSG _IOR(MCHP_IPC_IOC_MAGIC, 4, char *)
#define MCHP_IPC_IOC_GET_STATUS _IOR(MCHP_IPC_IOC_MAGIC, 5, int)
#endif /* _LINUX_MCHP_IPC_H_ */
