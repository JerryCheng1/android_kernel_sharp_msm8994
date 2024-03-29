/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IPC_ROUTER_H
#define _IPC_ROUTER_H

#include <linux/types.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/pm.h>
#include <linux/msm_ipc.h>
#include <linux/device.h>
#include <linux/kref.h>

/* Maximum Wakeup Source Name Size */
#define MAX_WS_NAME_SZ 32

#define IPC_RTR_ERR(buf, ...) \
	pr_err("IPC_RTR: " buf, __VA_ARGS__)

/**
 * enum msm_ipc_router_event - Events that will be generated by IPC Router
 */
enum msm_ipc_router_event {
	IPC_ROUTER_CTRL_CMD_DATA = 1,
	IPC_ROUTER_CTRL_CMD_HELLO,
	IPC_ROUTER_CTRL_CMD_BYE,
	IPC_ROUTER_CTRL_CMD_NEW_SERVER,
	IPC_ROUTER_CTRL_CMD_REMOVE_SERVER,
	IPC_ROUTER_CTRL_CMD_REMOVE_CLIENT,
	IPC_ROUTER_CTRL_CMD_RESUME_TX,
};

/**
 * rr_control_msg - Control message structure
 * @cmd: Command identifier for HELLO message in Version 1.
 * @hello: Message structure for HELLO message in Version 2.
 * @srv: Message structure for NEW_SERVER/REMOVE_SERVER events.
 * @cli: Message structure for REMOVE_CLIENT event.
 */
union rr_control_msg {
	uint32_t cmd;
	struct {
		uint32_t cmd;
		uint32_t magic;
		uint32_t capability;
	} hello;
	struct {
		uint32_t cmd;
		uint32_t service;
		uint32_t instance;
		uint32_t node_id;
		uint32_t port_id;
	} srv;
	struct {
		uint32_t cmd;
		uint32_t node_id;
		uint32_t port_id;
	} cli;
};

struct comm_mode_info {
	int mode;
	void *xprt_info;
};

/**
 * msm_ipc_port - Definition of IPC Router port
 * @list: List(local/control ports) in which this port is present.
 * @ref: Reference count for this port.
 * @this_port: Contains port's node_id and port_id information.
 * @port_name: Contains service & instance info if the port hosts a service.
 * @type: Type of the port - Client, Service, Control or Security Config.
 * @flags: Flags to identify the port state.
 * @port_lock_lhc3: Lock to protect access to the port information.
 * @mode_info: Communication mode of the port owner.
 * @port_rx_q: Receive queue where incoming messages are queued.
 * @port_rx_q_lock_lhc3: Lock to protect access to the port's rx_q.
 * @rx_ws_name: Name of the receive wakeup source.
 * @port_rx_ws: Wakeup source to prevent suspend until the rx_q is empty.
 * @port_rx_wait_q: Wait queue to wait for the incoming messages.
 * @restart_state: Flag to hold the restart state information.
 * @restart_lock: Lock to protect access to the restart_state.
 * @restart_wait: Wait Queue to wait for any restart events.
 * @endpoint: Contains the information related to user-space interface.
 * @notify: Function to notify the incoming events on the port.
 * @check_send_permissions: Function to check access control from this port.
 * @num_tx: Number of packets transmitted.
 * @num_rx: Number of packets received.
 * @num_tx_bytes: Number of bytes transmitted.
 * @num_rx_bytes: Number of bytes received.
 * @priv: Private information registered by the port owner.
 */
struct msm_ipc_port {
	struct list_head list;
	struct kref ref;

	struct msm_ipc_port_addr this_port;
	struct msm_ipc_port_name port_name;
	uint32_t type;
	unsigned flags;
	struct mutex port_lock_lhc3;
	struct comm_mode_info mode_info;

	struct msm_ipc_port_addr dest_addr;
	int conn_status;

	struct list_head port_rx_q;
	struct mutex port_rx_q_lock_lhc3;
	char rx_ws_name[MAX_WS_NAME_SZ];
	struct wakeup_source port_rx_ws;
	wait_queue_head_t port_rx_wait_q;
	wait_queue_head_t port_tx_wait_q;

	int restart_state;
	spinlock_t restart_lock;
	wait_queue_head_t restart_wait;

	void *rport_info;
	void *endpoint;
	void (*notify)(unsigned event, void *oob_data,
		       size_t oob_data_len, void *priv);
	int (*check_send_permissions)(void *data);

	uint32_t num_tx;
	uint32_t num_rx;
	unsigned long num_tx_bytes;
	unsigned long num_rx_bytes;
	void *priv;
};

#ifdef CONFIG_IPC_ROUTER
/**
 * msm_ipc_router_create_port() - Create a IPC Router port/endpoint
 * @notify: Callback function to notify any event on the port.
 *   @event: Event ID to be handled.
 *   @oob_data: Any out-of-band data associated with the event.
 *   @oob_data_len: Size of the out-of-band data, if valid.
 *   @priv: Private data registered during the port creation.
 * @priv: Private info to be passed while the notification is generated.
 *
 * @return: Pointer to the port on success, NULL on error.
 */
struct msm_ipc_port *msm_ipc_router_create_port(
	void (*notify)(unsigned event, void *oob_data,
		       size_t oob_data_len, void *priv),
	void *priv);

/**
 * msm_ipc_router_bind_control_port() - Bind a port as a control port
 * @port_ptr: Port which needs to be marked as a control port.
 *
 * @return: 0 on success, standard Linux error codes on error.
 */
int msm_ipc_router_bind_control_port(struct msm_ipc_port *port_ptr);

/**
 * msm_ipc_router_lookup_server_name() - Resolve server address
 * @srv_name: Name<service:instance> of the server to be resolved.
 * @srv_info: Buffer to hold the resolved address.
 * @num_entries_in_array: Number of server info the buffer can hold.
 * @lookup_mask: Mask to specify the range of instances to be resolved.
 *
 * @return: Number of server addresses resolved on success, < 0 on error.
 */
int msm_ipc_router_lookup_server_name(struct msm_ipc_port_name *srv_name,
				      struct msm_ipc_server_info *srv_info,
				      int num_entries_in_array,
				      uint32_t lookup_mask);

/**
 * msm_ipc_router_send_msg() - Send a message/packet
 * @src: Sender's address/port.
 * @dest: Destination address.
 * @data: Pointer to the data to be sent.
 * @data_len: Length of the data to be sent.
 *
 * @return: 0 on success, < 0 on error.
 */
int msm_ipc_router_send_msg(struct msm_ipc_port *src,
			    struct msm_ipc_addr *dest,
			    void *data, unsigned int data_len);

/**
 * msm_ipc_router_get_curr_pkt_size() - Get the packet size of the first
 *                                      packet in the rx queue
 * @port_ptr: Port which owns the rx queue.
 *
 * @return: Returns the size of the first packet, if available.
 *          0 if no packets available, < 0 on error.
 */
int msm_ipc_router_get_curr_pkt_size(struct msm_ipc_port *port_ptr);

/**
 * msm_ipc_router_read_msg() - Read a message/packet
 * @port_ptr: Receiver's port/address.
 * @data: Pointer containing the address of the received data.
 * @src: Address of the sender/source.
 * @len: Length of the data being read.
 *
 * @return: 0 on success, < 0 on error.
 */
int msm_ipc_router_read_msg(struct msm_ipc_port *port_ptr,
			    struct msm_ipc_addr *src,
			    unsigned char **data,
			    unsigned int *len);

/**
 * msm_ipc_router_close_port() - Close the port
 * @port_ptr: Pointer to the port to be closed.
 *
 * @return: 0 on success, < 0 on error.
 */
int msm_ipc_router_close_port(struct msm_ipc_port *port_ptr);

/**
 * msm_ipc_router_register_server() - Register a service on a port
 * @server_port: IPC Router port with which a service is registered.
 * @name: Service name <service_id:instance_id> that gets registered.
 *
 * @return: 0 on success, standard Linux error codes on error.
 */
int msm_ipc_router_register_server(struct msm_ipc_port *server_port,
				   struct msm_ipc_addr *name);

/**
 * msm_ipc_router_unregister_server() - Unregister a service from a port
 * @server_port: Port with with a service is already registered.
 *
 * @return: 0 on success, standard Linux error codes on error.
 */
int msm_ipc_router_unregister_server(struct msm_ipc_port *server_port);

#else

struct msm_ipc_port *msm_ipc_router_create_port(
	void (*notify)(unsigned event, void *oob_data,
		       size_t oob_data_len, void *priv),
	void *priv)
{
	return NULL;
}

static inline int msm_ipc_router_bind_control_port(
		struct msm_ipc_port *port_ptr)
{
	return -ENODEV;
}

int msm_ipc_router_lookup_server_name(struct msm_ipc_port_name *srv_name,
				      struct msm_ipc_server_info *srv_info,
				      int num_entries_in_array,
				      uint32_t lookup_mask)
{
	return -ENODEV;
}

int msm_ipc_router_send_msg(struct msm_ipc_port *src,
			    struct msm_ipc_addr *dest,
			    void *data, unsigned int data_len)
{
	return -ENODEV;
}

int msm_ipc_router_get_curr_pkt_size(struct msm_ipc_port *port_ptr)
{
	return -ENODEV;
}

int msm_ipc_router_read_msg(struct msm_ipc_port *port_ptr,
			    struct msm_ipc_addr *src,
			    unsigned char **data,
			    unsigned int *len)
{
	return -ENODEV;
}

int msm_ipc_router_close_port(struct msm_ipc_port *port_ptr)
{
	return -ENODEV;
}

static inline int msm_ipc_router_register_server(
			struct msm_ipc_port *server_port,
			struct msm_ipc_addr *name)
{
	return -ENODEV;
}

static inline int msm_ipc_router_unregister_server(
			struct msm_ipc_port *server_port)
{
	return -ENODEV;
}

#endif

#endif
