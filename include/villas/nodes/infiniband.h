/** Node type: infiniband
 *
 * @file
 * @author Dennis Potter <dennis@dennispotter.eu>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * VILLASnode
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

/**
 * @addtogroup infiniband infiniband node type
 * @ingroup node
 * @{
 */

#pragma once

#include <villas/node.h>
#include <villas/pool.h>
#include <villas/io.h>
#include <villas/queue_signalled.h>
#include <rdma/rdma_cma.h>
/* Enums */
enum poll_mode_e {
	EVENT,
	BUSY
};

struct infiniband {
	/* IBV/RDMA CM structs */
	struct context_s {
		struct rdma_cm_id *listen_id;
		struct rdma_cm_id *id;
		struct rdma_event_channel *ec;

		struct ibv_pd *pd;
		struct ibv_cq *recv_cq;
		struct ibv_cq *send_cq;
		struct ibv_comp_channel *comp_channel;
	} ctx;

	/* Queue Pair init variables */
	struct ibv_qp_init_attr qp_init;

	/* Size of receive and send completion queue */
	int recv_cq_size;
	int send_cq_size;

	/* Bool, set if threads should be aborted */
	int stopThreads;

	/* Connection specific variables */
	struct connection_s {
		struct addrinfo *src_addr;
		struct addrinfo *dst_addr;

		/* RDMA_PS_TCP or RDMA_PS_UDP */
		enum rdma_port_space port_space;

		/* Timeout for rdma_resolve_route */
		int timeout;

		/* Thread to monitor RDMA CM Event threads */
		pthread_t rdma_cm_event_thread;

		/* Bool, should data be send inline if possible? */
		int send_inline;

		/* Stack to temporarily save sent sample */
		struct send_wc_stack_s {
			uint64_t* array;
			unsigned top;
		} send_wc_stack;

		/* Counter to keep track of available recv. WRs */
		int available_recv_wrs;

		/* Fixed number to substract from min. number available
		 * WRs in receive queue */
		int buffer_subtraction;

		/* Unrealiable connectionless data */
		struct rdma_ud_param ud;

	} conn;

	/* Misc settings */
	enum poll_mode_e poll_mode;

	int is_source;
};

/** @see node_type::reverse */
int ib_reverse(struct node *n);

/** @see node_type::print */
char * ib_print(struct node *n);

/** @see node_type::parse */
int ib_parse(struct node *n, json_t *cfg);

/** @see node_type::open */
int ib_start(struct node *n);

/** @see node_type::destroy */
int ib_destroy(struct node *n);

/** @see node_type::close */
int ib_stop(struct node *n);

/** @see node_type::type_start */
int ib_type_start(struct super_node *n);

/** @see node_type::type_stop */
int ib_type_stop();

/** @see node_type::read */
int ib_read(struct node *n, struct sample *smps[], unsigned cnt, unsigned *release);

/** @see node_type::write */
int ib_write(struct node *n, struct sample *smps[], unsigned cnt, unsigned *release);

/** @} */