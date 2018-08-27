/** Receive messages from server snd print them on stdout.
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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
 *
 * @addtogroup tools Test and debug tools
 * @{
 *********************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <iostream>

#include <villas/node/config.h>
#include <villas/config_helper.h>
#include <villas/super_node.h>
#include <villas/utils.h>
#include <villas/node.h>
#include <villas/timing.h>
#include <villas/pool.h>
#include <villas/io.h>
#include <villas/kernel/rt.h>
#include <villas/plugin.h>

#include <villas/nodes/websocket.h>

using namespace villas::node;

static SuperNode sn; /**< The global configuration */
static struct io io = { .state = STATE_DESTROYED };

static struct dir {
	struct pool pool;
	pthread_t thread;
	bool enabled;
	int limit;
} sendd, recvv;

struct node *node;

static void quit(int signal, siginfo_t *sinfo, void *ctx)
{
	int ret;

	if (signal == SIGALRM)
		info("Reached timeout. Terminating...");

	if (recvv.enabled) {
		pthread_cancel(recvv.thread);
		pthread_join(recvv.thread, NULL);
	}

	if (sendd.enabled) {
		pthread_cancel(sendd.thread);
		pthread_join(sendd.thread, NULL);
	}

	ret = sn.stop();
	if (ret)
		error("Failed to stop super node");

	if (recvv.enabled) {
		ret = pool_destroy(&recvv.pool);
		if (ret)
			error("Failed to destroy pool");
	}

	if (sendd.enabled) {
		ret = pool_destroy(&sendd.pool);
		if (ret)
			error("Failed to destroy pool");
	}

	ret = io_close(&io);
	if (ret)
		error("Failed to close IO");

	ret = io_destroy(&io);
	if (ret)
		error("Failed to destroy IO");

	info(CLR_GRN("Goodbye!"));
	exit(EXIT_SUCCESS);
}

static void usage()
{
	std::cout << "Usage: villas-pipe [OPTIONS] CONFIG NODE" << std::endl
	          << "  CONFIG  path to a configuration file" << std::endl
	          << "  NODE    the name of the node to which samples are sent and received from" << std::endl
	          << "  OPTIONS are:" << std::endl
	          << "    -f FMT           set the format" << std::endl
	          << "    -o OPTION=VALUE  overwrite options in config file" << std::endl
	          << "    -x               swap read / write endpoints" << std::endl
	          << "    -s               only read data from stdin and send it to node" << std::endl
	          << "    -r               only read data from node and write it to stdout" << std::endl
	          << "    -t NUM           terminate after NUM seconds" << std::endl
	          << "    -L NUM           terminate after NUM samples sent" << std::endl
	          << "    -l NUM           terminate after NUM samples received" << std::endl
	          << "    -h               show this usage information" << std::endl
	          << "    -V               show the version of the tool" << std::endl << std::endl;

	print_copyright();
}

static void * send_loop(void *ctx)
{
	unsigned last_sequenceno = 0, release;
	int ret, scanned, sent, allocated, cnt = 0;
	struct sample *smps[node->out.vectorize];

	/* Initialize memory */
	unsigned pool_size = node_type(node)->pool_size ? node_type(node)->pool_size : LOG2_CEIL(node->out.vectorize);

	ret = pool_init(&sendd.pool, pool_size, SAMPLE_LENGTH(DEFAULT_SAMPLE_LENGTH), node_memory_type(node, &memory_hugepage));

	if (ret < 0)
		error("Failed to allocate memory for receive pool.");

	while (!io_eof(&io)) {
		allocated = sample_alloc_many(&sendd.pool, smps, node->out.vectorize);
		if (ret < 0)
			error("Failed to get %u samples out of send pool (%d).", node->out.vectorize, ret);
		else if (allocated < node->out.vectorize)
			warn("Send pool underrun");

		scanned = io_scan(&io, smps, allocated);
		if (scanned < 0) {
			continue;
			warn("Failed to read samples from stdin");
		}
		else if (scanned == 0)
			continue;

		/* Fill in missing sequence numbers */
		for (int i = 0; i < scanned; i++) {
			if (smps[i]->flags & SAMPLE_HAS_SEQUENCE)
				last_sequenceno = smps[i]->sequence;
			else
				smps[i]->sequence = last_sequenceno++;
		}

		release = allocated;

		sent = node_write(node, smps, scanned, &release);
		if (sent < 0)
			warn("Failed to sent samples to node %s: reason=%d", node_name(node), sent);
		else if (sent < scanned)
			warn("Failed to sent %d out of %d samples to node %s", scanned-sent, scanned, node_name(node));

		sample_decref_many(smps, release);

		cnt += sent;
		if (sendd.limit > 0 && cnt >= sendd.limit)
			goto leave;

		pthread_testcancel();
	}

leave:	if (io_eof(&io)) {
		if (recvv.limit < 0) {
			info("Reached end-of-file. Terminating...");
			killme(SIGTERM);
		}
		else
			info("Reached end-of-file. Wait for receive side...");
	}
	else {
		info("Reached send limit. Terminating...");
		killme(SIGTERM);
	}

	return nullptr;
}

static void * recv_loop(void *ctx)
{
	int recv, ret, cnt = 0, allocated = 0;
	unsigned release;
	struct sample *smps[node->in.vectorize];

	/* Initialize memory */
	unsigned pool_size = node_type(node)->pool_size ? node_type(node)->pool_size : LOG2_CEIL(node->in.vectorize);

	ret = pool_init(&recvv.pool, pool_size, SAMPLE_LENGTH(list_length(&node->signals)), node_memory_type(node, &memory_hugepage));

	if (ret < 0)
		error("Failed to allocate memory for receive pool.");

	for (;;) {
		allocated = sample_alloc_many(&recvv.pool, smps, node->in.vectorize);
		if (allocated < 0)
			error("Failed to allocate %u samples from receive pool.", node->in.vectorize);
		else if (allocated < node->in.vectorize)
			warn("Receive pool underrun: allocated only %i of %i samples", allocated, node->in.vectorize);

		release = allocated;

		recv = node_read(node, smps, allocated, &release);
		if (recv < 0)
			warn("Failed to receive samples from node %s: reason=%d", node_name(node), recv);
		else {
			io_print(&io, smps, recv);

			cnt += recv;
			if (recvv.limit > 0 && cnt >= recvv.limit)
				goto leave;
		}

		sample_decref_many(smps, release);
		pthread_testcancel();
	}

leave:	info("Reached receive limit. Terminating...");
	killme(SIGTERM);

	return nullptr;
}

int main(int argc, char *argv[])
{
	int ret, timeout = 0;
	bool reverse = false;
	const char *format = "villas.human";

	sendd.enabled = true;
	sendd.limit = -1;

	recvv.enabled = true;
	recvv.limit = -1;

	json_t *cfg_cli = json_object();

	int c;
	char *endptr;
	while ((c = getopt(argc, argv, "Vhxrsd:l:L:t:f:o:")) != -1) {
		switch (c) {
			case 'V':
				print_version();
				exit(EXIT_SUCCESS);

			case 'f':
				format = optarg;
				break;

			case 'x':
				reverse = true;
				break;

			case 's':
				recvv.enabled = false; // send only
				break;

			case 'r':
				sendd.enabled = false; // receive only
				break;

			case 'l':
				recvv.limit = strtoul(optarg, &endptr, 10);
				goto check;

			case 'L':
				sendd.limit = strtoul(optarg, &endptr, 10);
				goto check;

			case 't':
				timeout = strtoul(optarg, &endptr, 10);
				goto check;

			case 'o':
				ret = json_object_extend_str(cfg_cli, optarg);
				if (ret)
					error("Invalid option: %s", optarg);
				break;
			case 'h':
			case '?':
				usage();
				exit(c == '?' ? EXIT_FAILURE : EXIT_SUCCESS);
		}

		continue;

check:		if (optarg == endptr)
			error("Failed to parse parse option argument '-%c %s'", c, optarg);
	}

	if (argc != optind + 2) {
		usage();
		exit(EXIT_FAILURE);
	}

	char *configfile = argv[optind];
	char *nodestr    = argv[optind+1];
	struct format_type *fmt;

	ret = signals_init(quit);
	if (ret)
		error("Failed to initialize signals");

	ret = sn.parseUri(configfile);
	if (ret)
		error("Failed to parse configuration");

	ret = sn.init();
	if (ret)
		error("Failed to initialize super-node");

	ret = log_open(sn.getLog());
	if (ret)
		error("Failed to start log");

	fmt = format_type_lookup(format);
	if (!fmt)
		error("Invalid format: %s", format);

	ret = io_init_auto(&io, fmt, DEFAULT_SAMPLE_LENGTH, SAMPLE_HAS_ALL);
	if (ret)
		error("Failed to initialize IO");

	ret = io_check(&io);
	if (ret)
		error("Failed to validate IO configuration");

	ret = io_open(&io, nullptr);
	if (ret)
		error("Failed to open IO");

	node = sn.getNode(nodestr);
	if (!node)
		error("Node %s does not exist!", nodestr);

#ifdef LIBWEBSOCKETS_FOUND
	/* Only start web subsystem if villas-pipe is used with a websocket node */
	if (node_type(node)->start == websocket_start) {
		ret = web_start(sn.getWeb());
		if (ret)
			error("Failed to start web subsystem");

		ret = api_start(sn.getApi());
		if (ret)
			error("Failed to start API subsystem");
	}
#endif /* LIBWEBSOCKETS_FOUND */

	if (reverse)
		node_reverse(node);

	ret = node_type_start(node->_vt);//, &sn); // @todo: port to C++
	if (ret)
		error("Failed to intialize node type %s: reason=%d", node_type_name(node->_vt), ret);

	ret = node_check(node);
	if (ret)
		error("Invalid node configuration");

	ret = node_init2(node);
	if (ret)
		error("Failed to start node %s: reason=%d", node_name(node), ret);

	ret = node_start(node);
	if (ret)
		error("Failed to start node %s: reason=%d", node_name(node), ret);

	/* Start threads */
	if (recvv.enabled)
		pthread_create(&recvv.thread, NULL, recv_loop, NULL);

	if (sendd.enabled)
		pthread_create(&sendd.thread, NULL, send_loop, NULL);

	alarm(timeout);

	for (;;)
		pause();

	return 0;
}

/** @} */
