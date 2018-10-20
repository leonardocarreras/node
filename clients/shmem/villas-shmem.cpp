/** Test "client" for the shared memory interface.
 *
 * Waits on the incoming queue, prints received samples and writes them
 * back to the other queue.
 *
 * @author Georg Martin Reinke <georg.reinke@rwth-aachen.de>
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
 *********************************************************************************/

#include <string.h>
#include <iostream>
#include <atomic>

#include <villas/node/config.h>
#include <villas/log.hpp>
#include <villas/exceptions.hpp>
#include <villas/node.h>
#include <villas/pool.h>
#include <villas/sample.h>
#include <villas/shmem.h>
#include <villas/utils.h>
#include <villas/utils.hpp>

using namespace villas;

static Logger logger = logging.get("test-shmem");

static std::atomic<bool> stop(false);

void usage()
{
	std::cout << "Usage: villas-test-shmem WNAME VECTORIZE" << std::endl
	          << "  WNAME     name of the shared memory object for the output queue" << std::endl
	          << "  RNAME     name of the shared memory object for the input queue" << std::endl
	          << "  VECTORIZE maximum number of samples to read/write at a time" << std::endl;
}

void quit(int, siginfo_t*, void*)
{
	stop = true;
}

int main(int argc, char* argv[])
{
	int ret, readcnt, writecnt, avail;

	struct shmem_int shm;
	struct shmem_conf conf = {
		.polling = 0,
		.queuelen = DEFAULT_SHMEM_QUEUELEN,
		.samplelen = DEFAULT_SHMEM_SAMPLELEN
	};

	if (argc != 4) {
		usage();
		return 1;
	}

	ret = utils::signals_init(quit);
	if (ret)
		throw new RuntimeError("Failed to initialize signals");

	char *wname = argv[1];
	char *rname = argv[2];
	int vectorize = atoi(argv[3]);

	ret = shmem_int_open(wname, rname, &shm, &conf);
	if (ret < 0)
		throw new RuntimeError("Failed to open shared-memory interface");

	struct sample *insmps[vectorize], *outsmps[vectorize];

	while (!stop) {
		readcnt = shmem_int_read(&shm, insmps, vectorize);
		if (readcnt == -1) {
			logger->info("Node stopped, exiting");
			break;
		}

		avail = shmem_int_alloc(&shm, outsmps, readcnt);
		if (avail < readcnt)
			logger->warn("Pool underrun: %d / %d\n", avail, readcnt);

		for (int i = 0; i < avail; i++) {
			outsmps[i]->sequence = insmps[i]->sequence;
			outsmps[i]->ts = insmps[i]->ts;

			int len = MIN(insmps[i]->length, outsmps[i]->capacity);
			memcpy(outsmps[i]->data, insmps[i]->data, SAMPLE_DATA_LENGTH(len));

			outsmps[i]->length = len;
		}

		for (int i = 0; i < readcnt; i++)
			sample_decref(insmps[i]);

		writecnt = shmem_int_write(&shm, outsmps, avail);
		if (writecnt < avail)
			logger->warn("Short write");

		logger->info("Read / Write: {}/{}", readcnt, writecnt);
	}

	ret = shmem_int_close(&shm);
	if (ret)
		throw new RuntimeError("Failed to close shared-memory interface");

	logger->info(CLR_GRN("Goodbye!"));

	return 0;
}
