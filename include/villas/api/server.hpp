/** Socket API endpoint.
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
 *********************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <poll.h>

#include <villas/common.h>

namespace villas {
namespace node {

/* Forward declarations */
class Api;

namespace api {
namespace sessions {

/* Forward declarations */
class Socket;

} // namespace sessions

class Server {

protected:
	enum state state;

	Api *api;

	int sd;

	std::vector<pollfd> pfds;
	std::vector<sessions::Socket *> sessions;

	void acceptNewSession();
	void closeSession(sessions::Socket *s);

public:
	Server(Api *a);
	~Server();

	void start();
	void stop();

	void run(int timeout = 100);
};

} // namespace api
} // namespace node
} // namespace villas