/** HTTP Api session.
 *
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

#include <sstream>

#include <libwebsockets.h>

#include <villas/exceptions.hpp>
#include <villas/web.hpp>
#include <villas/log.hpp>
#include <villas/config.h>
#include <villas/api/sessions/http.hpp>

using namespace villas;
using namespace villas::node;
using namespace villas::node::api::sessions;

Http::Http(Api *a, lws *w) :
	Wsi(a, w)
{
	int hdrlen, options = -1, version;
	char *uri;

	if      ((hdrlen = lws_hdr_total_length(wsi, WSI_TOKEN_OPTIONS_URI)))
		options = 1;
	else if ((hdrlen = lws_hdr_total_length(wsi, WSI_TOKEN_POST_URI)))
		options = 0;

	uri = new char[hdrlen + 1];
	lws_hdr_copy(wsi, uri, hdrlen + 1, options ? WSI_TOKEN_OPTIONS_URI : WSI_TOKEN_POST_URI);

	/* Parse request URI */
	sscanf(uri, "/api/v%d", (int *) &version);

	if (version != api::version)
		throw RuntimeError("Unsupported API version: {}", version);

	/* This is an OPTIONS request.
	 *
	 *  We immediatly send headers and close the connection
	 *  without waiting for a POST body */
	if (options)
		lws_callback_on_writable(wsi);

	delete uri;
}

void Http::read(void *in, size_t len)
{
	request.buffer.append((const char *) in, len);
}

int Http::complete()
{
	json_t *req;

	req = request.buffer.decode();
	if (!req)
		return 0;

	request.buffer.clear();
	request.queue.push(req);

	return 1;
}

int Http::write()
{
	int ret;
	json_t *resp;

	resp = response.queue.pop();

	response.buffer.clear();
	response.buffer.encode(resp);

	json_decref(resp);

	std::stringstream headers;

	headers << "HTTP/1.1 200 OK\r\n"
	        << "Content-type: application/json\r\n"
	        << "User-agent: " USER_AGENT "\r\n"
	        << "Access-Control-Allow-Origin: *\r\n"
	        << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
	        << "Access-Control-Allow-Headers: Content-Type\r\n"
	        << "Access-Control-Max-Age: 86400\r\n"
	        << "Content-Length: " << response.buffer.size() << "\r\n"
	        << "\r\n";

	ret = lws_write(wsi, (unsigned char *) headers.str().data(), headers.str().size(), LWS_WRITE_HTTP_HEADERS);
	if (ret < 0)
		return -1;

	ret = lws_write(wsi, (unsigned char *) response.buffer.data(), response.buffer.size(), LWS_WRITE_HTTP);
	if (ret < 0)
		return -1;

	return 1;
}

std::string Http::getName()
{
	std::stringstream ss;

	ss << Wsi::getName() << ", mode=http";

	return ss.str();
}

int api_http_protocol_cb(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
	int ret;

	lws_context *ctx = lws_get_context(wsi);
	void *user_ctx = lws_context_user(ctx);

	Web *w = static_cast<Web *>(user_ctx);
	Api *a = w->getApi();

	Http *s = static_cast<Http *>(user);

	switch (reason) {
		case LWS_CALLBACK_HTTP_BIND_PROTOCOL:
			if (a == nullptr)
				return -1;

			new (s) Http(a, wsi);

			a->sessions.push_back(s);

			break;

		case LWS_CALLBACK_HTTP_DROP_PROTOCOL:
			if (!s)
				return -1;

			a->sessions.remove(s);

			s->~Http();

			return 1;

		case LWS_CALLBACK_HTTP_BODY:
			s->read(in, len);

			break;

		case LWS_CALLBACK_HTTP_BODY_COMPLETION:
			ret = s->complete();
			if (ret)
				a->pending.push(s);

			break;

		case LWS_CALLBACK_HTTP_WRITEABLE:
			ret = s->write();
			if (ret == 0) {
				goto try_to_reuse;
				break;
			}

			goto try_to_reuse;

		default:
			break;
	}

	return 0;

try_to_reuse:
	if (lws_http_transaction_completed(wsi))
		return -1;

	return 0;
}
