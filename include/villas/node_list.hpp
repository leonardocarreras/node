/** Node list
 *
 * @file
 * @author Steffen Vogel <post@steffenvogel.de>
 * @copyright 2014-2022, Institute for Automation of Complex Power Systems, EONERC
 * @license Apache 2.0
 **********************************************************************************/


#pragma once

#include <uuid/uuid.h>
#include <jansson.h>

#include <list>
#include <string>

namespace villas {
namespace node {

/* Forward declarations */
class Node;

class NodeList : public std::list<Node *> {

public:
	/** Lookup a node from the list based on its name */
	Node * lookup(const std::string &name);

	/** Lookup a node from the list based on its UUID */
	Node * lookup(const uuid_t &uuid);

	/** Parse an array or single node and checks if they exist in the "nodes" section.
	 *
	 * Examples:
	 *     out = [ "sintef", "scedu" ]
	 *     out = "acs"
	 *
	 * @param json A JSON array or string. See examples above.
	 * @param nodes The nodes will be added to this list.
	 * @param all This list contains all valid nodes.
	 */
	int parse(json_t *json, NodeList &all);

	json_t * toJson() const;
};

} // namespace node
} // namespace villas
