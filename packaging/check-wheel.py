"""
SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, RWTH Aachen University
SPDX-License-Identifier: Apache-2.0
"""  # noqa: E501

import json
import os

import villas.node as vn

# Node-types disappear silently when a dependency is missing at configure time
REQUIRED = {
    "amqp",
    "c37.118",
    "can",
    "comedi",
    "exec",
    "file",
    "infiniband",
    "influxdb",
    "kafka",
    "loopback",
    "modbus",
    "mqtt",
    "nanomsg",
    "ngsi",
    "redis",
    "rtp",
    "shmem",
    "signal.v2",
    "socket",
    "stats",
    "temper",
    "test_rtt",
    "uldaq",
    "webrtc",
    "websocket",
}

GPL = {
    "ethercat",
    "iec60870-5-104",
    "iec61850-8-1",
    "iec61850-9-2",
    "zeromq",
}

gpl = os.environ.get("VILLAS_WHEEL_VARIANT") == "gpl"

types = set(vn.node_types())

missing = sorted((REQUIRED | GPL if gpl else REQUIRED) - types)
if missing:
    raise SystemExit(f"node-types missing from the wheel: {missing}")

if not gpl and GPL & types:
    raise SystemExit(f"GPL node-types in an Apache-2.0 wheel: {sorted(GPL & types)}")

config = {
    "type": "signal.v2",
    "limit": 1,
    "rate": 100.0,
    "in": {"signals": [{"name": "sine", "signal": "sine"}]},
}

node = vn.Node(json.dumps(config), "check")
node.check()
node.prepare()
node.start()
sample = node.read()[0]
node.stop()

print(
    f"{len(types)} node-types, {len(vn.hook_types())} hooks, read {sample.length} value"
)
