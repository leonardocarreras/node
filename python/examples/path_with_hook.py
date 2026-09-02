"""
SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, RWTH Aachen University
SPDX-License-Identifier: Apache-2.0
"""  # noqa: E501

import json
import tempfile
import time

import villas.node as vn

OUTPUT = tempfile.mktemp(suffix=".dat")

GENERATOR = {
    "type": "signal.v2",
    "rate": 10.0,
    "realtime": True,
    "limit": 20,
    "in": {"signals": [{"name": "sine", "signal": "sine"}]},
}

SINK = {
    "type": "file",
    "uri": OUTPUT,
    "format": "villas.human",
}


class Gain(vn.Hook):
    """Signal processing in Python, called by the path thread for every sample"""

    def __init__(self, factor):
        super().__init__(priority=50)

        self.factor = factor
        self.calls = 0

    def process(self, smp):
        values = memoryview(smp)
        for i in range(len(values)):
            values[i] *= self.factor

        self.calls += 1

        return vn.Reason.OK


def main():
    generator = vn.Node(json.dumps(GENERATOR), "generator")
    sink = vn.Node(json.dumps(SINK), "sink")

    nodes = vn.NodeList()
    nodes.append(generator)
    nodes.append(sink)

    path = vn.Path()
    path.parse(json.dumps({"in": "generator", "out": "sink"}), nodes)

    generator.check()
    sink.check()
    path.check()

    generator.prepare()
    sink.prepare()

    gain = Gain(100.0)
    path.add_hook(gain)
    path.prepare(nodes)

    print(f"Starting {path}")

    generator.start()
    sink.start()
    path.start()

    time.sleep(2)

    path.stop()
    generator.stop()
    sink.stop()

    print(f"The hook processed {gain.calls} samples")
    print(open(OUTPUT).read())


if __name__ == "__main__":
    main()
