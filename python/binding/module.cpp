/* Python bindings.
 *
 * SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <villas/hook.hpp>
#include <villas/hook_list.hpp>
#include <villas/node.hpp>
#include <villas/node/memory.hpp>
#include <villas/node_list.hpp>
#include <villas/path.hpp>
#include <villas/plugin.hpp>
#include <villas/sample.hpp>
#include <villas/signal_list.hpp>
#include <villas/super_node.hpp>

namespace py = pybind11;
using namespace villas::node;

// Intrusive holder over the reference count of a sample.
template <typename T> class SampleRef {
public:
  SampleRef() : ptr(nullptr) {}
  explicit SampleRef(T *p) : ptr(p) {
    if (ptr)
      sample_incref(ptr);
  }
  SampleRef(const SampleRef &o) : SampleRef(o.ptr) {}
  ~SampleRef() {
    if (ptr)
      sample_decref(ptr);
  }
  T *get() const { return ptr; }

private:
  T *ptr;
};

PYBIND11_DECLARE_HOLDER_TYPE(T, SampleRef<T>, true)

static SampleRef<Sample> wrapNew(Sample *s) {
  SampleRef<Sample> ref(s);
  sample_decref(s);

  return ref;
}

static Node *makeNode(const std::string &config, const std::string &name) {
  json_error_t err;
  json_t *json = json_loads(config.c_str(), 0, &err);
  if (!json)
    throw py::value_error(err.text);

  uuid_t id;
  uuid_clear(id);

  auto *n = NodeFactory::make(json, id, name);
  if (!n)
    throw py::value_error("failed to create node");

  return n;
}

// Lets a Python class override Hook::process()
class PyHook : public Hook {
public:
  using Hook::Hook;

  Reason process(struct Sample *smp) override {
    PYBIND11_OVERRIDE(Reason, Hook, process, smp);
  }
};

PYBIND11_MODULE(_core, m) {
  if (villas::node::memory::init(0))
    throw std::runtime_error("failed to initialize memory subsystem");

  py::enum_<State>(m, "State")
      .value("INITIALIZED", State::INITIALIZED)
      .value("PARSED", State::PARSED)
      .value("CHECKED", State::CHECKED)
      .value("PREPARED", State::PREPARED)
      .value("STARTED", State::STARTED)
      .value("STOPPED", State::STOPPED);

  py::class_<Sample, SampleRef<Sample>>(m, "Sample", py::buffer_protocol())
      .def(py::init([](unsigned capacity) {
             return wrapNew(sample_alloc_mem(capacity));
           }),
           py::arg("capacity") = 64)
      .def_readwrite("sequence", &Sample::sequence)
      .def_readwrite("length", &Sample::length)
      .def_readonly("capacity", &Sample::capacity)
      .def_readwrite("flags", &Sample::flags)
      .def_property_readonly("ts_origin",
                             [](const Sample &s) {
                               return std::make_pair(s.ts.origin.tv_sec,
                                                     s.ts.origin.tv_nsec);
                             })
      .def_buffer([](Sample &s) -> py::buffer_info {
        return py::buffer_info(
            reinterpret_cast<double *>(s.data), sizeof(double),
            py::format_descriptor<double>::format(), 1,
            {static_cast<ssize_t>(s.length)},
            {static_cast<ssize_t>(sizeof(union SignalData))});
      });

  py::class_<Node>(m, "Node")
      .def(py::init([](const std::string &config, const std::string &name) {
             return makeNode(config, name);
           }),
           py::arg("config"), py::arg("name") = "")
      .def_property_readonly("name", &Node::getName)
      .def_property_readonly("state", &Node::getState)
      .def_property_readonly("details", &Node::getDetails)
      .def("check", &Node::check)
      .def("prepare", &Node::prepare)
      .def("start", &Node::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Node::stop, py::call_guard<py::gil_scoped_release>())
      .def(
          "read",
          [](Node &n, unsigned cnt, unsigned capacity) {
            std::vector<Sample *> raw(cnt);
            for (unsigned i = 0; i < cnt; i++)
              raw[i] = sample_alloc_mem(capacity);

            int ret;
            {
              py::gil_scoped_release unlock;
              ret = n.read(raw.data(), cnt);
            }

            std::vector<SampleRef<Sample>> out;
            for (unsigned i = 0; i < cnt; i++) {
              if (ret > 0 && i < (unsigned)ret)
                out.push_back(wrapNew(raw[i]));
              else
                sample_decref(raw[i]);
            }

            if (ret < 0)
              throw std::runtime_error("node read failed");

            return out;
          },
          py::arg("cnt") = 1, py::arg("capacity") = 64)
      .def(
          "new_sample",
          [](Node &n, unsigned capacity) {
            auto *s = sample_alloc_mem(capacity);

            // Node::getOutputSignals() is null unless a path feeds this node
            auto sigs = n.getOutputSignals();
            if (!sigs)
              sigs = std::make_shared<SignalList>(capacity, SignalType::FLOAT);

            new (&s->signals) SignalList::Ptr(sigs);

            return wrapNew(s);
          },
          py::arg("capacity") = 64)
      .def(
          "write",
          [](Node &n, std::vector<SampleRef<Sample>> smps) {
            std::vector<Sample *> raw;
            for (auto &s : smps)
              raw.push_back(s.get());

            py::gil_scoped_release unlock;
            return n.write(raw.data(), raw.size());
          },
          py::arg("samples"))
      .def_property_readonly("input_signals",
                             [](Node &n) {
                               std::vector<std::string> names;
                               auto sigs = n.getInputSignals(false);
                               if (sigs)
                                 for (const auto &s : *sigs)
                                   names.push_back(s->name);
                               return names;
                             })
      .def("__repr__", [](Node &n) {
        return "<villas.node.Node '" + n.getNameShort() + "'>";
      });

  py::class_<NodeList>(m, "NodeList")
      .def(py::init<>())
      .def(
          "append", [](NodeList &l, Node *n) { l.push_back(n); },
          py::keep_alive<1, 2>(), py::arg("node"))
      .def("__len__", [](const NodeList &l) { return l.size(); });

  py::enum_<Hook::Reason>(m, "Reason")
      .value("OK", Hook::Reason::OK)
      .value("ERROR", Hook::Reason::ERROR)
      .value("SKIP_SAMPLE", Hook::Reason::SKIP_SAMPLE)
      .value("STOP_PROCESSING", Hook::Reason::STOP_PROCESSING);

  py::enum_<Hook::Flags>(m, "HookFlags")
      .value("BUILTIN", Hook::Flags::BUILTIN)
      .value("PATH", Hook::Flags::PATH)
      .value("NODE_READ", Hook::Flags::NODE_READ)
      .value("NODE_WRITE", Hook::Flags::NODE_WRITE);

  py::class_<SignalList, SignalList::Ptr> signalList(m, "SignalList");
  py::class_<Path> path(m, "Path");
  py::class_<Hook, PyHook, Hook::Ptr> hook(m, "Hook");

  signalList.def("__len__", [](const SignalList &l) { return l.size(); })
      .def("names", [](const SignalList &l) {
        std::vector<std::string> names;
        for (const auto &s : l)
          names.push_back(s->name);

        return names;
      });

  hook.def(py::init([](int flags, int priority, bool enabled) {
             return new PyHook(nullptr, nullptr, flags, priority, enabled);
           }),
           py::arg("flags") =
               (int)Hook::Flags::PATH | (int)Hook::Flags::BUILTIN,
           py::arg("priority") = 100, py::arg("enabled") = true)
      .def("process", &Hook::process, py::arg("sample"))
      .def_property_readonly("signals", &Hook::getSignals);

  path.def(py::init<>())
      .def(
          "parse",
          [](Path &p, const std::string &config, NodeList &nodes) {
            json_error_t err;
            json_t *json = json_loads(config.c_str(), 0, &err);
            if (!json)
              throw py::value_error(err.text);

            uuid_t sn;
            uuid_clear(sn);

            p.parse(json, nodes, sn);
          },
          py::arg("config"), py::arg("nodes"))
      .def("check", &Path::check)
      .def("prepare", &Path::prepare, py::arg("nodes"))
      .def("start", &Path::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Path::stop, py::call_guard<py::gil_scoped_release>())
      .def(
          "add_hook", [](Path &p, Hook::Ptr h) { p.hooks.push_back(h); },
          py::keep_alive<1, 2>(), py::arg("hook"))
      .def_property_readonly("state", &Path::getState)
      .def("__str__", &Path::toString);

  m.def("node_types", []() {
    std::vector<std::string> types;
    for (auto *f : villas::plugin::registry->lookup<NodeFactory>())
      types.push_back(f->getName());

    std::sort(types.begin(), types.end());

    return types;
  });

  m.def("hook_types", []() {
    std::vector<std::string> types;
    for (auto *f : villas::plugin::registry->lookup<HookFactory>())
      types.push_back(f->getName());

    std::sort(types.begin(), types.end());

    return types;
  });

  py::class_<SuperNode>(m, "SuperNode")
      .def(py::init<>())
      .def(
          "load",
          [](SuperNode &sn, const std::string &path) { sn.parse(path); },
          py::arg("path"))
      .def("check", &SuperNode::check)
      .def("prepare", &SuperNode::prepare)
      .def("start", &SuperNode::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &SuperNode::stop, py::call_guard<py::gil_scoped_release>())
      .def("node", &SuperNode::getNode, py::return_value_policy::reference,
           py::arg("name"))
      .def_property_readonly("node_names",
                             [](SuperNode &sn) {
                               std::vector<std::string> names;
                               for (auto *n : sn.getNodes())
                                 names.push_back(n->getNameShort());

                               return names;
                             })
      .def_property_readonly("paths", [](SuperNode &sn) {
        std::vector<std::string> paths;
        for (auto *p : sn.getPaths())
          paths.push_back(p->toString());

        return paths;
      });
}
