/* Unit tests for kernel functions.
 *
 * Author: Steffen Vogel <post@steffenvogel.de>
 * SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
 * SPDX-License-Identifier: Apache-2.0
 */

#include <criterion/criterion.h>
#include <unistd.h>

#include <villas/kernel/kernel.hpp>
#include <villas/utils.hpp>

using namespace villas::kernel;

// cppcheck-suppress unknownMacro
TestSuite(kernel, .description = "Kernel features");

#define PAGESIZE (1 << 12)
#define CACHELINESIZE 64

#if defined(__x86_64__) || defined(__aarch64__)
#define HUGEPAGESIZE (1 << 21)
#elif defined(__i386__)
#define HUGEPAGESIZE (1 << 22)
#else
#error "Unsupported architecture"
#endif

// This test is not portable.
Test(kernel, sizes) {
  int sz;

  sz = getPageSize();
  cr_assert_eq(sz, PAGESIZE);

  sz = getHugePageSize();
  cr_assert(sz == HUGEPAGESIZE);

  sz = getCachelineSize();
  cr_assert_eq(sz, CACHELINESIZE);
}

#ifdef __linux__
Test(kernel, hugepages) {
  int ret;

  if (!villas::utils::isPrivileged()) {
    cr_skip("Super-user permissions required.");
  }

  ret = setNrHugepages(25);
  cr_assert_eq(ret, 0);

  ret = getNrHugepages();
  cr_assert_eq(ret, 25);

  ret = setNrHugepages(10);
  cr_assert_eq(ret, 0);

  ret = getNrHugepages();
  cr_assert_eq(ret, 10);
}

Test(kernel, version) {
  using villas::utils::Version;

  Version ver = villas::kernel::getVersion();
  Version ver1 = {100, 5};
  Version ver2 = {2, 6};

  cr_assert_lt(ver, ver1);
  cr_assert_gt(ver, ver2);
}

Test(kernel, module, .disabled = true) {
  int ret;

  ret = isModuleLoaded("nf_nat");
  cr_assert_eq(ret, 0);

  ret = isModuleLoaded("does_not_exist");
  cr_assert_neq(ret, 0);
}

Test(kernel, frequency) {
  int ret;
  uint64_t freq;

  ret = get_cpu_frequency(&freq);
  cr_assert_eq(ret, 0);

  // Check for plausability only
  cr_assert(freq > 1e9 && freq < 5e9);
}
#endif
