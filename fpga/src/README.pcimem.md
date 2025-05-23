# pcimem tool

## License

- SPDX-FileCopyrightText: 2010 Bill Farrow <bfarrow@beyondelectronics.us>
- SPDX-FileCopyrightText: 2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
- SPDX-FileCopyrightText: 2000 Jan-Derk Bakker <J.D.Bakker@its.tudelft.nl>
- SPDX-License-Identifier: GPL-2.0-or-later

## Overview

The pcimem application provides a simple method of reading and writing
to memory registers on a PCI card.

Usage: ./pcimem { sys file } { offset } [ type [ data ] ]
  sys file: sysfs file for the pci resource to act on
  offset  : offset into pci memory region to act upon
  type    : access operation type : [b]yte, [h]alfword, [w]ord
  data    : data to be written

## Platform Support

**WARNING!** This method is platform dependent and may not work on your
particular target architecture.  Refer to the PowerPC section below.

## Example

bash# ./pcimem /sys/devices/pci0001\:00/0001\:00\:07.0/resource0  0 w
  /sys/devices/pci0001:00/0001:00:07.0/resource0 opened.
  Target offset is 0x0, page size is 4096
  mmap(0, 4096, 0x3, 0x1, 3, 0x0)
  PCI Memory mapped to address 0x4801f000.
  Value at offset 0x0 (0x4801f000): 0xC0BE0100

## Why do this at all

When I start working on a new PCI device driver I generally go through a
discovery phase of reading and writing to certain registers on the PCI card.
Over the years I have written lots of small kernel modules to probe addresses
within the PCI memory space, constantly iterating: modify code, recompile, scp
to target, load module, unload module, dmesg.

Urk! There has to be a better way - sysfs and mmap() to the rescue.

## Sysfs

Let's start at with the PCI files under sysfs:

```bash
bash# ls -l /sys/devices/pci0001\:00/0001\:00\:07.0/
total 0
-rw-r--r-- 1 root root     4096 Jul  2 20:13 broken_parity_status
lrwxrwxrwx 1 root root        0 Jul  2 20:13 bus -> ../../../bus/pci
-r--r--r-- 1 root root     4096 Jul  2 20:13 class
-rw-r--r-- 1 root root      256 Jul  2 20:13 config
-r--r--r-- 1 root root     4096 Jul  2 20:13 device
-r--r--r-- 1 root root     4096 Jul  2 20:13 devspec
-rw------- 1 root root     4096 Jul  2 20:13 enable
-r--r--r-- 1 root root     4096 Jul  2 20:13 irq
-r--r--r-- 1 root root     4096 Jul  2 20:13 local_cpus
-r--r--r-- 1 root root     4096 Jul  2 20:13 modalias
-rw-r--r-- 1 root root     4096 Jul  2 20:13 msi_bus
-r--r--r-- 1 root root     4096 Jul  2 20:13 resource
-rw------- 1 root root     4096 Jul  2 20:13 resource0
-rw------- 1 root root    65536 Jul  2 20:13 resource1
-rw------- 1 root root 16777216 Jul  2 20:13 resource2
lrwxrwxrwx 1 root root        0 Jul  2 20:13 subsystem -> ../../../bus/pci
-r--r--r-- 1 root root     4096 Jul  2 20:13 subsystem_device
-r--r--r-- 1 root root     4096 Jul  2 20:13 subsystem_vendor
-rw-r--r-- 1 root root     4096 Jul  2 20:13 uevent
-r--r--r-- 1 root root     4096 Jul  2 20:13 vendor
```

The vendor and device files report the PCI vendor ID and device ID:

```bash
bash# cat device
0x0001
```

This info is also available from lspci

````bash
bash# lspci -v
0001:00:07.0 Class 0680: Unknown device bec0:0001 (rev 01)
    Flags: bus master, 66MHz, medium devsel, latency 128, IRQ 31
    Memory at 8d010000 (32-bit, non-prefetchable) [size=4K]
    Memory at 8d000000 (32-bit, non-prefetchable) [size=64K]
    Memory at 8c000000 (32-bit, non-prefetchable) [size=16M]
```

This PCI card makes 3 seperate regions of memory available to the host
computer. The sysfs resource0 file corresponds to the first memory region. The
PCI card lets the host computer know about these memory regions using the BAR
registers in the PCI config.

## `mmap()`

These sysfs resource can be used with mmap() to map the PCI memory into a
userspace applications memory space.  The application then has a pointer to the
start of the PCI memory region and can read and write values directly. (There
is a bit more going on here with respect to memory pointers, but that is all
taken care of by the kernel).

```c
fd = open("/sys/devices/pci0001\:00/0001\:00\:07.0/resource0", O_RDWR | O_SYNC);
ptr = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
printf("PCI BAR0 0x0000 = 0x%4x\n",  *((unsigned short *) ptr);
```

## PowerPC

To make this work on a PowerPC architecture you also need to make a small
change to the pci core. My example is from kernel 2.6.34, and hopefully this
will be fixed for us in a later kernel version.

```bash
bash# vi arch/powerpc/kernel/pci-common.c
    /* If memory, add on the PCI bridge address offset */
     if (mmap_state == pci_mmap_mem) {
-#if 0 /* See comment in pci_resource_to_user() for why this is disabled */
+#if 1 /* See comment in pci_resource_to_user() for why this is disabled */
         *offset += hose->pci_mem_offset;
 #endif
         res_bit = IORESOURCE_MEM;
 
         /* We pass a fully fixed up address to userland for MMIO instead of
         * a BAR value because X is lame and expects to be able to use that
         * to pass to /dev/mem !
         *
         * That means that we'll have potentially 64 bits values where some
         * userland apps only expect 32 (like X itself since it thinks only
         * Sparc has 64 bits MMIO) but if we don't do that, we break it on
         * 32 bits CHRPs :-(
         *
         * Hopefully, the sysfs insterface is immune to that gunk. Once X
         * has been fixed (and the fix spread enough), we can re-enable the
         * 2 lines below and pass down a BAR value to userland. In that case
         * we'll also have to re-enable the matching code in
         * __pci_mmap_make_offset().
         *
         * BenH.
         */
-#if 0
+#if 1
        else if (rsrc->flags &amp; IORESOURCE_MEM)
                offset = hose->pci_mem_offset;
#endif
```
