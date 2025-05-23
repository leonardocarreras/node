/* Memory management.
 *
 * Author: Daniel Krebs <github@daniel-krebs.net>
 * SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>

#include <unistd.h>

#include <villas/log.hpp>
#include <villas/memory_manager.hpp>

namespace villas {

// Basic memory block backed by an address space in the memory graph
//
// This is a generic representation of a chunk of memory in the system. It can
// reside anywhere and represent different types of memory.
class MemoryBlock {
public:
  using deallocator_fn = std::function<void(MemoryBlock *)>;

  using Ptr = std::shared_ptr<MemoryBlock>;
  using UniquePtr = std::unique_ptr<MemoryBlock, deallocator_fn>;

  // cppcheck-suppress passedByValue
  MemoryBlock(size_t offset, size_t size,
              const MemoryManager::AddressSpaceId &addrSpaceId)
      : offset(offset), size(size), addrSpaceId(addrSpaceId) {}

  MemoryManager::AddressSpaceId getAddrSpaceId() const { return addrSpaceId; }

  size_t getSize() const { return size; }

  size_t getOffset() const { return offset; }

protected:
  size_t offset; // Offset (or address) inside address space
  size_t size;   // Size in bytes of this block
  MemoryManager::AddressSpaceId addrSpaceId; // Identifier in memory graph
};

// Wrapper for a MemoryBlock to access the underlying memory directly
//
// The underlying memory block has to be accessible for the current process,
// that means it has to be mapped accordingly and registered to the global
// memory graph.
// Furthermore, this wrapper can be owning the memory block when initialized
// with a moved unique pointer. Otherwise, it just stores a reference to the
// memory block and it's the users responsibility to take care that the memory
// block is valid.
template <typename T> class MemoryAccessor {
public:
  using Type = T;

  // Take ownership of the MemoryBlock
  MemoryAccessor(std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn> mem)
      : translation(MemoryManager::get().getTranslationFromProcess(
            mem->getAddrSpaceId())),
        memoryBlock(std::move(mem)) {}

  // Just act as an accessor, do not take ownership of MemoryBlock
  MemoryAccessor(const MemoryBlock &mem)
      : translation(MemoryManager::get().getTranslationFromProcess(
            mem.getAddrSpaceId())) {}

  MemoryAccessor(const MemoryTranslation &translation)
      : translation(translation) {}

  T &operator*() const {
    return *reinterpret_cast<T *>(translation.getLocalAddr(0));
  }

  T &operator[](size_t idx) const {
    const size_t offset = sizeof(T) * idx;
    return *reinterpret_cast<T *>(translation.getLocalAddr(offset));
  }

  T *operator&() const {
    return reinterpret_cast<T *>(translation.getLocalAddr(0));
  }

  T *operator->() const {
    return reinterpret_cast<T *>(translation.getLocalAddr(0));
  }

  const MemoryBlock &getMemoryBlock() const {
    if (not memoryBlock)
      throw std::bad_alloc();
    else
      return *memoryBlock;
  }

private:
  // Cached memory translation for fast access
  MemoryTranslation translation;

  // Take the unique pointer in case user wants this class to have ownership
  std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn> memoryBlock;
};

// Base memory allocator
//
// Note the usage of CRTP idiom here to access methods of derived allocators.
// The concept is explained here at [1].
//
// [1] https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
template <typename DerivedAllocator> class BaseAllocator {
public:
  // memoryAddrSpaceId: memory that is managed by this allocator
  BaseAllocator(MemoryManager::AddressSpaceId memoryAddrSpaceId)
      : memoryAddrSpaceId(memoryAddrSpaceId) {
    // CRTP
    derivedAlloc = static_cast<DerivedAllocator *>(this);
    std::string loggerName = fmt::format("memory:", derivedAlloc->getName());
    logger = Log::get(loggerName);

    // Default deallocation callback
    free = [&](MemoryBlock *mem) {
      logger->warn("no free callback defined for addr space {}, not freeing",
                   mem->getAddrSpaceId());

      removeMemoryBlock(*mem);
    };
  }

  BaseAllocator(std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn> mem)
      : BaseAllocator(mem->getAddrSpaceId()) {
    // cppcheck-suppress useInitializationList
    memoryBlock = std::move(mem);
    derivedAlloc = nullptr;
  }

  virtual std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn>

  allocateBlock(size_t size) = 0;

  template <typename T> MemoryAccessor<T> allocate(size_t num) {
    if (num == 0) {
      // Doesn't make sense to allocate an empty block
      logger->error("Trying to allocate empty memory");
      throw std::bad_alloc();
    }

    const size_t size = num * sizeof(T);
    auto mem = allocateBlock(size);

    // Check if the allocated memory is really accessible by writing to the
    // allocated memory and reading back. Exponentially increase offset to
    // speed up testing.
    MemoryAccessor<volatile uint8_t> byteAccessor(*mem);
    size_t idx = 0;
    for (int i = 0; idx < mem->getSize(); i++, idx = (1 << i)) {
      auto val = static_cast<uint8_t>(i);
      byteAccessor[idx] = val;
      if (byteAccessor[idx] != val) {
        logger->error("Cannot access allocated memory");
        throw std::bad_alloc();
      }
    }

    return MemoryAccessor<T>(std::move(mem));
  }

protected:
  void insertMemoryBlock(const MemoryBlock &mem) {
    auto &mm = MemoryManager::get();
    mm.createMapping(mem.getOffset(), 0, mem.getSize(), derivedAlloc->getName(),
                     memoryAddrSpaceId, mem.getAddrSpaceId());
  }

  void removeMemoryBlock(const MemoryBlock &mem) {
    // This will also remove any mapping to and from the memory block
    auto &mm = MemoryManager::get();
    mm.removeAddressSpace(mem.getAddrSpaceId());
  }

  MemoryManager::AddressSpaceId getAddrSpaceId() const {
    return memoryAddrSpaceId;
  }

  MemoryBlock::deallocator_fn free;
  Logger logger;

  // Optional, if allocator should own the memory block
  std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn> memoryBlock;

private:
  MemoryManager::AddressSpaceId memoryAddrSpaceId;
  DerivedAllocator *derivedAlloc;
};

// Linear memory allocator
//
// This is the simplest kind of allocator. The idea is to keep a pointer at the
// first memory address of your memory chunk and move it every time an
// allocation is done. Due to its simplicity, this allocator doesn't allow
// specific positions of memory to be freed. Usually, all memory is freed
// together.
class LinearAllocator : public BaseAllocator<LinearAllocator> {
public:
  LinearAllocator(const MemoryManager::AddressSpaceId &memoryAddrSpaceId,
                  size_t memorySize, size_t internalOffset = 0);

  LinearAllocator(std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn> mem)
      : LinearAllocator(mem->getAddrSpaceId(), mem->getSize()) {
    memoryBlock = std::move(mem);
  }

  size_t getAvailableMemory() const { return memorySize - nextFreeAddress; }

  size_t getSize() const { return memorySize; }

  std::string getName() const;

  virtual std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn>
  allocateBlock(size_t size);

private:
  static constexpr size_t alignBytes = sizeof(uintptr_t);
  static constexpr size_t alignMask = alignBytes - 1;

  size_t getAlignmentPadding(uintptr_t addr) const {
    return (alignBytes - (addr & alignMask)) & alignMask;
  }

  size_t nextFreeAddress; // Next chunk will be allocated here
  size_t memorySize;      // Total size of managed memory
  size_t internalOffset;  // Offset in address space (usually 0)
  size_t allocationCount; // Number of individual allocations present
};

// Wrapper around mmap() to create villas memory blocks
//
// This class simply wraps around mmap() and munmap() to allocate memory in the
// host memory via the OS.
class HostRam {
public:
  class HostRamAllocator : public BaseAllocator<HostRamAllocator> {
  public:
    HostRamAllocator();

    std::string getName() const { return "HostRamAlloc"; }

    std::unique_ptr<MemoryBlock, MemoryBlock::deallocator_fn>
    allocateBlock(size_t size);
  };

  static HostRamAllocator &getAllocator() {
    // This will "leak" memory. Because we don't have a complex destructor it's okay
    // that this will be cleaned up implicitly on program exit.
    static auto allocator = new HostRamAllocator();
    return *allocator;
  }
};

class HostDmaRam {
public:
  class HostDmaRamAllocator : public LinearAllocator {
  public:
    HostDmaRamAllocator(int num);

    virtual ~HostDmaRamAllocator();

    std::string getName() const { return getUdmaBufName(num); }

  private:
    int num;
  };

  static HostDmaRamAllocator &getAllocator(int num = 0);

private:
  static std::map<int, std::unique_ptr<HostDmaRamAllocator>> allocators;

  static std::string getUdmaBufName(int num);

  static std::string getUdmaBufBasePath(int num);

  static size_t getUdmaBufBufSize(int num);

  static uintptr_t getUdmaBufPhysAddr(int num);
};

} // namespace villas
