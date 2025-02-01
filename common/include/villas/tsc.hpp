/* Measure time and sleep with IA-32 time-stamp counter (x86) or ARM cycle-based timing.
 *
 * Author: Steffen Vogel <post@steffenvogel.de>
 * SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cinttypes>

#if defined(__x86_64__) || defined(__i386__)

// ==========================
// x86 IMPLEMENTATION (TSC)
// ==========================

#include <cpuid.h>
#include <x86intrin.h>
#include <villas/kernel/kernel.hpp>

#ifndef bit_TSC
#define bit_TSC (1 << 4)
#endif

#define bit_TSC_INVARIANT (1 << 8)
#define bit_RDTSCP (1 << 27)

struct Tsc {
    uint64_t frequency;
    bool rdtscp_supported;
    bool is_invariant;
};

// Reads the current TSC value on x86
__attribute__((unused)) static uint64_t tsc_now(struct Tsc *t) {
    uint32_t tsc_aux;
    return t->rdtscp_supported ? __rdtscp(&tsc_aux) : __rdtsc();
}

// Function prototype to ensure correct attribute usage
int tsc_init(struct Tsc *t) __attribute__((warn_unused_result));

int tsc_init(struct Tsc *t);

uint64_t tsc_rate_to_cycles(struct Tsc *t, double rate);

#elif defined(__aarch64__)

// ==========================
// ARM (AArch64) IMPLEMENTATION
// Uses CLOCK_MONOTONIC_RAW but returns cycles instead of nanoseconds
// ==========================

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

// Define a constant for nanoseconds per second
#define NANOSECONDS_PER_SECOND 1000000000ULL

struct Tsc {
    uint64_t frequency; // Must be initialized based on hardware
    bool rdtscp_supported;
    bool is_invariant;
};

// Gets the current time in cycles, similar to x86 TSC
__attribute__((unused)) static uint64_t tsc_now(struct Tsc *t) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    uint64_t time_ns = ts.tv_sec * NANOSECONDS_PER_SECOND + ts.tv_nsec;
    
    // Convert nanoseconds to cycles using the frequency
    return (time_ns * t->frequency) / NANOSECONDS_PER_SECOND;
}

// Function prototype to apply the attribute correctly
int tsc_init(struct Tsc *t) __attribute__((warn_unused_result));

// Initialize TSC by setting the frequency (must be measured or assumed)
int tsc_init(struct Tsc *t) {
    FILE *f = fopen("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq", "r");
    if (f) {
        uint64_t freq_khz;
        if (fscanf(f, "%lu", &freq_khz) == 1) { // Check return value to avoid compiler warnings
            t->frequency = freq_khz * 1000; // Convert kHz to Hz
        } else {
            t->frequency = 1000000000ULL; // Default to 1 GHz if reading fails
        }
        fclose(f);
    } else {
        t->frequency = 1000000000ULL; // Default to 1 GHz if file opening fails
    }
    return 0;
}

// Convert rate to cycles based on CPU frequency
uint64_t tsc_rate_to_cycles(struct Tsc *t, double rate) {
    return (uint64_t)(rate * t->frequency);
}

#else

// ==========================
// UNSUPPORTED ARCHITECTURE
// ==========================
#error "Unsupported architecture"

#endif

