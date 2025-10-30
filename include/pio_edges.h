#pragma once

#include <cstdint>
#include "hardware/pio.h"
#include "pico/util/queue.h"


class Edges {

    public:

        static void init(int gpio);

        static bool get_tick(int& rise, uint32_t& tick32);

        static bool get_tick(int& rise, uint64_t& tick64);

        static uint32_t get_tick_hz();

        static uint64_t tick_to_nsec(uint64_t tick);
        static uint64_t tick_to_usec(uint64_t tick);

        // below here is mostly for debugging

        static uint32_t div();

        static PIO pio() { return _pio; }
        static uint sm() { return _sm; }

    private:

        static PIO _pio; // pointer to the hardware registers

        static uint _sm;

        // Each edge of the dcc signal uses two entries. The fastest (valid)
        // edges can arrive is one edge every 58 usec. The get_tick() method
        // must be called frequently enough to prevent the queue from filling.
        // To allow 'T' seconds of queuing, the queue needs to be
        //       T / 58us x 2
        // So to allow 10 msec of queuing, we need 346 entries.
        static const uint queue_len = 512; // 2K bytes, at least 14.8 msec

        static queue_t _queue;

        // We're in sync when we see something that is not a zero or one,
        // followed by a zero or one. The first tick will be the next thing
        // in the fifo.
        static int _sync;
        static int _rise;

        static uint32_t _tick;      // last tick value seen
        static uint32_t _tick_hi;   // upper 32 bits of 64-bit tick

        // x-register in pio code decrements every this-many clocks
        // (from pio code in edges.pio)
        static const uint32_t edges_div = 4;

        // calculated in init() as clock_get_hz(clk_sys) / edges_div
        // (unclear to me when clock_get_hz(clk_sys) returns its final value,
        // i.e. it might not be correct in the constructor)
        static uint32_t _tick_hz;

        static void pio_irq_handler();

        static void pio_irq_init();

}; // class Edges
