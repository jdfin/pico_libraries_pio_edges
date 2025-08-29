#include <cstdint>
#include <climits>
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/util/queue.h"
#include "xassert.h"
#include "edges.pio.h"
#include "pio_edges.h"

PIO Edges::_pio = NULL; // pointer to the hardware registers
uint Edges::_sm = UINT_MAX;
queue_t Edges::_queue;
int Edges::_sync = 0;
int Edges::_rise = 0;

uint32_t Edges::_tick = 0;
uint32_t Edges::_tick_hi = 0;
uint32_t Edges::_tick_hz = 0;


void Edges::init(int gpio)
{
    uint offset;

    _tick_hz = clock_get_hz(clk_sys) / edges_div;

    bool status =
        pio_claim_free_sm_and_add_program_for_gpio_range(&edges_program, &_pio,
                                                         &_sm, &offset, gpio,
                                                         1, true);
    xassert(status);

    pio_irq_init();

    edges_program_init(_pio, _sm, offset, gpio);
}


// sync=0: waiting for something not a zero or one
//      0/1:    stay in sync=0
//      other:  go to sync=1
// sync=1: waiting for rise/fall (must be zero or one)
//      0/1:    note rise/fall, go to sync=2
//      other:  stay in sync=1
// sync=2: waiting for timestamp (might be zero or one)
//      any:    return timestamp and rise/fall, go to sync=1
bool Edges::get_tick(int& rise, uint32_t& tick)
{
    int32_t data;
    if (!queue_try_remove(&_queue, &data))
        return false;

    if (_sync == 0) {
        if (data != 0 && data != 1)
            _sync = 1;
        return false;
    } else if (_sync == 1) {
        if (data == 0 || data == 1) {
            _rise = data;
            _sync = 2;
        }
        return false;
    } else if (_sync == 2) {
        rise = _rise;
        // pio counts down; change from down counter to up counter
        tick = uint32_t(-data);
        // if new tick is less than the last tick, increment upper 32 bits
        if (tick < _tick)
            _tick_hi++;
        _tick = tick;
        _sync = 1;
        return true;
    }
    xassert(false);
    __builtin_unreachable(); 
}


bool Edges::get_tick(int& rise, uint64_t& tick64)
{
    uint32_t tick_lo;
    if (get_tick(rise, tick_lo)) {
        // convert to 64-bit tick
        tick64 = (uint64_t(_tick_hi) << 32) | tick_lo;
        return true;
    }
    return false;
}


uint32_t Edges::get_tick_hz()
{
    return _tick_hz;
}


// Convert ticks to nsec, with rounding (usually an interval).
// Exact result is not necessarily an integer.
uint64_t Edges::tick_to_nsec(uint64_t tick)
{
    xassert(_tick_hz != 0); // init() sets _tick_hz
    return (tick * 1000000000ULL + _tick_hz / 2) / _tick_hz;
}


uint64_t Edges::tick_to_usec(uint64_t tick)
{
    xassert(_tick_hz != 0); // init() sets _tick_hz
    return (tick * 1000000ULL + _tick_hz / 2) / _tick_hz;
}


uint32_t Edges::div()
{
    return edges_div;
}


void Edges::pio_irq_handler()
{
    // read available data and put them in the queue
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm)) {
        uint32_t data = pio_sm_get_blocking(_pio, _sm);
        (void)queue_try_add(&_queue, &data);
    }
}


void Edges::pio_irq_init()
{
    xassert(_pio != NULL);
    xassert(_sm < NUM_PIO_STATE_MACHINES);

    // find free irq for PIOx_IRQ_0
    int pio_irq = pio_get_irq_num(_pio, 0);

    if (irq_get_exclusive_handler(pio_irq) != NULL) {
        // PIOx_IRQ_0 is in use exclusively, check PIOx_IRQ_1

        pio_irq = pio_get_irq_num(_pio, 1);

        // fail if PIOx_IRQ_1 is also in use exclusively
        bool status = (irq_get_exclusive_handler(pio_irq) != NULL);
        xassert(status);
    }

    queue_init(&_queue, sizeof(uint32_t), queue_len);

    irq_add_shared_handler(pio_irq, pio_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    // enable interrupt in this core's interrupt controller

    irq_set_enabled(pio_irq, true);

    // enable interrupt in the PIO

    const uint irq_idx = pio_irq - pio_get_irq_num(_pio, 0);

    const pio_interrupt_source pis = pio_get_rx_fifo_not_empty_interrupt_source(_sm);

    pio_set_irqn_source_enabled(_pio, irq_idx, pis, true);
}
