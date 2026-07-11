/**
 * @file tsbp_bus_sim.c
 * @brief PC-side timing feasibility simulation for TK Servo Bus Protocol.
 * @brief TK Servo Bus Protocol 的 PC 端时序可行性仿真。
 */

#include <stdint.h>
#include <stdio.h>

enum {
    FAST_OVERHEAD_BYTES = 6,
    DEFAULT_GUARD_US = 50,
    NODE_PROCESS_US = 20,
    REPLY_GUARD_US = 10,
};

typedef struct {
    uint8_t nodes;
    uint8_t rx_bytes;
    uint8_t tx_bytes;
    uint32_t baud;
} TsbpCase;

static uint32_t byte_time_x1000_us(uint32_t baud)
{
    return (uint32_t)((10000ULL * 1000ULL + (baud / 2U)) / baud);
}

static uint32_t frame_time_us(uint32_t bytes, uint32_t baud)
{
    return (uint32_t)(((uint64_t)bytes * 10ULL * 1000000ULL + (baud / 2U)) / baud);
}

static uint32_t ring_store_forward_us(const TsbpCase *c)
{
    uint32_t total = DEFAULT_GUARD_US + (uint32_t)c->nodes * NODE_PROCESS_US;

    for (uint8_t i = 0; i <= c->nodes; ++i) {
        const uint32_t process_bytes =
            (uint32_t)(c->nodes - i) * c->rx_bytes +
            (uint32_t)i * c->tx_bytes;
        total += frame_time_us(FAST_OVERHEAD_BYTES + process_bytes, c->baud);
    }
    return total;
}

static uint32_t parallel_slotted_us(const TsbpCase *c)
{
    const uint32_t broadcast_bytes =
        FAST_OVERHEAD_BYTES + (uint32_t)c->nodes * c->rx_bytes;
    const uint32_t reply_bytes = FAST_OVERHEAD_BYTES + c->tx_bytes;
    uint32_t total = DEFAULT_GUARD_US + frame_time_us(broadcast_bytes, c->baud);

    for (uint8_t i = 0; i < c->nodes; ++i) {
        total += REPLY_GUARD_US + frame_time_us(reply_bytes, c->baud);
    }
    return total;
}

static const char *fit(uint32_t used_us, uint32_t period_us)
{
    const uint32_t usage_x100 = (used_us * 10000U) / period_us;

    if (usage_x100 > 8000U) {
        return "NO";
    }
    if (usage_x100 > 6000U) {
        return "WARN";
    }
    return "OK";
}

static void print_case(const TsbpCase *c)
{
    const uint32_t ring_us = ring_store_forward_us(c);
    const uint32_t parallel_us = parallel_slotted_us(c);

    printf("baud=%lu nodes=%u rx=%u tx=%u byte=%.2fus\n",
           (unsigned long)c->baud,
           c->nodes,
           c->rx_bytes,
           c->tx_bytes,
           (double)byte_time_x1000_us(c->baud) / 1000.0);

    printf("  ring_store_forward=%4lu us | 1ms=%s 2ms=%s 5ms=%s\n",
           (unsigned long)ring_us,
           fit(ring_us, 1000U),
           fit(ring_us, 2000U),
           fit(ring_us, 5000U));

    printf("  parallel_slotted   =%4lu us | 1ms=%s 2ms=%s 5ms=%s\n",
           (unsigned long)parallel_us,
           fit(parallel_us, 1000U),
           fit(parallel_us, 2000U),
           fit(parallel_us, 5000U));
}

int main(void)
{
    const uint32_t bauds[] = {1000000U, 2000000U};
    const uint8_t nodes[] = {1U, 2U, 4U, 8U};
    const struct {
        uint8_t rx;
        uint8_t tx;
        const char *name;
    } profiles[] = {
        {13U, 17U, "default"},
        {8U, 8U, "compact-speed"},
        {6U, 6U, "minimal-current"},
    };

    printf("TSBP timing feasibility simulation\n");
    printf("assumptions: overhead=%uB, node_process=%uus, guard=%uus, reply_guard=%uus\n\n",
           FAST_OVERHEAD_BYTES, NODE_PROCESS_US, DEFAULT_GUARD_US, REPLY_GUARD_US);

    for (uint32_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); ++b) {
        for (uint32_t p = 0; p < sizeof(profiles) / sizeof(profiles[0]); ++p) {
            printf("[%s]\n", profiles[p].name);
            for (uint32_t n = 0; n < sizeof(nodes) / sizeof(nodes[0]); ++n) {
                const TsbpCase c = {
                    .nodes = nodes[n],
                    .rx_bytes = profiles[p].rx,
                    .tx_bytes = profiles[p].tx,
                    .baud = bauds[b],
                };
                print_case(&c);
            }
            printf("\n");
        }
    }
    return 0;
}
