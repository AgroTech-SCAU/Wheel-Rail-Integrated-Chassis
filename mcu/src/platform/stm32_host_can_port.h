#ifndef STM32_HOST_CAN_PORT_H
#define STM32_HOST_CAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "wr_can_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dedicated FDCAN3 host link: PD12/PD13 AF5, standard ID, FD+BRS, up to 64 B.
 * Controller initialization does not prove that the board routes these pins
 * through a powered CAN-FD transceiver. */
bool stm32_host_can_init(void);
bool stm32_host_can_receive(WrCanFrame* out, uint64_t* rx_us);
bool stm32_host_can_send(const WrCanFrame* frame);
uint32_t stm32_host_can_dropped_frames(void);

/* Called only by the single HAL_FDCAN_RxFifo0Callback in stm32_fdcan_port.c. */
void stm32_host_can_on_rx_fifo0(uint32_t interrupts);

#ifdef __cplusplus
}
#endif

#endif
