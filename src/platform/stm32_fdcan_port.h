#ifndef STM32_FDCAN_PORT_H
#define STM32_FDCAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define STM32_FDCAN_CLASSIC_DATA_LEN 8U

typedef enum {
    STM32_FDCAN_BUS_DRIVE = 0,
    STM32_FDCAN_BUS_STEER,
    STM32_FDCAN_BUS_COUNT,
} Stm32FdcanBus;

typedef enum {
    STM32_FDCAN_ID_STANDARD = 0,
    STM32_FDCAN_ID_EXTENDED,
} Stm32FdcanIdType;

typedef struct {
    uint32_t id;
    Stm32FdcanIdType id_type;
    uint8_t len;
    uint8_t data[STM32_FDCAN_CLASSIC_DATA_LEN];
} Stm32FdcanFrame;

typedef void (*Stm32FdcanRxCallback)(const Stm32FdcanFrame* frame, void* context);

bool stm32_fdcan_port_init(Stm32FdcanBus bus);
bool stm32_fdcan_port_send_standard(Stm32FdcanBus bus,
                                    uint32_t id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    uint32_t timeout_ms);
bool stm32_fdcan_port_send_extended(Stm32FdcanBus bus,
                                    uint32_t id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    uint32_t timeout_ms);
bool stm32_fdcan_port_register_rx_callback(Stm32FdcanBus bus,
                                           Stm32FdcanRxCallback callback,
                                           void* context);

#endif /* STM32_FDCAN_PORT_H */
