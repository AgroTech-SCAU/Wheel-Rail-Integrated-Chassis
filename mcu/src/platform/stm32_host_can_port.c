#include "stm32_host_can_port.h"

#include "fdcan.h"
#include "stm32_time_port.h"

#include <stddef.h>
#include <string.h>

enum { HOST_RX_CAPACITY = 32u, HOST_MAX_ISR_DRAIN = 32u };

typedef struct {
    WrCanFrame frame;
    uint64_t received_us;
} HostRxEntry;

static HostRxEntry s_rx[HOST_RX_CAPACITY];
static volatile uint32_t s_rx_head;
static volatile uint32_t s_rx_tail;
static volatile uint32_t s_rx_dropped;
static volatile bool s_started;

static uint32_t host_ram_words(const FDCAN_InitTypeDef* init) {
    return init->StdFiltersNbr + 2u * init->ExtFiltersNbr +
           init->RxFifo0ElmtsNbr * init->RxFifo0ElmtSize +
           init->RxFifo1ElmtsNbr * init->RxFifo1ElmtSize +
           init->RxBuffersNbr * init->RxBufferSize +
           2u * init->TxEventsNbr +
           (init->TxBuffersNbr + init->TxFifoQueueElmtsNbr) * init->TxElmtSize;
}

static bool host_length_to_dlc(uint8_t length, uint32_t* dlc) {
    switch(length) {
        case 0: *dlc = FDCAN_DLC_BYTES_0; return true;
        case 1: *dlc = FDCAN_DLC_BYTES_1; return true;
        case 2: *dlc = FDCAN_DLC_BYTES_2; return true;
        case 3: *dlc = FDCAN_DLC_BYTES_3; return true;
        case 4: *dlc = FDCAN_DLC_BYTES_4; return true;
        case 5: *dlc = FDCAN_DLC_BYTES_5; return true;
        case 6: *dlc = FDCAN_DLC_BYTES_6; return true;
        case 7: *dlc = FDCAN_DLC_BYTES_7; return true;
        case 8: *dlc = FDCAN_DLC_BYTES_8; return true;
        case 12: *dlc = FDCAN_DLC_BYTES_12; return true;
        case 16: *dlc = FDCAN_DLC_BYTES_16; return true;
        case 20: *dlc = FDCAN_DLC_BYTES_20; return true;
        case 24: *dlc = FDCAN_DLC_BYTES_24; return true;
        case 32: *dlc = FDCAN_DLC_BYTES_32; return true;
        case 48: *dlc = FDCAN_DLC_BYTES_48; return true;
        case 64: *dlc = FDCAN_DLC_BYTES_64; return true;
        default: return false;
    }
}

static bool host_dlc_to_length(uint32_t dlc, uint8_t* length) {
    switch(dlc) {
        case FDCAN_DLC_BYTES_0: *length = 0u; return true;
        case FDCAN_DLC_BYTES_1: *length = 1u; return true;
        case FDCAN_DLC_BYTES_2: *length = 2u; return true;
        case FDCAN_DLC_BYTES_3: *length = 3u; return true;
        case FDCAN_DLC_BYTES_4: *length = 4u; return true;
        case FDCAN_DLC_BYTES_5: *length = 5u; return true;
        case FDCAN_DLC_BYTES_6: *length = 6u; return true;
        case FDCAN_DLC_BYTES_7: *length = 7u; return true;
        case FDCAN_DLC_BYTES_8: *length = 8u; return true;
        case FDCAN_DLC_BYTES_12: *length = 12u; return true;
        case FDCAN_DLC_BYTES_16: *length = 16u; return true;
        case FDCAN_DLC_BYTES_20: *length = 20u; return true;
        case FDCAN_DLC_BYTES_24: *length = 24u; return true;
        case FDCAN_DLC_BYTES_32: *length = 32u; return true;
        case FDCAN_DLC_BYTES_48: *length = 48u; return true;
        case FDCAN_DLC_BYTES_64: *length = 64u; return true;
        default: return false;
    }
}

bool stm32_host_can_init(void) {
    if(s_started) {
        return true;
    }
    const FDCAN_InitTypeDef* host = &hfdcan3.Init;
    if(hfdcan3.Instance != FDCAN3 ||
       HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) != 100000000u ||
       hfdcan1.Init.MessageRAMOffset != 0u ||
       hfdcan2.Init.MessageRAMOffset != 1024u ||
       host->MessageRAMOffset != 1536u ||
       host_ram_words(&hfdcan1.Init) > 1024u ||
       host_ram_words(&hfdcan2.Init) > 512u ||
       host_ram_words(host) > 1024u ||
       host->FrameFormat != FDCAN_FRAME_FD_BRS ||
       host->NominalPrescaler != 5u || host->NominalTimeSeg1 != 15u ||
       host->NominalTimeSeg2 != 4u || host->DataPrescaler != 1u ||
       host->DataTimeSeg1 != 19u || host->DataTimeSeg2 != 5u ||
       hfdcan3.Init.StdFiltersNbr < 4u ||
       hfdcan3.Init.RxFifo0ElmtsNbr < HOST_RX_CAPACITY ||
       hfdcan3.Init.RxFifo0ElmtSize != FDCAN_DATA_BYTES_64 ||
       hfdcan3.Init.TxElmtSize != FDCAN_DATA_BYTES_64) {
        return false;
    }
    for(uint32_t i = 0u; i < 4u; ++i) {
        FDCAN_FilterTypeDef filter = {0};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = i;
        filter.FilterType = FDCAN_FILTER_DUAL;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = WR_CAN_ID_VELOCITY + i;
        filter.FilterID2 = WR_CAN_ID_VELOCITY + i;
        if(HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
            return false;
        }
    }
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
       HAL_FDCAN_ActivateNotification(&hfdcan3,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                      FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0u) != HAL_OK) {
        return false;
    }
    /* Allow the IRQ callback to enqueue a frame arriving as Start completes. */
    s_started = true;
    if(HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        s_started = false;
        return false;
    }
    return true;
}

bool stm32_host_can_receive(WrCanFrame* out, uint64_t* rx_us) {
    if(out == NULL || rx_us == NULL) {
        return false;
    }
    const uint32_t state = stm32_critical_enter();
    if(s_rx_tail == s_rx_head) {
        stm32_critical_exit(state);
        return false;
    }
    const HostRxEntry entry = s_rx[s_rx_tail & (HOST_RX_CAPACITY - 1u)];
    ++s_rx_tail;
    stm32_critical_exit(state);
    *out = entry.frame;
    *rx_us = entry.received_us;
    return true;
}

bool stm32_host_can_send(const WrCanFrame* frame) {
    FDCAN_TxHeaderTypeDef header = {0};
    if(!s_started || frame == NULL || frame->id > 0x7ffu || !frame->fd || !frame->brs ||
       !host_length_to_dlc(frame->length, &header.DataLength) ||
       HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0u) {
        return false;
    }
    header.Identifier = frame->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_ON;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &header,
                                         (uint8_t*)frame->data) == HAL_OK;
}

uint32_t stm32_host_can_dropped_frames(void) {
    const uint32_t state = stm32_critical_enter();
    const uint32_t dropped = s_rx_dropped;
    stm32_critical_exit(state);
    return dropped;
}

void stm32_host_can_on_rx_fifo0(uint32_t interrupts) {
    if(!s_started) {
        return;
    }
    if((interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0u) {
        ++s_rx_dropped;
    }
    if((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }
    /* Bound ISR work even when new frames arrive while draining hardware FIFO. */
    for(uint32_t drained = 0u; drained < HOST_MAX_ISR_DRAIN &&
                              HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) > 0u;
        ++drained) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t payload[64] = {0};
        if(HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &header, payload) != HAL_OK) {
            ++s_rx_dropped;
            break;
        }
        WrCanFrame frame = {0};
        if(header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
           header.FDFormat != FDCAN_FD_CAN || header.BitRateSwitch != FDCAN_BRS_ON ||
           header.Identifier < WR_CAN_ID_VELOCITY ||
           header.Identifier > WR_CAN_ID_TIME_REQUEST ||
           !host_dlc_to_length(header.DataLength, &frame.length)) {
            ++s_rx_dropped;
            continue;
        }
        frame.id = header.Identifier;
        frame.fd = true;
        frame.brs = true;
        memcpy(frame.data, payload, frame.length);
        if(s_rx_head - s_rx_tail >= HOST_RX_CAPACITY) {
            ++s_rx_dropped;
            continue;
        }
        HostRxEntry* slot = &s_rx[s_rx_head & (HOST_RX_CAPACITY - 1u)];
        slot->frame = frame;
        slot->received_us = stm32_time_now_us();
        ++s_rx_head;
    }
}
