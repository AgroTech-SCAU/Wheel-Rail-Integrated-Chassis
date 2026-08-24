#include "fs_ia10b.h"

#include <stddef.h>
#include <string.h>

static bool fs_ia10b_ops_valid(const FsIa10bPortOps* ops)
{
    return ops != NULL && ops->start_receive != NULL &&
           ops->abort_receive != NULL && ops->receive_is_ready != NULL &&
           ops->now_ms != NULL && ops->critical_enter != NULL &&
           ops->critical_exit != NULL;
}

static bool fs_ia10b_rearm(FsIa10b* self)
{
    return self->ops->start_receive(&self->rx_byte, 1U);
}

static void fs_ia10b_process_byte(FsIa10b* self, uint8_t byte, uint32_t received_at_ms)
{
    uint8_t i;
    if (self->frame_index == 0U) {
        if (byte == 0x20U) {
            self->frame[0] = 0x20U;
            self->frame_index = 1U;
        }
    } else if (self->frame_index == 1U) {
        if (byte == 0x40U) {
            self->frame[1] = 0x40U;
            self->frame_index = 2U;
        } else {
            self->frame_index = 0U;
        }
    } else {
        self->frame[self->frame_index++] = byte;
        if (self->frame_index >= FS_IA10B_IBUS_FRAME_LEN) {
            uint16_t checksum = 0xFFFFU;
            uint16_t received_checksum;
            for (i = 0U; i < 30U; ++i) {
                checksum = (uint16_t)(checksum - self->frame[i]);
            }
            received_checksum = (uint16_t)(self->frame[30] |
                                           ((uint16_t)self->frame[31] << 8U));
            if (checksum == received_checksum) {
                for (i = 0U; i < FS_IA10B_CHANNEL_COUNT; ++i) {
                    self->data.channel[i] =
                        (uint16_t)(self->frame[2U + i * 2U] |
                        ((uint16_t)self->frame[3U + i * 2U] << 8U));
                }
                self->data.last_update_ms = received_at_ms;
                self->data.valid = true;
                self->data.frame_count++;
            } else {
                self->data.error_count++;
            }
            self->frame_index = 0U;
        }
    }
}

FsIa10bStatus fs_ia10b_init(FsIa10b* self, const FsIa10bConfig* config)
{
    if (self == NULL || config == NULL || !fs_ia10b_ops_valid(config->ops)) {
        return FS_IA10B_STATUS_INVALID_PARAM;
    }

    memset(self, 0, sizeof(*self));
    self->ops = config->ops;
    self->initialized = true;
    if (!fs_ia10b_rearm(self)) {
        self->initialized = false;
        return FS_IA10B_STATUS_PORT_ERROR;
    }
    return FS_IA10B_STATUS_OK;
}

FsIa10bStatus fs_ia10b_maintain(FsIa10b* self)
{
    if (self == NULL) {
        return FS_IA10B_STATUS_INVALID_PARAM;
    }
    if (!self->initialized) {
        return FS_IA10B_STATUS_NOT_INITIALIZED;
    }
    if (self->ops->receive_is_ready() && !fs_ia10b_rearm(self)) {
        return FS_IA10B_STATUS_PORT_ERROR;
    }
    while (self->rx_tail != self->rx_head) {
        uint8_t byte = self->rx_queue[self->rx_tail];
        uint32_t received_at_ms = self->rx_time_queue[self->rx_tail];
        self->rx_tail = (uint8_t)((self->rx_tail + 1U) % FS_IA10B_RX_QUEUE_LEN);
        fs_ia10b_process_byte(self, byte, received_at_ms);
    }
    return FS_IA10B_STATUS_OK;
}

FsIa10bStatus fs_ia10b_on_rx_complete(FsIa10b* self)
{
    uint8_t next_head;

    if (self == NULL) {
        return FS_IA10B_STATUS_INVALID_PARAM;
    }
    if (!self->initialized) {
        return FS_IA10B_STATUS_NOT_INITIALIZED;
    }

    next_head = (uint8_t)((self->rx_head + 1U) % FS_IA10B_RX_QUEUE_LEN);
    if (next_head == self->rx_tail) {
        self->data.error_count++;
    } else {
        self->rx_queue[self->rx_head] = self->rx_byte;
        self->rx_time_queue[self->rx_head] = self->ops->now_ms();
        self->rx_head = next_head;
    }
    return fs_ia10b_rearm(self) ? FS_IA10B_STATUS_OK : FS_IA10B_STATUS_PORT_ERROR;
}

FsIa10bStatus fs_ia10b_on_rx_error(FsIa10b* self)
{
    if (self == NULL) {
        return FS_IA10B_STATUS_INVALID_PARAM;
    }
    if (!self->initialized) {
        return FS_IA10B_STATUS_NOT_INITIALIZED;
    }
    self->data.error_count++;
    self->frame_index = 0U;
    if (!self->ops->abort_receive() || !fs_ia10b_rearm(self)) {
        return FS_IA10B_STATUS_PORT_ERROR;
    }
    return FS_IA10B_STATUS_OK;
}

FsIa10bStatus fs_ia10b_get_data(const FsIa10b* self, FsIa10bData* out)
{
    uint32_t critical_state;
    if (self == NULL || out == NULL) {
        return FS_IA10B_STATUS_INVALID_PARAM;
    }
    if (!self->initialized) {
        return FS_IA10B_STATUS_NOT_INITIALIZED;
    }
    critical_state = self->ops->critical_enter();
    *out = self->data;
    self->ops->critical_exit(critical_state);
    return FS_IA10B_STATUS_OK;
}

bool fs_ia10b_is_online(const FsIa10b* self, uint32_t timeout_ms)
{
    if (self == NULL || !self->initialized || !self->data.valid) {
        return false;
    }
    return (uint32_t)(self->ops->now_ms() - self->data.last_update_ms) < timeout_ms;
}

uint16_t fs_ia10b_get_channel(const FsIa10b* self, uint8_t index)
{
    if (self == NULL || !self->initialized || index >= FS_IA10B_CHANNEL_COUNT) {
        return 0U;
    }
    return self->data.channel[index];
}
