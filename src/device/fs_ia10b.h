#ifndef FS_IA10B_H
#define FS_IA10B_H

#include <stdbool.h>
#include <stdint.h>

#define FS_IA10B_IBUS_FRAME_LEN 32U
#define FS_IA10B_CHANNEL_COUNT  14U
#define FS_IA10B_RX_QUEUE_LEN   64U

/** FS-iA10B iBUS 设备驱动；中断入口只缓存字节，maintain 在主循环解析 */

typedef enum {
    FS_IA10B_STATUS_OK = 0,
    FS_IA10B_STATUS_INVALID_PARAM,
    FS_IA10B_STATUS_NOT_INITIALIZED,
    FS_IA10B_STATUS_PORT_ERROR,
} FsIa10bStatus;

typedef struct {
    bool (*start_receive)(uint8_t *data, uint16_t len);
    bool (*abort_receive)(void);
    bool (*receive_is_ready)(void);
    uint32_t (*now_ms)(void);
    uint32_t (*critical_enter)(void);
    void (*critical_exit)(uint32_t state);
} FsIa10bPortOps;

typedef struct {
    const FsIa10bPortOps *ops;
} FsIa10bConfig;

typedef struct {
    /** iBUS 原始通道值、链路计数和毫秒时间戳快照 */
    uint16_t channel[FS_IA10B_CHANNEL_COUNT];
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t last_update_ms;
    bool valid;
} FsIa10bData;

typedef struct {
    const FsIa10bPortOps *ops;
    uint8_t rx_byte;
    uint8_t frame[FS_IA10B_IBUS_FRAME_LEN];
    uint8_t frame_index;
    volatile uint8_t rx_queue[FS_IA10B_RX_QUEUE_LEN];
    volatile uint32_t rx_time_queue[FS_IA10B_RX_QUEUE_LEN];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    volatile FsIa10bData data;
    bool initialized;
} FsIa10b;

FsIa10bStatus fs_ia10b_init(FsIa10b *self, const FsIa10bConfig *config);
FsIa10bStatus fs_ia10b_maintain(FsIa10b *self);
FsIa10bStatus fs_ia10b_on_rx_complete(FsIa10b *self);
FsIa10bStatus fs_ia10b_on_rx_error(FsIa10b *self);
FsIa10bStatus fs_ia10b_get_data(const FsIa10b *self, FsIa10bData *out);
bool fs_ia10b_is_online(const FsIa10b *self, uint32_t timeout_ms);
uint16_t fs_ia10b_get_channel(const FsIa10b *self, uint8_t index);

#endif /* FS_IA10B_H */
