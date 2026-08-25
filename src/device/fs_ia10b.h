#ifndef _fs_ia10b_h_
#define _fs_ia10b_h_

/**
 * @file fs_ia10b.h
 * @brief FlySky FS-iA10B iBUS 接收机驱动接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明
// ========================= ! //

/**
 * @brief iBUS 固定帧长度
 */
#define FS_IA10B_IBUS_FRAME_LEN 32u

/**
 * @brief FS-iA10B 通道数量
 */
#define FS_IA10B_CHANNEL_COUNT 14u

/**
 * @brief 中断接收字节队列长度
 */
#define FS_IA10B_RX_QUEUE_LEN 64u

/**
 * @brief FS-iA10B 驱动状态码
 */
typedef enum {
    FS_IA10B_STATUS_OK = 0,
    FS_IA10B_STATUS_INVALID_PARAM,
    FS_IA10B_STATUS_NOT_INITIALIZED,
    FS_IA10B_STATUS_PORT_ERROR,
} FsIa10bStatus;

/**
 * @brief FS-iA10B 平台能力接口
 */
typedef struct {
    bool (*start_receive)(uint8_t* data, uint16_t len);
    bool (*abort_receive)(void);
    bool (*receive_is_ready)(void);
    uint32_t (*now_ms)(void);
    uint32_t (*critical_enter)(void);
    void (*critical_exit)(uint32_t state);
} FsIa10bPortOps;

/**
 * @brief FS-iA10B 初始化配置
 */
typedef struct {
    const FsIa10bPortOps* ops;
} FsIa10bConfig;

/**
 * @brief iBUS 解码结果与链路状态
 */
typedef struct {
    uint16_t channel[FS_IA10B_CHANNEL_COUNT]; /**< iBUS 原始通道值 */
    uint32_t frame_count;                     /**< 有效帧计数 */
    uint32_t error_count;                     /**< 格式错误与队列溢出计数 */
    uint32_t last_update_ms;                  /**< 最近有效帧接收时间戳 */
    bool valid;                               /**< 是否已接收有效帧 */
} FsIa10bData;

/**
 * @brief FS-iA10B 驱动实例
 */
typedef struct {
    const FsIa10bPortOps* ops;
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

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 FS-iA10B 驱动实例
 * @param self 驱动实例
 * @param config 初始化配置
 * @return FsIa10bStatus 状态码
 */
FsIa10bStatus fs_ia10b_init(FsIa10b* self, const FsIa10bConfig* config);

/**
 * @brief 在主循环维护接收状态并解析缓存字节
 * @param self 驱动实例
 * @return FsIa10bStatus 状态码
 */
FsIa10bStatus fs_ia10b_maintain(FsIa10b* self);

/**
 * @brief 处理单字节接收完成事件
 * @param self 驱动实例
 * @return FsIa10bStatus 状态码
 */
FsIa10bStatus fs_ia10b_on_rx_complete(FsIa10b* self);

/**
 * @brief 处理 UART 接收错误事件
 * @param self 驱动实例
 * @return FsIa10bStatus 状态码
 */
FsIa10bStatus fs_ia10b_on_rx_error(FsIa10b* self);

/**
 * @brief 获取最近一次有效 iBUS 数据快照
 * @param self 驱动实例
 * @param out 输出数据
 * @return FsIa10bStatus 状态码
 */
FsIa10bStatus fs_ia10b_get_data(const FsIa10b* self, FsIa10bData* out);

/**
 * @brief 判断遥控链路是否在线
 * @param self 驱动实例
 * @param timeout_ms 在线超时时间
 * @return bool `true` 表示链路在线
 */
bool fs_ia10b_is_online(const FsIa10b* self, uint32_t timeout_ms);

/**
 * @brief 读取指定遥控通道原始值
 * @param self 驱动实例
 * @param index 通道索引
 * @return uint16_t 通道原始值
 */
uint16_t fs_ia10b_get_channel(const FsIa10b* self, uint8_t index);

#endif
