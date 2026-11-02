#ifndef HOST_LINK_SERVICE_H
#define HOST_LINK_SERVICE_H

#include "wr_can_protocol.h"

/* Single main-loop owner; ISR transport only queues timestamped frames. */
bool host_link_service_init(uint32_t boot_id);
WrResult host_link_service_receive(const WrCanFrame* frame, uint64_t rx_us);
void host_link_service_tick(uint64_t now_us);
/* Peek/commit retains a frame when the hardware TX FIFO is full. */
bool host_link_service_peek_tx(WrCanFrame* frame, uint64_t send_us);
void host_link_service_commit_tx(void);
uint32_t host_link_service_rejected_frames(void);

/* Sensor producers supply actual acquisition timestamps; missing/stale samples remain invalid. */
void host_link_service_submit_imu(const WrImuSample* sample);
void host_link_service_submit_orientation(const WrImuOrientation* sample);

#endif
