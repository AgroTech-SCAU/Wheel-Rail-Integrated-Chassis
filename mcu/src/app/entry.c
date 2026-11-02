#include "entry.h"

#include "chassis_service.h"
#include "host_link_service.h"
#include "stm32_board_port.h"
#include "stm32_host_can_port.h"
#include "stm32_time_port.h"

static bool s_ready;

static void flush_tx(void) {
    WrCanFrame frame;
    for(unsigned i = 0; i < 16u && host_link_service_peek_tx(&frame, stm32_time_now_us()); ++i) {
        if(!stm32_host_can_send(&frame)) break;
        host_link_service_commit_tx();
    }
}

void entry_init(void) {
    uint32_t boot_id;
    ChassisServiceConfig config = chassis_service_default_config();
    s_ready = false;
    /* Free-surface calibration; adjust for the chassis geometry and wheel alignment. */
    config.yaw_bias.enabled = true;
    config.yaw_bias.k_vx = -0.065f;
    config.yaw_bias.k_vy = 0.0466f;
    config.yaw_bias.v_deadband = 0.01f;
    /* Rail radius and probe hard limits must be calibrated here before use.
     * Physical E-stop input must be supplied by the board adapter. */
    if(chassis_service_init_with_config(&config) != CHASSIS_SERVICE_STATUS_OK ||
       !stm32_board_boot_id(&boot_id) || !host_link_service_init(boot_id)) return;
    s_ready = true;
}

void entry_loop(void) {
    WrCanFrame frame;
    uint64_t received_us;
    if(!s_ready) return;
    const Stm32BoardInputs board = stm32_board_read_inputs();
    const ChassisBoardInputs inputs = {
        .physical_estop = board.physical_estop,
        .estop_valid = board.estop_valid,
        .battery_valid = board.battery_valid,
        .battery_voltage = board.battery_voltage,
        .battery_current = board.current,
    };
    chassis_service_set_board_inputs(&inputs);
    /* Drain real RC/motor feedback before considering lower priority host commands. */
    (void)chassis_service_update();
    flush_tx();
    for(unsigned i = 0; i < 8u && stm32_host_can_receive(&frame, &received_us); ++i) {
        (void)host_link_service_receive(&frame, received_us);
        flush_tx();
    }
    (void)chassis_service_update();
    host_link_service_tick(stm32_time_now_us());
    flush_tx();
}
