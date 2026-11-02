#include "wheel_rail/error.hpp"

namespace wheel_rail {
namespace {
class Category final : public std::error_category {
public:
    const char* name() const noexcept override { return "wheel_rail"; }
    std::string message(int value) const override {
        switch (static_cast<Errc>(value)) {
        case Errc::invalid_frame: return "not a standard CAN-FD data frame";
        case Errc::unsupported_id: return "unsupported CAN identifier";
        case Errc::unsupported_version: return "unsupported wire protocol version";
        case Errc::invalid_length: return "invalid CAN-FD payload length";
        case Errc::invalid_type: return "message type does not match CAN identifier";
        case Errc::nonzero_reserved: return "reserved bits or padding are nonzero";
        case Errc::invalid_value: return "invalid field value";
        case Errc::not_open: return "CAN interface is not open";
        case Errc::not_authorized: return "autonomous control has not been confirmed";
        case Errc::not_ready: return "fresh ready chassis state is required";
        case Errc::pending_limit: return "too many commands awaiting acknowledgement";
        case Errc::ack_timeout: return "acknowledgement timed out; execution is unknown";
        case Errc::stale_state: return "chassis state is stale";
        case Errc::mcu_restarted: return "MCU boot identifier changed";
        case Errc::command_cancelled: return "pending command was cancelled locally";
        case Errc::stale_sequence: return "duplicate or out-of-order telemetry sequence";
        }
        return "unknown wheel rail error";
    }
};
} // namespace
std::error_code make_error_code(Errc value) noexcept {
    static Category category;
    return {static_cast<int>(value), category};
}
} // namespace wheel_rail
