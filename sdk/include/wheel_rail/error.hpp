#pragma once

#include <system_error>

namespace wheel_rail {
enum class Errc {
    invalid_frame = 1,
    unsupported_id,
    unsupported_version,
    invalid_length,
    invalid_type,
    nonzero_reserved,
    invalid_value,
    not_open,
    not_authorized,
    not_ready,
    pending_limit,
    ack_timeout,
    stale_state,
    mcu_restarted,
    command_cancelled,
    stale_sequence
};
std::error_code make_error_code(Errc value) noexcept;
} // namespace wheel_rail

namespace std {
template<> struct is_error_code_enum<wheel_rail::Errc> : true_type {};
} // namespace std
