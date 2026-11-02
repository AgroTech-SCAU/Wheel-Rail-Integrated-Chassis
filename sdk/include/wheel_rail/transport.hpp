#pragma once

#include "wheel_rail/protocol.hpp"
#include <chrono>
#include <string>

namespace wheel_rail {
enum class ReceiveStatus { frame, timeout, can_error, error };
struct ReceiveResult {
    ReceiveStatus status{ReceiveStatus::timeout};
    std::error_code error;
    std::uint32_t can_error_bits{};
};

// Client owns a Transport; an application may supply one for simulation.
class Transport {
public:
    virtual ~Transport() = default;
    virtual std::error_code open(const std::string& interface_name) = 0;
    virtual void close() noexcept = 0;
    virtual bool is_open() const noexcept = 0;
    virtual std::error_code send(const protocol::Frame& frame) = 0;
    virtual ReceiveResult receive(protocol::Frame& frame, std::chrono::milliseconds timeout) = 0;
};

class SocketCanTransport final : public Transport {
public:
    SocketCanTransport() = default;
    ~SocketCanTransport() override;
    SocketCanTransport(const SocketCanTransport&) = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;
    std::error_code open(const std::string& interface_name) override;
    void close() noexcept override;
    bool is_open() const noexcept override;
    std::error_code send(const protocol::Frame& frame) override;
    ReceiveResult receive(protocol::Frame& frame, std::chrono::milliseconds timeout) override;
private:
    int socket_{-1};
};
} // namespace wheel_rail
