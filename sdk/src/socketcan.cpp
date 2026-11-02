#include "wheel_rail/transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstring>
#include <system_error>

namespace wheel_rail {
namespace {

std::error_code system_error() noexcept {
    return {errno, std::generic_category()};
}

bool valid_fd_length(std::uint8_t size) noexcept {
    return size <= 8 || size == 12 || size == 16 || size == 20 ||
           size == 24 || size == 32 || size == 48 || size == 64;
}

ReceiveResult receive_error(std::error_code error) {
    return {ReceiveStatus::error, error, 0};
}

} // namespace

SocketCanTransport::~SocketCanTransport() {
    close();
}

std::error_code SocketCanTransport::open(const std::string& interface_name) {
    if (is_open()) {
        return std::make_error_code(std::errc::already_connected);
    }
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ ||
        interface_name.find('\0') != std::string::npos) {
        return std::make_error_code(std::errc::invalid_argument);
    }

    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) {
        return system_error();
    }
    const auto fail = [fd](std::error_code error) {
        ::close(fd);
        return error;
    };

    ifreq request{};
    std::memcpy(request.ifr_name, interface_name.c_str(), interface_name.size());
    if (::ioctl(fd, SIOCGIFINDEX, &request) < 0) {
        return fail(system_error());
    }
    const int interface_index = request.ifr_ifindex;
    if (::ioctl(fd, SIOCGIFMTU, &request) < 0) {
        return fail(system_error());
    }
    if (request.ifr_mtu != CANFD_MTU) {
        return fail(std::make_error_code(std::errc::operation_not_supported));
    }

    const int enabled = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)) < 0) {
        return fail(system_error());
    }

    std::array<can_filter, 10> filters{};
    for (std::size_t index = 0; index < 9; ++index) {
        filters[index].can_id = static_cast<canid_t>(0x180u + index);
        filters[index].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    }
    filters[9].can_id = 0x18bu;
    filters[9].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(sizeof(filters))) < 0) {
        return fail(system_error());
    }

    const can_err_mask_t error_mask = CAN_ERR_MASK;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask,
                     sizeof(error_mask)) < 0) {
        return fail(system_error());
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_index;
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        return fail(system_error());
    }
    socket_ = fd;
    return {};
}

void SocketCanTransport::close() noexcept {
    const int fd = socket_;
    socket_ = -1;
    if (fd >= 0) {
        ::close(fd);
    }
}

bool SocketCanTransport::is_open() const noexcept {
    return socket_ >= 0;
}

std::error_code SocketCanTransport::send(const protocol::Frame& frame) {
    if (!is_open()) {
        return make_error_code(Errc::not_open);
    }
    if (!frame.fd || frame.id > CAN_SFF_MASK) {
        return make_error_code(Errc::invalid_frame);
    }
    if (!valid_fd_length(frame.size)) {
        return make_error_code(Errc::invalid_length);
    }

    canfd_frame message{};
    message.can_id = frame.id;
    message.len = frame.size;
    message.flags = frame.bitrate_switch ? CANFD_BRS : 0;
    std::memcpy(message.data, frame.data.data(), frame.size);
    const ssize_t written = ::write(socket_, &message, CANFD_MTU);
    if (written < 0) {
        return system_error();
    }
    if (written != CANFD_MTU) {
        return std::make_error_code(std::errc::io_error);
    }
    return {};
}

ReceiveResult SocketCanTransport::receive(protocol::Frame& frame,
                                          std::chrono::milliseconds timeout) {
    if (!is_open()) {
        return receive_error(make_error_code(Errc::not_open));
    }
    if (timeout.count() < 0 || timeout.count() > INT_MAX) {
        return receive_error(std::make_error_code(std::errc::invalid_argument));
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    int wait_ms = static_cast<int>(timeout.count());
    for (;;) {
        pollfd descriptor{socket_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, wait_ms);
        if (ready == 0) {
            return {ReceiveStatus::timeout, {}, 0};
        }
        if (ready < 0 && errno != EINTR) {
            return receive_error(system_error());
        }
        if (ready > 0) {
            if (descriptor.revents & POLLNVAL) {
                return receive_error(std::make_error_code(std::errc::bad_file_descriptor));
            }
            if ((descriptor.revents & (POLLIN | POLLERR | POLLHUP)) == 0) {
                return receive_error(std::make_error_code(std::errc::io_error));
            }

            canfd_frame raw{};
            const ssize_t count = ::read(socket_, &raw, CANFD_MTU);
            if (count >= 0) {
                if (count != CAN_MTU && count != CANFD_MTU) {
                    return receive_error(make_error_code(Errc::invalid_length));
                }
                if (raw.can_id & CAN_ERR_FLAG) {
                    return {ReceiveStatus::can_error, {}, raw.can_id & CAN_ERR_MASK};
                }
                if (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) {
                    return receive_error(make_error_code(Errc::invalid_frame));
                }

                protocol::Frame received{};
                received.id = raw.can_id & CAN_SFF_MASK;
                received.fd = count == CANFD_MTU;
                received.bitrate_switch = received.fd && (raw.flags & CANFD_BRS);
                received.size = raw.len;
                if ((received.fd && !valid_fd_length(received.size)) ||
                    (!received.fd && received.size > CAN_MAX_DLEN)) {
                    return receive_error(make_error_code(Errc::invalid_length));
                }
                std::memcpy(received.data.data(), raw.data, received.size);
                frame = received;
                return {ReceiveStatus::frame, {}, 0};
            }
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                return receive_error(system_error());
            }
            if (descriptor.revents & (POLLERR | POLLHUP)) {
                return receive_error(std::make_error_code(std::errc::io_error));
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return {ReceiveStatus::timeout, {}, 0};
        }
        const auto remaining = deadline - now;
        const auto whole_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        wait_ms = static_cast<int>(whole_ms.count() + (whole_ms < remaining ? 1 : 0));
    }
}

} // namespace wheel_rail
