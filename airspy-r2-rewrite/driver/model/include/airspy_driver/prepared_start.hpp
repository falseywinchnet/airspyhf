#ifndef AIRSPY_DRIVER_PREPARED_START_HPP
#define AIRSPY_DRIVER_PREPARED_START_HPP

#include <cstdint>

namespace airspy::driver {

enum class ReceiverMode : std::uint8_t {
    off = 0,
    receive = 1,
    armed = 2
};

enum class StartPath : std::uint8_t {
    none,
    prepared,
    legacy
};

enum class StartResult : std::uint8_t {
    ok,
    stop_failed,
    submit_failed,
    receive_failed,
    cleanup_failed
};

struct StartOutcome {
    StartResult result{StartResult::stop_failed};
    StartPath path{StartPath::none};
};

class PreparedStartBackend {
public:
    virtual ~PreparedStartBackend() = default;

    virtual bool set_receiver_mode(ReceiverMode mode) noexcept = 0;
    virtual void clear_bulk_in_halt() noexcept = 0;
    virtual bool submit_transfer_pool() noexcept = 0;
    virtual bool cancel_and_drain_transfer_pool() noexcept = 0;
};

[[nodiscard]] StartOutcome start_receiver(PreparedStartBackend& backend) noexcept;

} // namespace airspy::driver

#endif
