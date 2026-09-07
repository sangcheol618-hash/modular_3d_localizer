#pragma once

#include <optional>

#include <builtin_interfaces/msg/time.hpp>

namespace modular_3d_localizer
{
struct ScanProcessingConfig
{
    // Zero disables rate limiting.
    double max_registration_rate_hz{0.0};
};

class ScanProcessingController
{
public:
    explicit ScanProcessingController(
        const ScanProcessingConfig &config = ScanProcessingConfig{});

    void setConfig(const ScanProcessingConfig &config);

    bool shouldProcess(const builtin_interfaces::msg::Time &scan_stamp);

private:
    ScanProcessingConfig config_;
    std::optional<builtin_interfaces::msg::Time> last_processed_scan_stamp_;
};
}  // namespace modular_3d_localizer
