#include "modular_3d_localizer/scan_processing_controller.hpp"

namespace modular_3d_localizer
{
ScanProcessingController::ScanProcessingController(const ScanProcessingConfig &config)
: config_(config)
{
}

void ScanProcessingController::setConfig(const ScanProcessingConfig &config)
{
    config_ = config;
    last_processed_scan_stamp_.reset();
}

bool ScanProcessingController::shouldProcess(
    const builtin_interfaces::msg::Time &scan_stamp)
{
    if (config_.max_registration_rate_hz <= 0.0 ||
        !last_processed_scan_stamp_.has_value())
    {
        last_processed_scan_stamp_ = scan_stamp;
        return true;
    }

    const auto &previous_stamp = *last_processed_scan_stamp_;
    const double elapsed_seconds =
        static_cast<double>(scan_stamp.sec) - static_cast<double>(previous_stamp.sec) +
        (static_cast<double>(scan_stamp.nanosec) -
         static_cast<double>(previous_stamp.nanosec)) * 1e-9;
    const double minimum_interval_seconds = 1.0 / config_.max_registration_rate_hz;

    // A bag restart or a clock reset should not block the next scan.
    if (elapsed_seconds < 0.0 || elapsed_seconds >= minimum_interval_seconds)
    {
        last_processed_scan_stamp_ = scan_stamp;
        return true;
    }

    return false;
}
}  // namespace modular_3d_localizer
