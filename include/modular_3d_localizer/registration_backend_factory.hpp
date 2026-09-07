#pragma once

#include <memory>
#include <string>

#include "modular_3d_localizer/localization_backend/coarse_to_fine_gicp_registration.hpp"
#include "modular_3d_localizer/localization_backend/fast_gicp_registration.hpp"
#include "modular_3d_localizer/localization_backend/gicp_registration.hpp"
#include "modular_3d_localizer/registration_backend.hpp"

namespace modular_3d_localizer
{
std::unique_ptr<RegistrationBackend> createRegistrationBackend(
    const std::string &backend_name,
    const GicpRegistrationConfig &gicp_config,
    const CoarseToFineGicpRegistrationConfig &coarse_to_fine_gicp_config,
    const FastGicpRegistrationConfig &fast_gicp_config);
}  // namespace modular_3d_localizer
