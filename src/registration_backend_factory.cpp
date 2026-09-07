#include "modular_3d_localizer/registration_backend_factory.hpp"

namespace modular_3d_localizer
{
std::unique_ptr<RegistrationBackend> createRegistrationBackend(
    const std::string &backend_name,
    const GicpRegistrationConfig &gicp_config,
    const CoarseToFineGicpRegistrationConfig &coarse_to_fine_gicp_config,
    const FastGicpRegistrationConfig &fast_gicp_config)
{
    if (backend_name == "gicp")
    {
        return std::make_unique<GicpRegistration>(gicp_config);
    }
    if (backend_name == "coarse_to_fine_gicp")
    {
        return std::make_unique<CoarseToFineGicpRegistration>(
            coarse_to_fine_gicp_config);
    }

    if (backend_name == "fast_gicp")
    {
        return std::make_unique<FastGicpRegistration>(fast_gicp_config);
    }

    return nullptr;
}
}  // namespace modular_3d_localizer
