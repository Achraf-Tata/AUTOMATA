#pragma once

#include "thermal_network.hpp"
#include "contract_abstraction.hpp"

namespace circular {

// Forward aliases from the authoritative Slide 15-17 models
using model::ALPHA;
using model::BETA;
using model::GAMMA;
using model::TE_NOM;
using model::TH_NOM;
using model::SAFE_MIN;
using model::SAFE_MAX;
using model::NUM_CONTROLS;
using model::CONTROLS;
using model::ThermalNetworkModel;

inline constexpr double TAU = 1.0;

inline void step_network_discrete(
    std::vector<double>& T,
    const std::vector<double>& u,
    double Te)
{
    ThermalNetworkModel model;
    model.step(T, u, Te);
}

inline bool verify_metzler_property() noexcept {
    ThermalNetworkModel model;
    return model.verify_metzler_monotonicity();
}

} // namespace circular
