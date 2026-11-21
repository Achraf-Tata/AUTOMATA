#pragma once

#include "thermal_network.hpp"
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace circular::environment {

// Weather Strategy Interface (Strategy Pattern, OCP)
class IWeatherStrategy {
public:
    virtual ~IWeatherStrategy() = default;
    [[nodiscard]] virtual double get_temperature(double time_sec) const = 0;
    [[nodiscard]] virtual const char* get_name() const noexcept = 0;
};

// Standard nominal competition weather from Slide 15: Te = -1.0 C
class ConstantWeatherStrategy : public IWeatherStrategy {
public:
    [[nodiscard]] double get_temperature(double /*time_sec*/) const override {
        return model::TE_NOM; // -1.0 C
    }
    [[nodiscard]] const char* get_name() const noexcept override {
        return "Nominal Competition (-1.0 C)";
    }
};

// 24-hour sinusoidal diurnal weather cycle centered at -1.0 C
class DiurnalWeatherStrategy : public IWeatherStrategy {
public:
    [[nodiscard]] double get_temperature(double time_sec) const override {
        return -1.0 + 3.0 * std::sin(2.0 * M_PI * time_sec / 86400.0); // [-4.0 C, +2.0 C]
    }
    [[nodiscard]] const char* get_name() const noexcept override {
        return "24h Diurnal Cycle (-4..+2 C)";
    }
};

// Cold winter snap weather condition: Te = -5.0 C (stress test)
class ColdSnapWeatherStrategy : public IWeatherStrategy {
public:
    [[nodiscard]] double get_temperature(double /*time_sec*/) const override {
        return -5.0;
    }
    [[nodiscard]] const char* get_name() const noexcept override {
        return "Cold Snap (-5.0 C)";
    }
};

// User-controlled manual outside temperature
class ManualWeatherStrategy : public IWeatherStrategy {
public:
    double manual_temp = model::TE_NOM;

    explicit ManualWeatherStrategy(double temp = model::TE_NOM) : manual_temp(temp) {}

    [[nodiscard]] double get_temperature(double /*time_sec*/) const override {
        return manual_temp;
    }
    [[nodiscard]] const char* get_name() const noexcept override {
        return "Manual Temperature Slider";
    }
};

// Factory for weather strategies (Factory Pattern)
class WeatherFactory {
public:
    enum class Type {
        Nominal,
        Diurnal,
        ColdSnap,
        Manual
    };

    [[nodiscard]] static std::unique_ptr<IWeatherStrategy> create(Type type, double manual_val = -1.0) {
        switch (type) {
            case Type::Nominal:
                return std::make_unique<ConstantWeatherStrategy>();
            case Type::Diurnal:
                return std::make_unique<DiurnalWeatherStrategy>();
            case Type::ColdSnap:
                return std::make_unique<ColdSnapWeatherStrategy>();
            case Type::Manual:
                return std::make_unique<ManualWeatherStrategy>(manual_val);
            default:
                return std::make_unique<ConstantWeatherStrategy>();
        }
    }
};

} // namespace circular::environment
