#pragma once

#include <cmath>


namespace aeropredict::aerodynamics
{


template <typename T> struct State
{
    T V;
    T γ;
    T θ;
    T q;
};


template <typename T> State<T> &operator+=(State<T> &A, const State<T> &B)
{
    A.V += B.V;
    A.γ += B.γ;
    A.θ += B.θ;
    A.q += B.q;

    return A;
}


template <typename T> State<T> operator+(State<T> A, const State<T> &B)
{
    A += B;
    return A;
}


template <typename T> State<T> operator*(T scalar, State<T> B)
{
    B.V *= scalar;
    B.γ *= scalar;
    B.θ *= scalar;
    B.q *= scalar;

    return B;
}


template <typename T> State<T> operator*(State<T> A, T scalar)
{
    return scalar * A;
}


namespace constants
{

// Physical properties
constexpr double m = 11.0;    // kg
constexpr double Iy = 1.135;  // kg m²
constexpr double S = 0.55;    // m²
constexpr double c = 0.18994; // m
constexpr double ρ = 1.2682;  // kg/m³
constexpr double g = 9.81;    // m/s²

// Lift coefficients
constexpr double CL0 = 0.23;
constexpr double CLα = 5.61;
constexpr double CLq = 7.95;
constexpr double CLδe = 0.13;

// Drag coefficients
constexpr double CD0 = 0.0424;
constexpr double CDα = 0.132;
constexpr double CDq = 0.0;
constexpr double CDδe = 0.0135;

// Pitching moment coefficients
constexpr double Cm0 = 0.0135;
constexpr double Cmα = -2.74;
constexpr double Cmq = -38.21;
constexpr double Cmδe = -0.99;

// Simulation settings
constexpr double V0 = 25.0;       // m/s
constexpr double dt = 0.01;       // s
constexpr double duration = 10.0; // s


template <typename T> T CL(const State<T> &state, T δe)
{
    const T V = state.V;
    const T q = state.q;
    const T α = state.θ - state.γ;

    return (CL0 + CLα * α + CLq * ((q * c) / (2 * V)) + CLδe * δe);
}


template <typename T> T CD(const State<T> &state, T δe)
{
    const T V = state.V;
    const T q = state.q;
    const T α = state.θ - state.γ;

    return (CD0 + CDα * α + CDq * ((q * c) / (2 * V)) + CDδe * δe);
}


template <typename T> T Cm(const State<T> &state, T δe)
{
    const T V = state.V;
    const T q = state.q;
    const T α = state.θ - state.γ;

    return (Cm0 + Cmα * α + Cmq * ((q * c) / (2 * V)) + Cmδe * δe);
}


template <typename T> T L(const State<T> &state, T δe)
{
    const T V = state.V;

    return ((0.5) * ρ * V * V * S * CL(state, δe));
}


template <typename T> T D(const State<T> &state, T δe)
{
    const T V = state.V;

    return ((0.5) * ρ * V * V * S * CD(state, δe));
}


template <typename T> T M(const State<T> &state, T δe)
{
    const T V = state.V;

    return ((0.5) * ρ * V * V * S * Cm(state, δe));
}


} // namespace constants


namespace derivative
{

template <typename T> T V_dot(const State<T> &state, T thrust, T δe)
{
    const T γ = state.γ;
    const T α = state.θ - γ;

    return (((thrust * std::cos(α) - constants::D(state, δe)) / constants::m) -
            constants::g * std::sin(γ));
}


template <typename T> T γ_dot(const State<T> &state, T thrust, T δe)
{
    const T V = state.V;
    const T γ = state.γ;
    const T α = state.θ - γ;

    return ((constants::L(state, δe) + thrust * std::sin(α) -
             constants::m * constants::g * std::cos(γ)) /
            (constants::m * V));
}


template <typename T> T θ_dot(const State<T> &state)
{
    return state.q;
}


template <typename T> T q_dot(const State<T> &state, T δe)
{
    return ((constants::M(state, δe)) / (constants::Iy));
}


template <typename T> State<T> y_dot(const State<T> &state, T thrust, T δe)
{
    State<T> derivative;

    derivative.V = V_dot(state, thrust, δe);
    derivative.γ = γ_dot(state, thrust, δe);
    derivative.θ = θ_dot(state);
    derivative.q = q_dot(state, δe);

    return derivative;
}


} // namespace derivative


} // namespace aeropredict::aerodynamics
