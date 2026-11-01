#pragma once

#include <iostream>
#include <vector>
#include "aerodynamics.hpp"
#include "integrator.hpp"


namespace aeropredict::prediction
{
template <typename T> struct Trajectory
{
    std::vector<aerodynamics::State<T>> state;
    T start_time;
    T end_time;
    T timestep;
};


template <typename T>
Trajectory<T> predict(aerodynamics::State<T> state, T horizon, T time, T thrust, T δe,
                      T h)
{
    int steps = static_cast<int>(horizon / h);
    Trajectory<T> trajectory;
    trajectory.state.reserve(steps + 2);
    trajectory.state.push_back(state);
    trajectory.start_time = time;
    trajectory.timestep = h;
    T sim_time = time;

    auto f = [&thrust, &δe](auto t, const auto &state)
    { return aerodynamics::derivative::y_dot(state, thrust, δe); };

    for (int i = 0; i < steps; ++i)
    {
        integrator::rk4(state, sim_time, h, f);
        trajectory.state.push_back(state);
    }

    T remaining = horizon - steps * h;

    if (remaining > 0.0)
    {
        integrator::rk4(state, sim_time, remaining, f);
        trajectory.state.push_back(state);
    }

    trajectory.end_time = sim_time;
    return trajectory;
}


template <typename T>
T cost(const aerodynamics::State<T> &state, T θ_target, T δe, T δe_trim)
{
    T A = ((state.θ - θ_target) / (0.01745328889));
    T B = (state.q / 0.05);
    T C = ((δe - δe_trim) / (0.01745328889));

    return A * A + B * B + 0.1 * C * C;
}


template <typename T>
T select_control(const aerodynamics::State<T> &state, T horizon, T θ_target,
                 const std::vector<T> &candidates, T time, T thrust, T δe_trim, T h)
{
    aerodynamics::State<T> pred_state = state;
    Trajectory<T> pred_trajectory =
        predict(pred_state, horizon, time, thrust, candidates[0] + δe_trim, h);

    T lowest_cost =
        cost(pred_trajectory.state.back(), θ_target, candidates[0] + δe_trim, δe_trim);
    T best_cand = candidates[0];

    for (std::size_t i = 1; i < candidates.size(); ++i)
    {
        aerodynamics::State<T> pred_state_l = state;
        Trajectory<T> pred_trajectory_l =
            predict(pred_state_l, horizon, time, thrust, candidates[i] + δe_trim, h);

        T cand_cost = cost(pred_trajectory_l.state.back(), θ_target,
                           candidates[i] + δe_trim, δe_trim);

        if (lowest_cost > cand_cost)
        {
            lowest_cost = cand_cost;
            best_cand = candidates[i];
        }
    }

    return best_cand;
}


} // namespace aeropredict::prediction
