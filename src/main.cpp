#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>
#include "aerodynamics.hpp"
#include "integrator.hpp"
#include "prediction.hpp"

#define DEG2RAD(x) (((3.141592 / 180.0) * (x)))


int main()
{
    using namespace aeropredict;

    aerodynamics::State state = {.V = aerodynamics::constants::V0,
                                 .γ = 0.0,
                                 .θ = 0.04970075,
                                 .q = 0.0};

    double t = 0.00;

    double thrust = 10.320108;
    const double δe_trim = -0.12391925;
    double δe = -0.12391925;


    auto f = [&thrust, &δe](auto t, const auto &state)
    { return aerodynamics::derivative::y_dot(state, thrust, δe); };


    double δe_offset[5] = {DEG2RAD(-4.0), DEG2RAD(-2.0), DEG2RAD(0.0), DEG2RAD(2.0),
                           DEG2RAD(4.0)};

    for (size_t i = 0; i < 5; ++i)
    {
        t = 0.0;
        state = {.V = aerodynamics::constants::V0, .γ = 0.0, .θ = 0.04970075, .q = 0.0};

        std::filesystem::path abs_path =
            std::filesystem::absolute("data/data" + std::to_string(i) + ".csv");
        std::ofstream csv_file(abs_path);
        csv_file << std::fixed << std::setprecision(15);

        while (t < 10.00)
        {
            δe = (t >= 1.0 && t < 1.5) ? δe_trim + δe_offset[i] : δe_trim;

            integrator::rk4(state, t, 0.01, f);

            csv_file << t << "," << state.V << "," << state.γ << "," << state.θ << ","
                     << state.q << "\n";
        }

        csv_file.close();
    }

    t = 0.0;
    δe = δe_trim + DEG2RAD(4.0);
    state = {.V = aerodynamics::constants::V0, .γ = 0.0, .θ = 0.04970075, .q = 0.0};

    std::filesystem::path abs_path = std::filesystem::absolute("data/pred.csv");
    std::ofstream csv_file(abs_path);
    csv_file << std::fixed << std::setprecision(15);

    prediction::Trajectory<double> predicted =
        prediction::predict(state, 1.0, 10.0, thrust, δe, 0.01);

    for (std::size_t i = 0; i < predicted.state.size(); ++i)
    {
        csv_file << (predicted.end_time - predicted.start_time) * predicted.timestep * i +
                predicted.start_time
                 << "," << predicted.state[i].V << "," << predicted.state[i].γ << ","
                 << predicted.state[i].θ << "," << predicted.state[i].q << "\n";
    }
    csv_file.close();

    state = {.V = aerodynamics::constants::V0, .γ = 0.0, .θ = 0.04970075, .q = 0.0};
    std::vector<double> candidates;

    for (int i = -30; i <= 30; ++i) {
        candidates.push_back(DEG2RAD(i * 0.1));
    }

    std::filesystem::path abs_path_ = std::filesystem::absolute("data/control.csv");
    std::ofstream csv_file_(abs_path_);
    csv_file_ << std::fixed << std::setprecision(15);

    double time = 0.0;

    while (70.0 > time)
    {
        double deflection = prediction::select_control(state, 1.0, 0.06715404, candidates,
                                                       time, thrust, δe_trim, 0.01);

        csv_file_ << time << "," << state.V << "," << state.γ << "," << state.θ << ","
                  << state.q << "," << deflection + δe_trim << "\n";

        δe = deflection + δe_trim;
        integrator::rk4(state, time, 0.01, f);
    }
    csv_file_.close();
}
