#include <filesystem>
#include <fstream>
#include <iomanip>
#include "aerodynamics.hpp"
#include "integrator.hpp"

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

            csv_file << t << "," << state.V << "," << state.γ << "," << state.θ << "," << state.q
                     << "\n";
        }

        csv_file.close();
    }
}