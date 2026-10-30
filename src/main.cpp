#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "aerodynamics.hpp"
#include "integrator.hpp"


int main()
{

    using namespace aeropredict;

    aerodynamics::State state = {.V = aerodynamics::constants::V0,
                                 .γ = 0.0,
                                 .θ = 0.04970075,
                                 .q = 0.0};

    double t = 0.00;

    double thrust = 10.320108;
    double δe = -0.12391925;


    auto f = [&thrust, &δe](auto t, const auto &state)
    { return aerodynamics::derivative::y_dot(state, thrust, δe); };


    std::filesystem::path abs_path = std::filesystem::absolute("data/data.csv");
    std::ofstream csv_file(abs_path);
    csv_file << std::fixed << std::setprecision(15);


    std::cout << state.V << "," << state.γ << "," << state.θ << "," << state.q
              << std::endl;

    const double δe_trim = -0.12391925;

    while (t < 10.00)
    {
        δe = (t >= 1.0 && t < 1.5) ? δe_trim + 0.0174533 : δe_trim;

        std::cout << state.V << "," << state.γ << "," << state.θ << "," << state.q
                  << std::endl;

        integrator::rk4(state, t, 0.01, f);

        csv_file << state.V << "," << state.γ << "," << state.θ << "," << state.q << "\n";
    }


    csv_file.close();
}