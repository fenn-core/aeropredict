#pragma once


namespace aeropredict::integrator
{

template <typename T, typename Vec, typename Func> Vec &rk4(Vec &y, T &t, T h, Func f)
{
    const Vec k1 = f(t, y);
    const Vec k2 = f(t + (h / 2.0), y + (h / 2.0) * k1);
    const Vec k3 = f(t + (h / 2.0), y + (h / 2.0) * k2);
    const Vec k4 = f(t + h, y + h * k3);

    y = y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    t += h;

    return y;
}


} // namespace aeropredict::integrator
