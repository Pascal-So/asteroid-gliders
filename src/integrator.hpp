#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

namespace integrator {
    template <typename VectorSpace, typename GradientFunc, typename Scalar>
    VectorSpace explicitEuler(const VectorSpace start, GradientFunc f, const Scalar stepsize) {
        const auto k1 = stepsize * f(start);

        return start + k1;
    }

    template <typename VectorSpace, typename GradientFunc, typename Scalar>
    VectorSpace midpoint(const VectorSpace start, GradientFunc f, const Scalar stepsize) {
        const auto k1 = stepsize * f(start);
        const auto k2 = stepsize * f(start + k1 / 2);

        return start + k2;
    }

    template <typename VectorSpace, typename GradientFunc, typename Scalar>
    VectorSpace rungeKutta4(const VectorSpace start, GradientFunc f, const Scalar stepsize) {
        const auto k1 = stepsize * f(start);
        const auto k2 = stepsize * f(start + k1 / 2);
        const auto k3 = stepsize * f(start + k2 / 2);
        const auto k4 = stepsize * f(start + k3);

        return start + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    }
}

#endif // INTEGRATOR_HPP
