#ifndef MASTER_THESIS_UTILS_HEADER_FILE
#define MASTER_THESIS_UTILS_HEADER_FILE

#include <cstddef>
#include <type_traits>

template <typename Solver, typename = typename std::enable_if<std::is_same<size_t, decltype(((Solver*)nullptr)->getGuess())>::value>::type>
constexpr bool isSolverIR(int, const Solver &) {
    return false;
}

template <typename S>
constexpr bool isSolverIR(double, const S & ) {
    return true;
}

template <typename Solver, typename = typename std::enable_if<std::is_same<size_t, decltype(((Solver*)nullptr)->getGuess())>::value>::type>
double ifNotIRGuess(double, size_t s, const Solver & solver) {
    return solver.getGuess() == s;
}

template <typename S>
double ifNotIRGuess(double r, double, const S & ) {
    return r;
}

#endif
