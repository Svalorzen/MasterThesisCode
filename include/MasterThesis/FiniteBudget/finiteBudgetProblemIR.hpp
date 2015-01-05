#ifndef MASTER_THESIS_FINITE_BUDGET_IR_MODEL_HEADER_FILE
#define MASTER_THESIS_FINITE_BUDGET_IR_MODEL_HEADER_FILE

#include <cstddef>
#include <tuple>
#include <random>
#include <array>

#include <iostream>

class FiniteBudgetModelIR {
    public:
        FiniteBudgetModelIR(unsigned worldWidth, double leftP, size_t maxBudget, double discount);

        size_t getS() const;
        size_t getA() const;
        size_t getO() const;
        double getDiscount() const;
        std::tuple<size_t, size_t, double> sampleSOR(size_t, size_t) const;

        std::tuple<size_t, double> sampleOR(size_t,size_t, size_t) const;
        // In this class we use sampleSR in order to produce trajectories
        // which are not actually sampled from the true model distribution.
        // This works because sampleSR is not used anywhere else but to
        // make trajectories. The reason we do this is to maintain a random
        // model for the targets, which makes somewhat sense, but to have
        // targets move non-randomly, which also makes sense.
        std::tuple<size_t, double> sampleSR(size_t, size_t) const;

        double getTransitionProbability(size_t, size_t, size_t) const;
        double getObservationProbability(size_t, size_t, size_t) const;
        double getExpectedReward(size_t, size_t, size_t) const;

        // This function is here so that we can keep the
        // same code for performing experiments as CameraPath
        // is using
        size_t getTrueState(size_t s) const { return s; }

        std::pair<size_t, size_t> decodeAction(size_t) const;
        size_t encodeAction(size_t, size_t) const;

        size_t convertToNormalState(size_t) const;
        size_t getRemainingBudget(size_t) const;

        bool isTerminal(size_t) const;
    private:
        size_t sampleTransition(size_t) const;
        // This is the function that creates non-fully-random transitions
        // to make targets move in a believable fashion. The idea is to
        // make each target select a random cell and go there. When he arrives,
        // he selects a new target and so on.
        size_t sampleTrajectoryTransition(size_t) const;
        size_t sampleObservation(size_t, size_t) const;

        size_t makeState(size_t s, size_t budget) const;


        size_t S, A;
        double discount_;
        size_t maxBudget_, worldWidth_;

        double cameraPrecision_, leftProbability_;

        mutable std::default_random_engine rand_;
};

#endif
