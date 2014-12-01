#ifndef MASTER_THESIS_MYOPIC_MODEL_IR_HEADER_FILE
#define MASTER_THESIS_MYOPIC_MODEL_IR_HEADER_FILE

#include <cstddef>
#include <tuple>
#include <random>

class MyopicModelIR {
    public:
        MyopicModelIR(size_t size, double discount);

        size_t getS() const;
        size_t getA() const;
        size_t getO() const;
        double getDiscount() const;

        std::tuple<size_t, size_t, double> sampleSOR(size_t, size_t) const;
        std::tuple<size_t, double> sampleSR(size_t, size_t) const;
        bool isTerminal(size_t) const;

        double getTransitionProbability(size_t, size_t, size_t) const;
        double getObservationProbability(size_t, size_t, size_t) const;
        double getExpectedReward(size_t, size_t, size_t) const;

        std::tuple<size_t, double> sampleOR(size_t,size_t, size_t) const;

        std::pair<size_t, size_t> decodeAction(size_t) const;
        size_t encodeAction(size_t, size_t) const;

        // This function is here so that we can keep the
        // same code for performing experiments as CameraPath
        // is using
        size_t getTrueState(size_t s) const { return s; }
    private:
        size_t sampleTransition(size_t) const;
        size_t sampleObservation(size_t, size_t) const;

        size_t size_, S, A;
        double discount_;

        mutable std::default_random_engine rand_;
};

#endif
