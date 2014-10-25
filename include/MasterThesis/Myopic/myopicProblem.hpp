#ifndef MYOPIC_PROBLEM
#define MYOPIC_PROBLEM

#include <cstddef>
#include <tuple>
#include <random>
#include <array>

#include <iostream>

class MyopicModel {
    public:
        MyopicModel(size_t size, double discount);

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

    private:
        size_t sampleTransition(size_t) const;
        size_t sampleObservation(size_t, size_t) const;

        unsigned cameraSize_;

        // This holds information for each available camera;
        // For each, it stores
        // - [number of observed cells]
        // - [top-left corner x coordinate]
        // - [top-left corner y coordinate]
        // - [width of the field of view]
        std::vector<std::array<unsigned, 4>> cameraData;

        size_t size_, S, A;
        double discount_;

        mutable std::default_random_engine rand_;
};

#endif
