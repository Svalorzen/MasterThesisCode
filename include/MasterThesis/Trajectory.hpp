#ifndef MASTER_THESIS_TRAJECTORY_HEADER_FILE
#define MASTER_THESIS_TRAJECTORY_HEADER_FILE

#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/POMDP/Types.hpp>

template <typename M, typename std::enable_if<AIToolbox::MDP::is_generative_model<M>::value, int>::type = 0>
std::vector<size_t> makeTrajectory(const M& model, unsigned horizon, const AIToolbox::POMDP::Belief & b) {
    using namespace AIToolbox;
    static std::default_random_engine rand(Impl::Seeder::getSeed());

    std::vector<size_t> traj;
    traj.reserve(horizon);

    size_t state = sampleProbability(model.getS(), b, rand);
    for ( unsigned i = 0; i < horizon+1; ++i ) {
        traj.push_back(state);
        std::tie(state, std::ignore) = model.sampleSR(state, 0);
    }

    return traj;
}

#endif
