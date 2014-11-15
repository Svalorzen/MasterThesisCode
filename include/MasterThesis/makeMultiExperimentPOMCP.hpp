#ifndef MASTER_THESIS_MAKE_MULTI_EXPERIMENT_POMCP_HEADER_FILE
#define MASTER_THESIS_MAKE_MULTI_EXPERIMENT_POMCP_HEADER_FILE

#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/ProbabilityUtils.hpp>

#include <MasterThesis/IO.hpp>
#include <MasterThesis/Signals.hpp>

#include <cstddef>
#include <vector>
#include <algorithm>
#include <random>
#include <iostream>
#include <iomanip>
#include <string>

namespace ap = AIToolbox::POMDP;

// This extracts the action that maximizes value across all solvers.
template <typename POMCP>
size_t extractAction(const std::vector<POMCP> & solvers) {
    size_t A = solvers[0].getModel().getA();
    std::vector<double> values(A, 0.0);

    for ( const auto & solver : solvers ) {
        for ( size_t a = 0; a < A; ++a )
            values[a] += solver.getGraph().children[a].V;
    }
    auto x = std::distance(std::begin(values), std::max_element(std::begin(values), std::end(values)));
    return x;
}

template <typename Model, typename Solver>
void makeMultiExperimentPOMCP(
                    unsigned numExperiments, unsigned numTargets,
                    unsigned modelHorizon,   const Model  & model,            const ap::Belief & modelBelief,
                    unsigned solverHorizon,  std::vector<Solver> & solvers,   const ap::Belief & solverBelief,
                    const std::string & outputFilename )
{
    static std::default_random_engine rand(AIToolbox::Impl::Seeder::getSeed());

    double totalReward = 0.0;
    std::vector<double> timestepTotalReward(modelHorizon, 0.0);
    double avgReward   = 0.0;

    std::vector<size_t> obs(numTargets), pos(numTargets);

    std::cout << numExperiments << " experiments with: Submodular POMCP! ";

    std::cout << "Initial Belief: " << printBelief(modelBelief)  << '\n';
    std::cout << "Solver  Belief: " << printBelief(solverBelief) << '\n';

    unsigned experiment = 1;
    for ( ; experiment <= numExperiments; ++experiment ) {
        // Run pomcp, but don't get actions yet
        for ( unsigned p = 0; p < numTargets; ++p ) {
            pos[p] = AIToolbox::sampleProbability(model.getS(), modelBelief, rand);
            solvers[p].sampleAction(solverBelief, std::min(solverHorizon, modelHorizon));
        }
        // Extract action
        size_t a = extractAction(solvers);

        for ( unsigned i = 1; i <= modelHorizon; ++i ) {
            double rew;
            // Extract observations and rewards
            for ( unsigned p = 0; p < numTargets; ++p ) {
                std::tie(pos[p], obs[p], std::ignore) = model.sampleSOR( pos[p], a );

                rew += ( solvers[p].getGuess() == pos[p] );
            }

            totalReward            += rew;
            timestepTotalReward[i] += rew;
            avgReward               = totalReward / (experiment + ((double)i)/modelHorizon);

            std::cout // << "[S = " << s << "][A = " << a << "][S1 = " << s1 << "][ O = " << o << " ][ R = " << rew << " ]"
                      << "EXPERIMENT "  << std::setw(4) << experiment
                      << ", TIMESTEP "      << std::setw(4) << i
                      << "\tTotal rew: "    << std::setw(4) << totalReward
                      << "\tAvg: "          << std::setw(4) << avgReward
                      << '\r'               << std::flush;

            for ( unsigned p = 0; p < numTargets; ++p )
                solvers[p].sampleAction(a, obs[p], std::min(solverHorizon, modelHorizon - i));

            a = extractAction(solvers);
        }
        if ( processInterrupted ) break;
        if ( ! (experiment % 100) )
            gnuplotCumulativeSave(timestepTotalReward, outputFilename, experiment);
    }
    gnuplotCumulativeSave(timestepTotalReward, outputFilename, std::min(experiment, numExperiments));
}

#endif
