#ifndef MASTER_THESIS_MAKE_EXPERIMENT_POMCP_HEADER_FILE
#define MASTER_THESIS_MAKE_EXPERIMENT_POMCP_HEADER_FILE

#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/ProbabilityUtils.hpp>

#include <MasterThesis/IO.hpp>
#include <MasterThesis/Utils.hpp>

#include <random>
#include <iostream>
#include <iomanip>
#include <string>

namespace ap = AIToolbox::POMDP;

/*
 * We need to output to file:
 *
 * 1: A file which contains the average cumulative reward over all experiments.
 *
 * We need to output to stdout:
 *
 * 1: The parameters of the experiment
 * 2: The current state of the experiment (\r)
 *    - Current experiment.
 *    - Current timestep of the experiment.
 *    - Current global average reward
 *
 */

template <typename Model, typename Solver>
void makeExperimentPOMCP(
                    unsigned numExperiments,
                    unsigned modelHorizon,   const Model  & model,    const ap::Belief & modelBelief,
                    unsigned solverHorizon,        Solver & solver,   const ap::Belief & solverBelief,
                    const std::string & outputFilename )
{
    static std::default_random_engine rand(AIToolbox::Impl::Seeder::getSeed());

    double totalReward = 0.0;
    std::vector<double> timestepTotalReward(modelHorizon, 0.0);
    double avgReward   = 0.0;

    // We check whether this is a solver for IR problems;
    // if so the rewards are extracted based on the returned
    // actions; otherwise we compute them using the implicit
    // prediction actions for non-IR versions.
    bool usingIR = isSolverIR(1, solver);

    std::cout << numExperiments << " experiments with: POMCP! ";
    if ( usingIR )
        std::cout << "Using IR reward from model.\n";
    else
        std::cout << "Using guess reward.\n";

    std::cout << "Initial Belief: " << printBelief(modelBelief)  << '\n';
    std::cout << "Solver  Belief: " << printBelief(solverBelief) << '\n';

    for ( unsigned experiment = 1; experiment <= numExperiments; ++experiment ) {
        size_t s = AIToolbox::sampleProbability(model.getS(), modelBelief, rand);
        size_t a = solver.sampleAction(solverBelief, std::min(solverHorizon, modelHorizon));

        for ( unsigned i = 1; i <= modelHorizon; ++i ) {
            size_t s1, o; double rew;

            // Extract reward
            if ( usingIR )
                std::tie(s1, o, rew) = model.sampleSOR(s, a);
            else {
                std::tie(s1, o, std::ignore) = model.sampleSOR(s, a);
                rew = ( solver.getGuess() == s );
            }

            totalReward            += rew;
            timestepTotalReward[i] += rew;
            avgReward               = totalReward / (experiment + ((double)i)/modelHorizon);

            std::cout << "[S  = "           << std::setw(3) << s << ']'
                      << "[A  = "           << std::setw(3) << a << ']';
            if ( usingIR ) {
            std::cout << "[AN = "           << std::setw(3) << a / model.getS() << ']'
                      << "[AP = "           << std::setw(3) << a % model.getS() << ']';
            }
            std::cout << "[S1 = "           << std::setw(3) << s1               << ']'
                      << "[O  = "           << std::setw(3) << o                << ']'
                      << "[R  = "           << std::setw(4) << rew              << ']'
                      << "\t\tEXPERIMENT "  << std::setw(4) << experiment
                      << ", TIMESTEP "      << std::setw(4) << i
                      << "\tTotal rew: "    << std::setw(4) << totalReward
                      << "\tAvg: "          << std::setw(4) << avgReward
                      << '\r'               << std::flush;

            // Update states
            s = s1;

            a = solver.sampleAction(a, o, std::min(solverHorizon, modelHorizon - i));
        }

        if ( ! (experiment % 100) )
            gnuplotCumulativeSave(timestepTotalReward, outputFilename);
    }
    gnuplotCumulativeSave(timestepTotalReward, outputFilename);
}

#endif
