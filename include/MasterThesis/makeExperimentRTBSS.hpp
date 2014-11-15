#ifndef MASTER_THESIS_MAKE_EXPERIMENT_RTBSS_HEADER_FILE
#define MASTER_THESIS_MAKE_EXPERIMENT_RTBSS_HEADER_FILE

#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/Impl/Seeder.hpp>
#include <AIToolbox/ProbabilityUtils.hpp>
#include <AIToolbox/POMDP/Utils.hpp>

#include <MasterThesis/IO.hpp>
#include <MasterThesis/Utils.hpp>
#include <MasterThesis/Signals.hpp>

#include <random>
#include <iostream>
#include <iomanip>
#include <fstream>
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
void makeExperimentRTBSS(
                    unsigned numExperiments,
                    unsigned modelHorizon,   const Model  & model,    const ap::Belief & modelBelief,
                    unsigned solverHorizon,        Solver & solver,         ap::Belief   solverBelief,
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

    std::cout << numExperiments << " experiments with: RTBSS! ";
    if ( usingIR )
        std::cout << "Using IR reward from model";
    else
        std::cout << "Using guess reward";

#ifdef ENTROPY
            std::cout << " and ENTROPY\n";
#else
            std::cout << " and MAX OF BELIEF\n";
#endif

    std::cout << "Initial Belief: " << printBelief(modelBelief)  << '\n';
    std::cout << "Solver  Belief: " << printBelief(solverBelief) << '\n';

    auto restartBelief = solverBelief;

    unsigned experiment = 1;
    for ( ; experiment <= numExperiments; ++experiment ) {
        solverBelief = restartBelief;
        size_t s = AIToolbox::sampleProbability(model.getS(), modelBelief, rand);

        for ( unsigned i = 1; i <= modelHorizon; ++i ) {
            size_t s1, a, o; double rew;

            std::tie(a, std::ignore)    = solver.sampleAction(solverBelief, std::min(solverHorizon, modelHorizon - i + 1));
            std::tie(s1, o, rew)        = model.sampleSOR(s, a);

            rew = ifNotIRGuess(rew, s, solver);

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
            // Update belief
            solverBelief = ap::updateBelief(model, solverBelief, a, o);
        }
        if ( processInterrupted ) break;
        if ( ! (experiment % 100) )
            gnuplotCumulativeSave(timestepTotalReward, outputFilename, experiment);
    }
    gnuplotCumulativeSave(timestepTotalReward, outputFilename, std::min(experiment, numExperiments));
}

#endif
