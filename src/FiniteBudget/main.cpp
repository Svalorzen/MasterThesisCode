#include <AIToolbox/ProbabilityUtils.hpp>

#include <AIToolbox/POMDP/Algorithms/POMCP.hpp>
#include <AIToolbox/POMDP/Algorithms/RTBSS.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>

#include <MasterThesis/Algorithms/rPOMCP.hpp>
#include <MasterThesis/Algorithms/RTBSSb.hpp>
#include <MasterThesis/FiniteBudget/finiteBudgetProblem.hpp>
#include <MasterThesis/FiniteBudget/finiteBudgetProblemIR.hpp>
#include <MasterThesis/makeExperimentPOMCP.hpp>
#include <MasterThesis/makeExperimentRTBSS.hpp>
#include <MasterThesis/makeMultiExperimentPOMCP.hpp>

#include <iostream>
#include <fstream>

int main(int argc, char * argv[]) {
    using namespace AIToolbox;
    // We register to this so if the user does Ctrl-C
    // we still save the results on file.
    registerSigInt();

    if ( argc > 1 && std::string(argv[1]) == "help" ) {
        std::cout << "solver     ==> 0: POMCP(IR); 1: rPOMCP; 2: RTBSS(IR); 3: RTBSSb\n"
                     "worldWidth ==> width of the room\n"
                     "initState  ==> the initial state, or gridSize^2+1 for uniform\n"
                     "solverHor  ==> the solver horizon\n"
                     "modelHor   ==> the length of an episode\n"
                     "iterations ==> the number of iterations for POMCP\n"
                     "k          ==> the max trigger for rPOMCP\n"
                     "numExp     ==> number of episodes to do\n"
                     "filename   ==> where to save results\n"
                     "budget     ==> number of available observing actions\n"
                     "leftP      ==> probability of the target to transition to the left\n";
        return 0;
    }

    if ( argc < 12 ) {
        std::cout << "Usage: " << argv[0] << " [help] solver worldWidth initState solverHor modelHor iterations k numExp filename budget leftP\n";
        return 0;
    }

    unsigned solver         = std::stoi(argv[1]);
    unsigned worldWidth     = std::stoi(argv[2]);
    unsigned initState      = std::stoi(argv[3]);
    unsigned solverHor      = std::stoi(argv[4]);
    unsigned modelHor       = std::stoi(argv[5]);
    unsigned iterations     = std::stod(argv[6]);
    unsigned k              = std::stod(argv[7]);
    unsigned numExp         = std::stoi(argv[8]);
    std::string filename    = argv[9];
    unsigned budget         = std::stoi(argv[10]);
    double leftP            = std::stod(argv[11]);

    double discount = 0.9;

    POMDP::Belief belief(worldWidth * (budget + 2), 0.0);
    if ( initState >= worldWidth )
        for ( unsigned i = 0; i < worldWidth; ++i )
            belief[i] = 1.0 / worldWidth; // Uniform start with full budget.
    else
        belief[initState] = 1.0;

    switch ( solver ) {
        case 0: {
            auto model = FiniteBudgetModelIR(worldWidth, leftP, budget, discount);
            auto pomcp = POMDP::POMCP<decltype(model)>(model, 1000, iterations, 5);
            makeExperimentPOMCP(numExp, modelHor, model, belief, solverHor, pomcp, belief, filename);
            break;
        }
        case 1: {
            auto model = FiniteBudgetModel(worldWidth, leftP, budget, discount);
            auto pomcp = rPOMCP<decltype(model)>(model, 1000, iterations, 5, k);
            // We use trajectories so targets move in a realistic way
            makeExperimentPOMCP(numExp, modelHor, model, belief, solverHor, pomcp, belief, filename, true);
            break;
        }
        case 2: {
            auto model = FiniteBudgetModelIR(worldWidth, leftP, budget, discount);
            auto rtbss = POMDP::RTBSS<decltype(model)>(model, 1);
            makeExperimentRTBSS(numExp, modelHor, model, belief, solverHor, rtbss, belief, filename);
            break;
        }
        case 3: {
            auto model = FiniteBudgetModel(worldWidth, leftP, budget, discount);
#ifndef ENTROPY
            auto function = [](const POMDP::Belief & b) {
                double e = 0.0;
                for ( auto v : b )
                    if ( checkDifferentSmall(v, 0.0) ) e += v * std::log(v);
                return e;
            };
            auto rtbss = RTBSSb<decltype(model)>(model, 0, function);
#else
            auto function = [](const POMDP::Belief & b) {
                return *std::max_element(std::begin(b), std::end(b));
            };
            auto rtbss = RTBSSb<decltype(model)>(model, 1, function);
#endif
            // We use trajectories so targets move in a realistic way
            makeExperimentRTBSS(numExp, modelHor, model, belief, solverHor, rtbss, belief, filename, true);
            break;
        }
    }

    return 0;
}
