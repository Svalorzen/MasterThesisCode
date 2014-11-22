#include <AIToolbox/ProbabilityUtils.hpp>

#include <AIToolbox/POMDP/Policies/Policy.hpp>

#include <MasterThesis/Myopic/myopicProblem.hpp>
#include <MasterThesis/Myopic/myopicProblemIR.hpp>

#include <AIToolbox/POMDP/Algorithms/POMCP.hpp>
#include <AIToolbox/POMDP/Algorithms/RTBSS.hpp>
#include <MasterThesis/Algorithms/rPOMCP.hpp>
#include <MasterThesis/Algorithms/RTBSSb.hpp>

#include <MasterThesis/makeExperimentPOMCP.hpp>
#include <MasterThesis/makeExperimentRTBSS.hpp>
#include <MasterThesis/Signals.hpp>

#include <iostream>
#include <string>

int main(int argc, char * argv[]) {
    using namespace AIToolbox;
    // We register to this so if the user does Ctrl-C
    // we still save the results on file.
    registerSigInt();

    if ( argc > 1 && std::string(argv[1]) == "help" ) {
        std::cout << "solver     ==> 0: POMCP(IR); 1: rPOMCP; 2: RTBSS(IR); 3: RTBSSb\n"
                     "gridSize   ==> half the states\n"
                     "initState  ==> the initial state, or 2*gridSize for uniform\n"
                     "solverHor  ==> the solver horizon\n"
                     "modelHor   ==> the length of an episode\n"
                     "iterations ==> the number of iterations for POMCP\n"
                     "k          ==> the max trigger for rPOMCP\n"
                     "numExp     ==> number of episodes to do\n"
                     "filename   ==> where to save results\n";
        return 0;
    }

    if ( argc < 10 ) {
        std::cout << "Usage: " << argv[0] << " [help] solver gridSize initState solverHor modelHor iterations k numExp filename\n";
        return 0;
    }

    unsigned solver         = std::stoi(argv[1]);
    unsigned gridSize       = std::stoi(argv[2]);
    unsigned initState      = std::stoi(argv[3]);
    unsigned solverHor      = std::stoi(argv[4]);
    unsigned modelHor       = std::stoi(argv[5]);
    unsigned iterations     = std::stod(argv[6]);
    unsigned k              = std::stod(argv[7]);
    unsigned numExp         = std::stoi(argv[8]);
    std::string filename    = argv[9];

    double discount = 0.9;

    std::cout << "Iterations = " << iterations << "\n";

    auto S = gridSize * 2;
    if ( initState > S ) {
        std::cerr  << "Initial state out is too high!\n";
        return 1;
    }

    POMDP::Belief belief(S, 0.0);

    if ( initState == S )
        for ( auto & v : belief )
            v = 1.0 / S;
    else
        belief[initState] = 1.0;

    switch ( solver ) {
        case 0: {
            auto model = MyopicModelIR(gridSize, discount);
            auto pomcp = POMDP::POMCP<decltype(model)>(model, 1000, iterations, 5);
            makeExperimentPOMCP(numExp, modelHor, model, belief, solverHor, pomcp, belief, filename);
            break;
        }
        case 1: {
            auto model = MyopicModel(gridSize, discount);
            auto pomcp = rPOMCP<decltype(model)>(model, 1000, iterations, 5, k);
            makeExperimentPOMCP(numExp, modelHor, model, belief, solverHor, pomcp, belief, filename);
            break;
        }
        case 2: {
            auto model = MyopicModelIR(gridSize, discount);
            auto rtbss = POMDP::RTBSS<decltype(model)>(model, 1);
            makeExperimentRTBSS(numExp, modelHor, model, belief, solverHor, rtbss, belief, filename);
            break;
        }
        case 3: {
            auto model = MyopicModel(gridSize, discount);
#ifdef ENTROPY
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
            makeExperimentRTBSS(numExp, modelHor, model, belief, solverHor, rtbss, belief, filename);
            break;
        }
    }

    return 0;
}
