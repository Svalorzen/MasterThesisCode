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

#define STRING(a) #a
#define PRINT(a) std::cout << "###PRINT### " << STRING(a) << " = " << a << "\n";

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
                     "budget     ==> number of available observing actions\n";
        return 0;
    }

    if ( argc < 10 ) {
        std::cout << "Usage: " << argv[0] << " [help] solver worldWidth initState solverHor modelHor iterations k numExp filename budget\n";
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

    double discount = 0.9;

    if ( argc < 7 ) {
        std::cout << "Usage: " << argv[0] << " gridSize pomcpHor horizon numExp samples maxK\n";
        return 0;
    }

    POMDP::Belief belief(worldWidth * (budget + 2), 0.0);
    if ( initState >= worldWidth )
        for ( auto & v : belief )
            v = 1.0 / worldWidth;
    else
        belief[initState] = 1.0;

    // This would set the belief to the center state,
    // no matter the gridSize
    //
    // bool even = !(gridSize % 2);
    // belief[gridSize * (gridSize-1)/2 - even] = 1.0;

    double leftP = 0.6;

    FiniteBudgetModelIR model(worldWidth, leftP, budget, discount);

    PRINT(model.getTransitionProbability(1, 2, 3));
    PRINT(model.getTransitionProbability(1, 2, 2));
    PRINT(model.getTransitionProbability(0, 2, 3));
    PRINT(model.getTransitionProbability(3, 2, 0));
    PRINT(model.getTransitionProbability(3, 2, 3));


    size_t state = 0;
    PRINT((state = std::get<0>(model.sampleSOR(state, 8))));
    PRINT(model.getRemainingBudget(state));
    PRINT(model.convertToNormalState(state));

    PRINT((state = std::get<0>(model.sampleSOR(state, 8))));
    PRINT(model.getRemainingBudget(state));
    PRINT(model.convertToNormalState(state));

    PRINT((state = std::get<0>(model.sampleSOR(state, 8))));
    PRINT(model.getRemainingBudget(state));
    PRINT(model.convertToNormalState(state));

    PRINT((state = std::get<0>(model.sampleSOR(state, 8))));
    PRINT(model.getRemainingBudget(state));
    PRINT(model.convertToNormalState(state));

    return 0;
}
