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
    unsigned k              = std::stoi(argv[7]);
    unsigned numExp         = std::stoi(argv[8]);
    std::string filename    = argv[9];
    unsigned budget         = std::stoi(argv[10]);
    double leftP            = std::stod(argv[11]);

    double discount = 0.9;

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

    FiniteBudgetModel model(worldWidth, leftP, budget, discount);

    PRINT(model.getA());
    PRINT(model.getS());
    PRINT(model.getO());

    size_t s = 0, a;
    for ( int i = 0; i < 50; ++i ) {
        std::cin >> a;
        std::tie(s, std::ignore) = model.sampleSR(s, a);
        model.visualize(s, a);

    }

    return 0;
}
