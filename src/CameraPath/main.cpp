#include <AIToolbox/ProbabilityUtils.hpp>

#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>

#include <MasterThesis/Algorithms/rPOMCP.hpp>
#include <MasterThesis/Algorithms/RTBSSb.hpp>
#include <MasterThesis/CameraPath/cameraPathProblem.hpp>
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
        std::cout << "solver     ==> 1: rPOMCP; 3: RTBSSb; 4: rPOMCP multi\n"
                     "gridSize   ==> width/height of the room\n"
                     "initState  ==> the initial state, or gridSize^2+1 for uniform\n"
                     "solverHor  ==> the solver horizon\n"
                     "modelHor   ==> the length of an episode\n"
                     "iterations ==> the number of iterations for POMCP\n"
                     "k          ==> the max trigger for rPOMCP\n"
                     "numExp     ==> number of episodes to do\n"
                     "filename   ==> where to save results\n"
                     "[nrPpl]    ==> number of people in multi experiment\n";
        return 0;
    }

    if ( argc < 10 ) {
        std::cout << "Usage: " << argv[0] << " [help] solver gridSize initState solverHor modelHor iterations k numExp filename nrPpl\n";
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
    unsigned nrPpl          = 1;

    if ( solver == 4 ) {
        if ( argc < 11 ) {
            std::cout << "Usage: " << argv[0] << " [help] solver gridSize initState solverHor modelHor iterations k numExp filename nrPpl\n";
            return 0;
        }
        nrPpl               = std::stoi(argv[10]);
    }

    double discount = 0.9;

    if ( argc < 7 ) {
        std::cout << "Usage: " << argv[0] << " gridSize pomcpHor horizon numExp samples maxK\n";
        return 0;
    }

    CameraPathModel model(gridSize, discount);
    size_t S = model.getS();

    POMDP::Belief belief(S, 0.0);
    if ( initState == S )
        for ( auto & v : belief )
            v = 1.0 / S;
    else
        belief[initState] = 1.0;

    // This would set the belief to the center state,
    // no matter the gridSize
    //
    // bool even = !(gridSize % 2);
    // belief[gridSize * (gridSize-1)/2 - even] = 1.0;

    switch ( solver ) {
        case 1: {
            auto pomcp = rPOMCP<decltype(model)>(model, 1000, iterations, 5, k);
            // We use trajectories so targets move in a realistic way
            makeExperimentPOMCP(numExp, modelHor, model, belief, solverHor, pomcp, belief, filename, true);
            break;
        }
        case 3: {
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
            // We use trajectories so targets move in a realistic way
            makeExperimentRTBSS(numExp, modelHor, model, belief, solverHor, rtbss, belief, filename, true);
            break;
        }
        case 4: {
            std::vector<rPOMCP<decltype(model)>> solvers;
            solvers.reserve(nrPpl);
            for ( unsigned i = 0; i < nrPpl; ++i )
                solvers.emplace_back(model, 1000, iterations, 5, k);

            makeMultiExperimentPOMCP(numExp, nrPpl, modelHor, model, belief, solverHor, solvers, belief, filename, true);
            break;
        }
    }

    return 0;
}
