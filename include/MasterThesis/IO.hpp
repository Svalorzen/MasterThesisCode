#ifndef MASTER_THESIS_IO_HEADER_FILE
#define MASTER_THESIS_IO_HEADER_FILE

#include <AIToolbox/POMDP/Types.hpp>

#include <sstream>
#include <string>
#include <fstream>
#include <iomanip>

std::stringstream printBelief(const AIToolbox::POMDP::Belief & b) {
    std::stringstream output;
    for ( auto & v : b )
        output << "[" << v << "]";
    return output;
}

template <typename T>
void gnuplotCumulativeSave(const T & values, const std::string & filename) {
    std::ofstream file(filename);
    double rew = 0.0;
    int i = 0;
    for ( auto & value : values ) {
        rew += value;
        ++i;

        file << std::setw(4) << i << '\t' << rew << '\n';
    }
}

#endif
