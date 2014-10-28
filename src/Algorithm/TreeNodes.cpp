#include <MasterThesis/Algorithms/Utils/TreeNodes.hpp>

#include <iostream>

BeliefNode::BeliefNode() : N(0), V(0.0), actionsV(0.0), bestAction(0), knowledgeMeasure_(0.0) {
#ifndef ENTROPY
    maxS_ = 0;
#endif
}

#ifdef ENTROPY
// Note for ENTROPY implementation:
// In theory this is wrong as we should update all the entropy terms, one
// for each different type of particle. In practice we hope this will work
// anyway, and that there are not going to be huge problems, as each particle
// should be seen enough times to still keep a decent approximation of its
// entropy term. Minor errors are ok since this is still an estimation.
void BeliefNode::updateBeliefAndKnowledge(size_t s) {
    // Remove entropy term for this state from summatory
    knowledgeMeasure_ -= trackBelief_[s].negativeEntropy;
    // Updating belief
    trackBelief_[s].N += 1;
    // Computing new entropy term for this state
    double p = static_cast<double>(trackBelief_[s].N) / static_cast<double>(N+1);
    double newEntropy = p * std::log(p);
    // Update values
    trackBelief_[s].negativeEntropy = newEntropy;
    knowledgeMeasure_ += newEntropy;
}

#else

// This is the Max-Belief implementation
void BeliefNode::updateBeliefAndKnowledge(size_t s) {
    trackBelief_[s].N += 1;

    if ( trackBelief_[s].N > trackBelief_[maxS_].N )
        maxS_ = s;

    knowledgeMeasure_ = static_cast<double>(trackBelief_[maxS_].N) / static_cast<double>(N+1);
}

#endif

double BeliefNode::getKnowledgeMeasure() const {
    return knowledgeMeasure_;
}

HeadBeliefNode::HeadBeliefNode(size_t A, std::default_random_engine & rand) : BeliefNode(), rand_(&rand) {
    children.resize(A);
}

HeadBeliefNode::HeadBeliefNode(size_t A, size_t beliefSize, const AIToolbox::POMDP::Belief & b, std::default_random_engine & rand) : BeliefNode(), rand_(&rand), beliefSize_(beliefSize) {
    children.resize(A);
    std::unordered_map<size_t, unsigned> generatedSamples;

    size_t S = b.size();
    for ( size_t i = 0; i < beliefSize_; ++i )
        generatedSamples[AIToolbox::sampleProbability(S, b, *rand_)] += 1;

    sampleBelief_.reserve(beliefSize_);
    for ( auto & pair : generatedSamples ) {
        sampleBelief_.emplace_back(pair);
        // Compute entropy here since we don't have a parent in this case (is it really needed?)
        // double p = static_cast<double>(pair.second) / static_cast<double>(beliefSize_);
        // negativeEntropy += p * std::log(p);
    }
}

HeadBeliefNode::HeadBeliefNode(size_t A, BeliefNode && bn, std::default_random_engine& rand) : BeliefNode(std::move(bn)), rand_(&rand), beliefSize_(0) {
    children.resize(A);
    sampleBelief_.reserve(trackBelief_.size());
    for ( auto & pair : trackBelief_ ) {
        sampleBelief_.emplace_back(pair.first, pair.second.N);
        beliefSize_ += pair.second.N;
    }
    TrackBelief().swap(trackBelief_); // Clear belief memory
}

bool HeadBeliefNode::isSampleBeliefEmpty() const {
    return sampleBelief_.empty();
}

size_t HeadBeliefNode::sampleBelief() const {
    std::uniform_int_distribution<unsigned> generator(1, beliefSize_);
    int pick = generator(*rand_);

    size_t index = 0;
    while (true) {
        pick -= sampleBelief_[index].second;
        if ( pick < 1 ) return sampleBelief_[index].first;
        ++index;
    }
}

size_t HeadBeliefNode::getMostCommonParticle() const {
    // We return the most common particle in the head belief
    size_t bestGuess; unsigned bestGuessCount = 0;
    for ( auto & pair : sampleBelief_ ) {
        if ( pair.second > bestGuessCount ) {
            bestGuessCount = pair.second;
            bestGuess = pair.first;
        }
    }
    return bestGuess;
}

void HeadBeliefNode::printSampleBelief() const {
    std::cout << "State\t | Count\n";
    std::cout << "-----------------------------------\n";
    for ( auto & pair : sampleBelief_ ) {
        std::cout << pair.first << "\t | " << pair.second << "\n";
    }
            std::cout << "-----------------------------------\n";
}

void BeliefNode::printTrackBelief() const {
    std::cout << "Track Belief:\n";
    for ( auto & pair : trackBelief_ ) {
        std::cout << "State: " << pair.first << ", Count: " << pair.second.N << "\n";
    }
}
