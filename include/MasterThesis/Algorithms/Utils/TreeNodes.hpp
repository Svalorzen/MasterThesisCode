#ifndef MASTER_THESIS_TREE_NODES_HEADER_FILE
#define MASTER_THESIS_TREE_NODES_HEADER_FILE

#include <vector>
#include <unordered_map>

#include <AIToolbox/ProbabilityUtils.hpp>
#include <AIToolbox/POMDP/Types.hpp>

// #include <boost/pool/pool_alloc.hpp>

struct ActionNode;
using ActionNodes = std::vector<ActionNode>;

struct BeliefParticle {
    unsigned N = 0;             // Number of particles for this particular type (state)
#ifdef ENTROPY
    double negativeEntropy = 0; // Estimated entropy deriving from this particle type
#endif
};

// This is used to keep track of beliefs down in the tree. We use a map since
// we do not need to sample from here, just to access fast and recompute the
// entropy values.
using TrackBelief  = std::unordered_map<
                                    size_t, 
                                    BeliefParticle,
                                    std::hash<size_t>,
                                    std::equal_to<size_t>
                                    //,boost::fast_pool_allocator<std::pair<size_t, BeliefParticle>>
                                    >;

class BeliefNode {
    public:
        BeliefNode();

        // This function updates the knowledge measure after adding a new belief particle.
        void updateBeliefAndKnowledge(size_t s);    

        // This function returns the current estimate for reward for this node.
        double getKnowledgeMeasure() const;         

        unsigned N;          // Counter for number of times we went through this belief node.
        ActionNodes children;

        double V;            // Estimated value for this belief, taking into account future rewards/actions.
        double actionsV;     // Estimated value for the actions (could be mean, max, or other)
        size_t bestAction;   // Tracker of best available action in MAX-mode, to select node value.

        void printTrackBelief() const;

    protected:
        TrackBelief trackBelief_; // This is a particle belief which is easy to update

        double knowledgeMeasure_; // Estimated entropy for this belief.
#ifndef ENTROPY
        size_t maxS_;             // This keeps track of the belief peak state for max of belief
#endif
};

using BeliefNodes = std::unordered_map<size_t, BeliefNode>;

struct ActionNode {
    BeliefNodes children;
    double V       = 0.0; // Tracks the value of the action, as a weighted
    // average of the values of the next step beliefNodes.
    unsigned N     = 0;
};

// This is used to sample at the top of the tree. It is a vector containing a
// state-count pair for each particle.
using SampleBelief = std::vector<std::pair<size_t, unsigned>>;

// This converts the unordered belief map of an ordinary belief node into a vector.
// This should speed up the sampling process considerably.
class HeadBeliefNode : public BeliefNode {
    public:
        HeadBeliefNode(size_t A, std::default_random_engine & rand);
        HeadBeliefNode(size_t A, size_t beliefSize, const AIToolbox::POMDP::Belief & b, std::default_random_engine & rand);
        HeadBeliefNode(size_t A, BeliefNode && bn, std::default_random_engine & rand);

        bool isSampleBeliefEmpty() const;
        size_t sampleBelief() const;
        size_t getMostCommonParticle() const;
        void printSampleBelief() const;

    private:
        std::default_random_engine * rand_; // We use POMCP one;
        SampleBelief sampleBelief_;         // This is a particle belief which is easy to sample
        size_t beliefSize_;                 // This is the total number of particles for this belief, needed because of SampleBelief structure
};

#endif
