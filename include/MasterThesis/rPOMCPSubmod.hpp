#ifndef MASTER_THESIS_rPOMCP_SUBMODULAR_HEADER_FILE
#define MASTER_THESIS_rPOMCP_SUBMODULAR_HEADER_FILE

#include <AIToolbox/ProbabilityUtils.hpp>
#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/Impl/Seeder.hpp>
#include <unordered_map>
#include <iostream>

#include <MasterThesis/TreeNodes.hpp>

namespace ap = AIToolbox::POMDP;
namespace a = AIToolbox;

#ifndef DOXYGEN_SKIP
        // This is done to avoid bringing around the enable_if everywhere.
        template <typename M, typename = typename std::enable_if<ap::is_generative_model<M>::value>::type>
        class rPOMCPSubmod;
#endif

        /**
         * @brief This class represents the rPOMCP online planner.
         *
         * This class has been modified so that first level actions are all sampled equally, so
         * as to evaluate more precisely values for all of them.
         */
        template <typename M>
        class rPOMCPSubmod<M> {
            public:

                /**
                 * @brief Basic constructor.
                 *
                 * @param m The POMDP model that rPOMCPSubmod will operate upon.
                 * @param beliefSize The size of the initial particle belief.
                 * @param iterations The number of episodes to run before completion.
                 * @param exp The exploration constant. This parameter is VERY important to determine the final rPOMCPSubmod performance.
                 * @param k The number of samples a belief node must have before it switches to MAX. If very very high is nearly equal to mean.
                 */
                rPOMCPSubmod(const M& m, size_t beliefSize, unsigned iterations, double exp, unsigned k = 500);

                size_t getGuess() const;

                /**
                 * @brief This function resets the internal graph and samples for the provided belief and horizon.
                 *
                 * In general it would be better if the belief did not contain
                 * any terminal states; although not necessary, it would
                 * prevent unnecessary work from being performed.
                 *
                 * @param b The initial belief for the environment.
                 * @param horizon The horizon to plan for.
                 *
                 * @return The best action.
                 */
                size_t sampleAction(const ap::Belief& b, unsigned horizon);

                /**
                 * @brief This function uses the internal graph to plan.
                 *
                 * This function can be called after a previous call to
                 * sampleAction with a Belief. Otherwise, it will invoke it
                 * anyway with a random belief.
                 *
                 * If a graph is already present though, this function will
                 * select the branch defined by the input action and
                 * observation, and prune the rest. The search will be started
                 * using the existing graph: this should make search faster,
                 * and also not require any belief updates.
                 *
                 * NOTE: Currently there is no particle reinvigoration
                 * implemented, so for long horizons you can expect
                 * progressively degrading performances.
                 *
                 * @param a The action taken in the last timestep.
                 * @param o The observation received in the last timestep.
                 * @param horizon The horizon to plan for.
                 *
                 * @return The best action.
                 */
                size_t sampleAction(size_t a, size_t o, unsigned horizon);

                /**
                 * @brief This function sets the new size for initial beliefs created from sampleAction().
                 *
                 * Note that this parameter does not bound particle beliefs
                 * created within the tree by result of rollouts: only the ones
                 * directly created from true Beliefs.
                 *
                 * @param beliefSize The new particle belief size.
                 */
                void setBeliefSize(size_t beliefSize);

                /**
                 * @brief This function sets the number of performed rollouts in rPOMCPSubmod.
                 *
                 * @param iter The new number of rollouts.
                 */
                void setIterations(unsigned iter);

                /**
                 * @brief This function sets the new exploration constant for rPOMCPSubmod.
                 *
                 * This parameter is EXTREMELY important to determine rPOMCPSubmod
                 * performance and, ultimately, convergence. In general it is
                 * better to find it empirically, by testing some values and
                 * see which one performs best. Tune this parameter, it really
                 * matters!
                 *
                 * @param exp The new exploration contant.
                 */
                void setExploration(double exp);

                /**
                 * @brief This function returns the POMDP generative model being used.
                 *
                 * @return The POMDP generative model.
                 */
                const M& getModel() const;

                /**
                 * @brief This function returns a reference to the internal graph structure holding the results of rollouts.
                 *
                 * @return The internal graph.
                 */
                const HeadBeliefNode& getGraph() const;

                /**
                 * @brief This function returns the initial particle size for converted Beliefs.
                 *
                 * @return The initial particle count.
                 */
                size_t getBeliefSize() const;

                /**
                 * @brief This function returns the number of iterations performed to plan for an action.
                 *
                 * @return The number of iterations.
                 */
                unsigned getIterations() const;

                /**
                 * @brief This function returns the currently set exploration constant.
                 *
                 * @return The exploration constant.
                 */
                double getExploration() const;

            private:
                const M& model_;
                size_t S, A, beliefSize_;
                unsigned iterations_, maxDepth_;
                double exploration_;
                unsigned k_;
                size_t aCounterTop_;

                mutable std::default_random_engine rand_;

                HeadBeliefNode graph_;

                // Private Methods
                size_t runSimulation(unsigned horizon);
                double simulate(BeliefNode & b, size_t s, unsigned horizon);

                double maxBeliefNodeUpdate(BeliefNode& bn, const ActionNode & aNode, size_t a);

                template <typename Iterator>
                Iterator findBestA(Iterator begin, Iterator end);

                template <typename Iterator>
                Iterator findBestBonusA(Iterator begin, Iterator end, unsigned count);
        };

template <typename M>
rPOMCPSubmod<M>::rPOMCPSubmod(const M& m, size_t beliefSize, unsigned iter, double exp, unsigned k) : model_(m), S(model_.getS()), A(model_.getA()),
                                                                            beliefSize_(beliefSize), iterations_(iter),
                                                                            exploration_(exp), k_(k),
                                                                            rand_(AIToolbox::Impl::Seeder::getSeed()), graph_(A, rand_) {}

template <typename M>
size_t rPOMCPSubmod<M>::sampleAction(const ap::Belief& b, unsigned horizon) {
    // Reset graph
    graph_ = HeadBeliefNode(A, beliefSize_, b, rand_);

    return runSimulation(horizon);
}

template <typename M>
size_t rPOMCPSubmod<M>::getGuess() const {
    return graph_.getMostCommonParticle();
}

template <typename M>
size_t rPOMCPSubmod<M>::sampleAction(size_t a, size_t o, unsigned horizon) {
    auto & obs = graph_.children[a].children;

    auto it = obs.find(o);
    if ( it == obs.end() ) {
        std::cerr << "Observation " << o << " never experienced in simulation, restarting with uniform belief..\n";
        return sampleAction(ap::Belief(S, 1.0 / S), horizon);
    }

    // Here we need an additional step, because *it is contained by graph_.
    // If we just move assign, graph_ is first going to delete everything it
    // contains (included *it), and then we are going to move unallocated memory
    // into graph_! So we move *it outside of the graph_ hierarchy, so that
    // we can then assign safely.
    { BeliefNode tmp = std::move(it->second); graph_ = HeadBeliefNode(A, std::move(tmp), rand_); }

    if ( graph_.isSampleBeliefEmpty() ) {
        std::cerr << "rPOMCPSubmod Lost track of the belief, restarting with uniform..\n";
        return sampleAction(ap::Belief(S, 1.0 / S), horizon);
    }

    return runSimulation(horizon);
}

template <typename M>
size_t rPOMCPSubmod<M>::runSimulation(unsigned horizon) {
    if ( !horizon ) return 0;

    maxDepth_ = horizon;

    aCounterTop_ = 0;
    for (unsigned i = 0; i < iterations_; ++i )
        simulate(graph_, graph_.sampleBelief(), 0);

    auto begin = std::begin(graph_.children);
    return std::distance(begin, findBestA(begin, std::end(graph_.children)));
}

template <typename M>
double rPOMCPSubmod<M>::simulate(BeliefNode & b, size_t s, unsigned depth) {
    b.N++;

    // Select next action node
    size_t a;
    if ( depth == 0 ) {
        // Here we guarantee that all top-level actions are taken the same amount of time.
        a = aCounterTop_;
        aCounterTop_ = (aCounterTop_ + 1) % A;
    }
    else {
        auto begin = std::begin(b.children);
        a = std::distance(begin, findBestBonusA(begin, std::end(b.children), b.N));
    }
    auto & aNode = b.children[a];

    // Generate next step
    size_t s1, o;
    std::tie(s1, o, std::ignore) = model_.sampleSOR(s, a);

    double immAndFutureRew = 0.0;
    {
        typename decltype(aNode.children)::iterator ot;
        bool newNode = false;

        // MODIFICATION: ADD ALL NODES TILL END (NO ROLLOUTS)
        // This either adds a node or sets ot to the existing node.
        ot = aNode.children.find(o);
        if ( ot == aNode.children.end() ) {
            newNode = true;
            std::tie(ot, std::ignore) = aNode.children.insert(std::make_pair(o, BeliefNode()));
        }

        // Compute knowledge for new observation node (entropy/max belief)
        // This needs to be done here since we are going to upgrade a future belief.
        ot->second.updateBeliefAndKnowledge(s1);

        // We only go deeper if needed (maxDepth_ is always at least 1).
        if ( depth + 1 < maxDepth_ && !model_.isTerminal(s1) && !newNode) {
            ot->second.children.resize(A);
            immAndFutureRew = simulate( ot->second, s1, depth + 1 );
        }
        // Otherwise we increase the N for the bottom leaves, since they can't get it otherwise and is needed for entropy
        else {
            ot->second.N += 1;
            // For leaves we still extract entropy
            if ( depth + 1 >= maxDepth_ )
                immAndFutureRew = ot->second.getKnowledgeMeasure();
        }
    }

    // Action update
    aNode.N += 1;
    aNode.V += ( immAndFutureRew - aNode.V ) / static_cast<double>(aNode.N);

    // At this point the current beliefNode has a correct estimate of its
    // own entropy. What it needs to do is select its best action.

    // Here we decide what to transmit to the upper level. In case this
    // node has not been explored enough, then we simply pass on the new
    // datapoint. Otherwise we compute the max over the actions, and we
    // transmit a fake datapoint that will modify the value of the action
    // above as if we chose the best action all the time in the past.
    if ( b.N >= k_ ) {
        // Force looking out for best action
        if ( b.N == k_ ) {
            b.actionsV = HUGE_VAL;
            b.bestAction = a;
        }
        maxBeliefNodeUpdate(b, aNode, a);
    }
    else {
        b.actionsV += ( immAndFutureRew - b.actionsV ) / static_cast<double>(b.N);
    }

    double oldV = b.V;
    // Note that both actionsV than entropy have been modified from last time!
    // We discount the action part since it's the future reward part, while the
    // immediate reward is the direct entropy, which is not discounted
    b.V = model_.getDiscount() * b.actionsV + b.getKnowledgeMeasure();
    // This replaces our old value with the new value in the action update
    return (b.N - 1)*(b.V - oldV) + b.V;
}

template <typename M>
double rPOMCPSubmod<M>::maxBeliefNodeUpdate(BeliefNode& b, const ActionNode & aNode, size_t a) {
    if ( aNode.V >= b.actionsV ) {
        b.actionsV   = aNode.V;
        b.bestAction = a;
    }
    // Note: This is needed because the value may go down!
    else if ( a == b.bestAction ) {
        auto begin = std::begin(b.children);
        auto it = findBestA(begin, std::end(b.children));
        b.actionsV   = it->V;
        b.bestAction = std::distance(begin, it);
    }
}

template <typename M>
template <typename Iterator>
Iterator rPOMCPSubmod<M>::findBestA(Iterator begin, Iterator end) {
    return std::max_element(begin, end, [](const ActionNode & lhs, const ActionNode & rhs){ return lhs.V < rhs.V; });
}

template <typename M>
template <typename Iterator>
Iterator rPOMCPSubmod<M>::findBestBonusA(Iterator begin, Iterator end, unsigned count) {
    // Count here can be as low as 1.
    // Since log(1) = 0, and 0/0 = error, we add 1.0.
    double logCount = std::log(count + 1.0);
    // We use this function to produce a score for each action. This can be easily
    // substituted with something else to produce different rPOMCPSubmod variants.
    auto evaluationFunction = [this, logCount](const ActionNode & an){
        return an.V + exploration_ * std::sqrt( logCount / an.N );
    };

    auto bestIterator = begin++;
    double bestValue = evaluationFunction(*bestIterator);

    for ( ; begin < end; ++begin ) {
        double actionValue = evaluationFunction(*begin);
        if ( actionValue > bestValue ) {
            bestValue = actionValue;
            bestIterator = begin;
        }
    }

    return bestIterator;
}

template <typename M>
void rPOMCPSubmod<M>::setBeliefSize(size_t beliefSize) {
    beliefSize_ = beliefSize;
}

template <typename M>
void rPOMCPSubmod<M>::setIterations(unsigned iter) {
    iterations_ = iter;
}

template <typename M>
void rPOMCPSubmod<M>::setExploration(double exp) {
    exploration_ = exp;
}

template <typename M>
const M& rPOMCPSubmod<M>::getModel() const {
    return model_;
}

template <typename M>
const HeadBeliefNode& rPOMCPSubmod<M>::getGraph() const {
    return graph_;
}

template <typename M>
size_t rPOMCPSubmod<M>::getBeliefSize() const {
    return beliefSize_;
}

template <typename M>
unsigned rPOMCPSubmod<M>::getIterations() const {
    return iterations_;
}

template <typename M>
double rPOMCPSubmod<M>::getExploration() const {
    return exploration_;
}

#endif

