#include <MasterThesis/FiniteBudget/finiteBudgetProblem.hpp>

#include <cassert>
#include <AIToolbox/Impl/Seeder.hpp>

FiniteBudgetModel::FiniteBudgetModel(unsigned worldWidth, double leftP, size_t maxBudget, double d) : S(worldWidth * (maxBudget+2)), A(worldWidth + 1), discount_(d),
                                                                                                      maxBudget_(maxBudget), worldWidth_(worldWidth), rand_(AIToolbox::Impl::Seeder::getSeed())
{
    if ( worldWidth < 1 ) throw std::invalid_argument("This grid size is not allowed: " + std::to_string(worldWidth));
    cameraPrecision_ = 0.8;
    leftProbability_ = leftP;
}

// INFO FUNCTIONS
size_t FiniteBudgetModel::getS() const { return S; }

size_t FiniteBudgetModel::getA() const {
    return A;
}

size_t FiniteBudgetModel::getO() const { return 2; }
double FiniteBudgetModel::getDiscount() const { return discount_; }

// SAMPLING FUNCTIONS

std::tuple<size_t, size_t, double> FiniteBudgetModel::sampleSOR(size_t s, size_t a) const {
    auto trueS = convertToNormalState(s);
    auto budget = getRemainingBudget(s);

    std::cout << "SAMPLING TRANSITION WITH TRUES = " << trueS << "\n";
    size_t trueS1 = sampleTransition(trueS);
    std::cout << "RESULT IS TRUES1 = " << trueS1 << "\n";
    auto s1 = makeState(trueS1, a == worldWidth_ ? budget : budget + 1);

    size_t o = sampleObservation(s1, a);

    return std::make_tuple(s1, o, 0.0);
}

std::tuple<size_t, double> FiniteBudgetModel::sampleOR(size_t, size_t a, size_t s1) const {
    return std::make_tuple(sampleObservation(s1, a), 0.0);
}

std::tuple<size_t, double> FiniteBudgetModel::sampleSR(size_t s, size_t a) const {
    auto trueS = convertToNormalState(s);
    auto budget = getRemainingBudget(s);

    size_t trueS1 = sampleTrajectoryTransition(trueS);
    auto s1 = makeState(trueS1, a == S ? budget : budget + 1);
    return std::make_tuple(s1, 0.0);
}

// IMPLEMENTATIONS

// TAKES TRUE STATE
size_t FiniteBudgetModel::sampleTransition(size_t s) const {
    static std::uniform_real_distribution<double> prob(0, 1);

    auto dice = prob(rand_);
    // Moving to the right..
    if ( dice > leftProbability_ )
        return (s+1)%worldWidth_;
    // To the left
    return (s-1+worldWidth_)%worldWidth_;
}

size_t FiniteBudgetModel::sampleTrajectoryTransition(size_t s) const {
    return sampleTransition(s);
}

// TAKES FAKE STATE
size_t FiniteBudgetModel::sampleObservation(size_t s1, size_t a) const {
    auto budget = getRemainingBudget(s1);
    static std::uniform_real_distribution<double> prob(0, 1);

    auto dice = prob(rand_);
    // If there's no more budget, or we don't look..
    if ( budget > maxBudget_ || a == worldWidth_ ) {
        std::cout << ( budget > maxBudget_ ? "OverBudget\n" : "Not Looking\n");
        return dice > 0.5;
    }

    auto trueS1 = convertToNormalState(s1);
    // If we look in the right place..
    if ( a == trueS1 )
        return dice <= cameraPrecision_;

    return dice > cameraPrecision_;
}

// PROBABILITIES

double FiniteBudgetModel::getObservationProbability(size_t s1, size_t a, size_t o) const {
    auto budget = getRemainingBudget(s1);
    // If there's no more budget, we return randomly
    if ( budget > maxBudget_ ) return 0.5;

    auto trueS1 = convertToNormalState(s1);
    // If we didn't see the person..
    if ( o == 0 )
        return trueS1 == a ? 1.0 - cameraPrecision_ : cameraPrecision_;

    return trueS1 == a ? cameraPrecision_ : 1.0 - cameraPrecision_;
}

// Changed from Basic
double FiniteBudgetModel::getTransitionProbability(size_t s, size_t, size_t s1) const {
    auto trueS = convertToNormalState(s);
    auto trueS1 = convertToNormalState(s1);

    if ( trueS1 == (trueS - 1 + worldWidth_)%worldWidth_ )
        return leftProbability_;
    if ( trueS1 == (trueS + 1)%worldWidth_ )
        return 1-leftProbability_;
    return 0.0;
}

bool FiniteBudgetModel::isTerminal(size_t) const { return false; }

size_t FiniteBudgetModel::convertToNormalState(size_t s) const {
    return s % worldWidth_;
}

size_t FiniteBudgetModel::getRemainingBudget(size_t s) const {
    return s / worldWidth_;
}

size_t FiniteBudgetModel::makeState(size_t s, size_t b) const {
    std::cout << "MAKING STATE WITH S = " << s << " AND BUDGET = " << b <<"\n";
    b = std::min(b, maxBudget_+1);
    std::cout << "RESULT IS " << s + b * worldWidth_ << "\n";
    return s + b * worldWidth_;
}
