#include <MasterThesis/FiniteBudget/finiteBudgetProblemIR.hpp>

#include <cassert>
#include <AIToolbox/Impl/Seeder.hpp>

FiniteBudgetModelIR::FiniteBudgetModelIR(unsigned worldWidth, double leftP, size_t maxBudget, double d) : S(worldWidth * (maxBudget+2)), A(worldWidth * (worldWidth + 1)), discount_(d),
                                                                                                      maxBudget_(maxBudget), worldWidth_(worldWidth), rand_(AIToolbox::Impl::Seeder::getSeed())
{
    if ( worldWidth < 1 ) throw std::invalid_argument("This grid size is not allowed: " + std::to_string(worldWidth));
    cameraPrecision_ = 0.8;
    leftProbability_ = leftP;
}

// INFO FUNCTIONS
size_t FiniteBudgetModelIR::getS() const { return S; }

size_t FiniteBudgetModelIR::getA() const { return A; }
size_t FiniteBudgetModelIR::getO() const { return 2; }
double FiniteBudgetModelIR::getDiscount() const { return discount_; }

// SAMPLING FUNCTIONS

std::tuple<size_t, size_t, double> FiniteBudgetModelIR::sampleSOR(size_t s, size_t a) const {
    auto trueS = convertToNormalState(s);
    auto budget = getRemainingBudget(s);

    size_t trueS1 = sampleTransition(trueS);

    size_t an, ap;
    std::tie(an, ap) = decodeAction(a);

    auto s1 = makeState(trueS1, an == worldWidth_ ? budget : budget + 1);

    size_t o = sampleObservation(s1, an);

    return std::make_tuple(s1, o, ap == s);
}

std::tuple<size_t, double> FiniteBudgetModelIR::sampleOR(size_t s, size_t a, size_t s1) const {
    size_t an, ap;
    std::tie(an, ap) = decodeAction(a);
    return std::make_tuple(sampleObservation(s1, an), s == ap);
}

std::tuple<size_t, double> FiniteBudgetModelIR::sampleSR(size_t s, size_t a) const {
    auto trueS = convertToNormalState(s);
    auto budget = getRemainingBudget(s);

    size_t an, ap;
    std::tie(an, ap) = decodeAction(a);

    size_t trueS1 = sampleTrajectoryTransition(trueS);
    auto s1 = makeState(trueS1, an == worldWidth_ ? budget : budget + 1);
    return std::make_tuple(s1, s == ap);
}

// IMPLEMENTATIONS

// TAKES TRUE STATE
size_t FiniteBudgetModelIR::sampleTransition(size_t s) const {
    static std::uniform_real_distribution<double> prob(0, 1);

    auto dice = prob(rand_);
    // Moving to the right..
    if ( dice > leftProbability_ )
        return (s+1)%worldWidth_;
    // To the left
    return (s-1+worldWidth_)%worldWidth_;
}

size_t FiniteBudgetModelIR::sampleTrajectoryTransition(size_t s) const {
    return sampleTransition(s);
}

// TAKES FAKE STATE
size_t FiniteBudgetModelIR::sampleObservation(size_t s1, size_t a) const {
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

double FiniteBudgetModelIR::getObservationProbability(size_t s1, size_t a, size_t o) const {
    auto budget = getRemainingBudget(s1);
    // If there's no more budget, we return randomly
    if ( budget > maxBudget_ || a == worldWidth_ ) return 0.5;

    auto trueS1 = convertToNormalState(s1);
    // If we didn't see the person..
    if ( o == 0 )
        return trueS1 == a ? 1.0 - cameraPrecision_ : cameraPrecision_;

    return trueS1 == a ? cameraPrecision_ : 1.0 - cameraPrecision_;
}

// Changed from Basic
double FiniteBudgetModelIR::getTransitionProbability(size_t s, size_t, size_t s1) const {
    auto trueS = convertToNormalState(s);
    auto trueS1 = convertToNormalState(s1);

    if ( trueS1 == (trueS - 1 + worldWidth_)%worldWidth_ )
        return leftProbability_;
    if ( trueS1 == (trueS + 1)%worldWidth_ )
        return 1-leftProbability_;
    return 0.0;
}

double FiniteBudgetModelIR::getExpectedReward(size_t s, size_t a, size_t) const {
    size_t ap;
    std::tie(std::ignore, ap) = decodeAction(a);

    return s == ap;
}

bool FiniteBudgetModelIR::isTerminal(size_t) const { return false; }

size_t FiniteBudgetModelIR::convertToNormalState(size_t s) const {
    return s % worldWidth_;
}

size_t FiniteBudgetModelIR::getRemainingBudget(size_t s) const {
    return s / worldWidth_;
}

size_t FiniteBudgetModelIR::makeState(size_t s, size_t b) const {
    b = std::min(b, maxBudget_+1);
    assert(s < worldWidth_);
    return s + b * worldWidth_;
}

std::pair<size_t, size_t> FiniteBudgetModelIR::decodeAction(size_t a) const {
    return std::make_pair(a % (worldWidth_+1), a / (worldWidth_+1));
}

size_t FiniteBudgetModelIR::encodeAction(size_t an, size_t ap) const {
    return an + ap * (worldWidth_+1);
}
