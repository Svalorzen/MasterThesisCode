#include <MasterThesis/Myopic/myopicProblemIR.hpp>

#include <AIToolbox/Impl/Seeder.hpp>

MyopicModelIR::MyopicModelIR(size_t s, double d) : size_(s), S(size_ * 2), A(S*S), discount_(d), rand_(AIToolbox::Impl::Seeder::getSeed())  {
    if ( size_ < 1 ) throw std::invalid_argument("This size is not allowed: " + std::to_string(size_));
}

size_t MyopicModelIR::getS() const { return S; }
size_t MyopicModelIR::getA() const { return A; }
size_t MyopicModelIR::getO() const { return 2; }
double MyopicModelIR::getDiscount() const { return discount_; }

std::tuple<size_t, size_t, double> MyopicModelIR::sampleSOR(size_t s, size_t a) const {
    size_t s1 = sampleTransition(s);

    size_t an, ap;
    std::tie(an, ap) = decodeAction(a);

    size_t o = sampleObservation(s1, an);

    return std::make_tuple(s1, o, s == ap);
}

std::tuple<size_t, double> MyopicModelIR::sampleOR(size_t s, size_t a, size_t s1) const {
    size_t an, ap;
    std::tie(an, ap) = decodeAction(a);
    return std::make_tuple(sampleObservation(s1, an), s == ap);
}

size_t MyopicModelIR::sampleTransition(size_t s) const {
    static std::uniform_int_distribution<unsigned> dist(0, size_-1);

    // Random side
    if ( s < size_ ) return dist(rand_);

    // Deterministic side
    return (((( s - size_ ) + 1) % size_ ) + size_);
}

size_t MyopicModelIR::sampleObservation(size_t s1, size_t an) const {
    static std::uniform_int_distribution<unsigned> dist1(1, 5);
    // 0.2 chance of failing
    bool cameraWorks = (dist1(rand_) != 5);

    if ( cameraWorks ) {
        return s1 == an;
    }
    else {
        return s1 != an;
    }
}

std::tuple<size_t, double> MyopicModelIR::sampleSR(size_t s, size_t a) const {
    size_t ap;
    std::tie(std::ignore, ap) = decodeAction(a);
    return std::make_tuple(sampleTransition(s), s == ap);
}

bool MyopicModelIR::isTerminal(size_t) const { return false; }


double MyopicModelIR::getTransitionProbability(size_t s, size_t, size_t s1) const {
    // If they are both in the random part, it's possible
    if ( s < size_ && s1 < size_ ) return 1.0/size_;
    // If they are in different parts, no way
    if ( s < size_ || s1 < size_ ) return 0.0;

    if ( s1 == (((( s - size_ ) + 1) % size_ ) + size_) ) return 1.0;
    return 0.0;
}

double MyopicModelIR::getObservationProbability(size_t s, size_t a, size_t o) const {
    size_t an;
    std::tie(an, std::ignore) = decodeAction(a);

    if ( s == an ) return o == 1 ? 0.8 : 0.2;
    return o == 1 ? 0.2 : 0.8;
}

double MyopicModelIR::getExpectedReward(size_t s, size_t a, size_t) const {
    size_t ap;
    std::tie(std::ignore, ap) = decodeAction(a);

    return s == ap;
}

std::pair<size_t, size_t> MyopicModelIR::decodeAction(size_t a) const {
    return std::make_pair(a / S, a % S);
}

size_t MyopicModelIR::encodeAction(size_t an, size_t ap) const {
    return ap + an * S;
}
