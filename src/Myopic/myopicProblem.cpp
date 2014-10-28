#include <MasterThesis/Myopic/myopicProblem.hpp>

#include <AIToolbox/Impl/Seeder.hpp>

MyopicModel::MyopicModel(size_t s, double d) : size_(s), S(size_ * 2), A(S), discount_(d), rand_(AIToolbox::Impl::Seeder::getSeed())  {
    if ( size_ < 1 ) throw std::invalid_argument("This size is not allowed: " + std::to_string(size_));
}

size_t MyopicModel::getS() const { return S; }
size_t MyopicModel::getA() const { return A; }
size_t MyopicModel::getO() const { return 2; }
double MyopicModel::getDiscount() const { return discount_; }

std::tuple<size_t, size_t, double> MyopicModel::sampleSOR(size_t s, size_t a) const {
    size_t s1 = sampleTransition(s);
    size_t o = sampleObservation(s1, a);

    return std::make_tuple(s1, o, 0.0);
}

std::tuple<size_t, double> MyopicModel::sampleOR(size_t, size_t a, size_t s1) const {
    return std::make_tuple(sampleObservation(s1, a), 0.0);
}

size_t MyopicModel::sampleTransition(size_t s) const {
    static std::uniform_int_distribution<unsigned> dist(0, size_-1);

    // Random side
    if ( s < size_ ) return dist(rand_);

    // Deterministic side
    return (((( s - size_ ) + 1) % size_ ) + size_);
}

size_t MyopicModel::sampleObservation(size_t s1, size_t a) const {
    static std::uniform_int_distribution<unsigned> dist1(1, 5);
    // 0.2 chance of failing
    bool cameraWorks = (dist1(rand_) != 5);

    if ( cameraWorks ) {
        return s1 == a;
    }
    else {
        return s1 != a;
    }
}

std::tuple<size_t, double> MyopicModel::sampleSR(size_t s, size_t) const {
    return std::make_tuple(sampleTransition(s), 0.0);
}

bool MyopicModel::isTerminal(size_t) const { return false; }


double MyopicModel::getTransitionProbability(size_t s, size_t, size_t s1) const {
    // If they are both in the random part, it's possible
    if ( s < size_ && s1 < size_ ) return 1.0/size_;
    // If they are in different parts, no way
    if ( s < size_ || s1 < size_ ) return 0.0;

    if ( s1 == (((( s - size_ ) + 1) % size_ ) + size_) ) return 1.0;
    return 0.0;
}

double MyopicModel::getObservationProbability(size_t s, size_t a, size_t o) const {
    if ( s == a ) return o == 1 ? 0.8 : 0.2;
    return o == 1 ? 0.2 : 0.8;
}

double MyopicModel::getExpectedReward(size_t, size_t, size_t) const {
    return 0.0;
}
