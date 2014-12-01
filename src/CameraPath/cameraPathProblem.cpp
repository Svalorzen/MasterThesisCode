#include <MasterThesis/CameraPath/cameraPathProblem.hpp>

#include <cassert>
#include <AIToolbox/Impl/Seeder.hpp>

constexpr int cameraField = 10;
constexpr int preferredPathProbabilityD20 = 14;
constexpr double preferredPathProbability = 0.7;
constexpr double nonPreferredPathProbability = 0.1;

CameraPathModel::CameraPathModel(unsigned gridSize, double d) : gridSize_(gridSize), gridCells_(gridSize_*gridSize_), S(gridCells_*4+1),
                                                                  discount_(d), rand_(AIToolbox::Impl::Seeder::getSeed())
{
    if ( gridSize < 1 ) throw std::invalid_argument("This grid size is not allowed: " + std::to_string(gridSize));
    bool even = !(gridSize_ % 2);
    entranceA_ = ((gridSize_+1) / 2)        * gridSize_ - 1;
    entranceB_ = ((gridSize_+1) / 2 + even) * gridSize_ - 1;

    cameraSize_ = ((gridSize_-1) / cameraField + 1);
    A = cameraSize_ * cameraSize_;
    cameraData.resize(A);

    // Initialize camera data
    size_t a = 0;
    for ( unsigned ay = 0; ay < cameraSize_; ++ay ) {
        for ( unsigned ax = 0; ax < cameraSize_; ++ax ) {
            unsigned x1, y1, x2, y2;
            x1 = ax * cameraField;
            y1 = ay * cameraField;
            x2 = std::min( (ax+1) * cameraField - 1, gridSize_ - 1);
            y2 = std::min( (ay+1) * cameraField - 1, gridSize_ - 1);

            cameraData[a] = {{
                (x2 - x1 + 1) * (y2 - y1 + 1), // Count of cells inside.
                x1, y1, x2 - x1 + 1
            }};

            ++a;
        }
    }
}

// INFO FUNCTIONS
size_t CameraPathModel::getS() const { return S; }

size_t CameraPathModel::getA() const {
#ifdef HALF_VISIBILITY
    return (A+1)/2;
#else
    return A;
#endif
}

size_t CameraPathModel::getO() const { return cameraField * cameraField + 1; }
double CameraPathModel::getDiscount() const { return discount_; }

// SAMPLING FUNCTIONS

std::tuple<size_t, size_t, double> CameraPathModel::sampleSOR(size_t s, size_t a) const {
#ifdef HALF_VISIBILITY
    a *= 2;
    if ( !(A % 2) && ( a / cameraSize_ ) % 2 ) ++a;
#endif

    size_t s1 = sampleTransition(s);
    size_t o = sampleObservation(s1, a);

    return std::make_tuple(s1, o, 0.0);
}

std::tuple<size_t, double> CameraPathModel::sampleOR(size_t, size_t a, size_t s1) const {
#ifdef HALF_VISIBILITY
    a *= 2;
    if ( !(A % 2) && ( a / cameraSize_ ) % 2 ) ++a;
#endif

    return std::make_tuple(sampleObservation(s1, a), 0.0);
}

std::tuple<size_t, double> CameraPathModel::sampleSR(size_t s, size_t) const {
    return std::make_tuple(sampleTrajectoryTransition(s), 0.0);
}

// IMPLEMENTATIONS

// Modified from Basic..
size_t CameraPathModel::sampleTransition(size_t s) const {
    static std::uniform_int_distribution<unsigned> dist1(1, 20);
    static std::uniform_int_distribution<unsigned> dist2(1, 3);

    // From outside
    if ( s == S-1 ) {
        // 0.05 chance for both
        auto dice = dist1(rand_);
        // Start off walking left
        if ( dice == 19 ) return entranceA_ + gridCells_ * LEFT;
        if ( dice == 20 ) return entranceB_ + gridCells_ * LEFT;
        return s;
    }

    auto preferredDirection = getPreferredDirectionFromState(s);
    auto normalState = convertToNormalState(s);
    auto dice = dist1(rand_);

    auto newDirection = preferredDirection;

    // 0.85 % of choosing preferred direction.
    if ( dice > preferredPathProbabilityD20 ) {
        newDirection = dist2(rand_);
        // This we do since dist2 does not produce 0.
        if ( newDirection == preferredDirection ) newDirection = 0;
    }

    auto newState = getNextDirState(normalState, newDirection);

    if ( newState == S-1 ) return newState;
    return newState + gridCells_ * newDirection;
}

size_t CameraPathModel::sampleTrajectoryTransition(size_t s) const {
    static std::uniform_int_distribution<unsigned> dist1(0, 9);
    static std::uniform_int_distribution<unsigned> distg(0, gridSize_-1);
    static size_t xg = 0, yg = 0;

    if ( s == coordToState(xg, yg) || dist1(rand_) == 0 ) {
        xg = distg(rand_); yg = distg(rand_);
    }

    size_t x, y;
    std::tie(x, y) = stateToCoord(s);

    int dir;
    if ( x < xg ) dir = RIGHT;
    else if ( y < yg ) dir = DOWN;
    else if ( x > xg ) dir = LEFT;
    else dir = UP;

    return getNextDirState(s, dir);
}

double computePrecision(int puc, int data) {
    // 0.5-1
    return 1.0 - ( std::abs(puc - data/2 - 1) / (double) data );
}

size_t CameraPathModel::sampleObservation(size_t s1, size_t a) const {
    static std::uniform_int_distribution<unsigned> dist1(0, 4);
    static std::uniform_real_distribution<double>  prob(0, 1);

    // Modified this line from Basic
    s1 = convertToNormalState(s1);
    size_t positionUnderCamera = checkCameraField(a, s1);

    // If the person is under the camera, we see it more correctly
    // towards the center, and worse towards the edges (not exactly
    // like this, since we compute imprecision from center as if the
    // camera sees a line rather than an area, but close enough.
    if ( positionUnderCamera != 0 ) {
        double precision = computePrecision(positionUnderCamera, cameraData[a][0]);

        // Camera worked correctly
        if ( precision > prob(rand_) ) return positionUnderCamera;
        // We return a state close to the one we saw (kind of noise..) or nothing
        int cameraCheck = dist1(rand_);
        if ( cameraCheck == 4 ) return positionUnderCamera;
        return checkCameraField( a, getNextDirState(s1, cameraCheck) );
    }
    // Otherwise we don't see the target.
    else {
        return 0;
    }
}

// PROBABILITIES

double CameraPathModel::getObservationProbability(size_t s1, size_t a, size_t o) const {
    // Changed this line from basic
    s1 = convertToNormalState(s1);
    size_t positionUnderCamera = checkCameraField(a, s1);

    // If the target is not under the camera, we cannot see it
    if ( !positionUnderCamera )
        return o ? 0.0 : 1.0;
    // Precision with used camera
    double precision = computePrecision(positionUnderCamera, cameraData[a][0]);
    double error = (1.0 - precision)/5.0;

    if ( o == positionUnderCamera )
        return precision + error;

    double retvalue = 0.0;
    // We accumulate due to non-movements around the edges.
    for ( unsigned i = 0; i < 4; ++i )
        if ( o == checkCameraField(a, getNextDirState(s1, i)) ) retvalue += error;

    return retvalue;
}

// Changed from Basic
double CameraPathModel::getTransitionProbability(size_t s, size_t, size_t s1) const {
    if ( s == S-1 ) {
        if ( s1 == s ) return 0.9;
        if ( s1 == entranceA_ + gridCells_ * LEFT || s1 == entranceB_ + gridCells_ * LEFT ) return 0.05;
        return 0.0;
    }
    auto preferredDirection = getPreferredDirectionFromState(s);
    auto normalState = convertToNormalState(s);

    if ( s1 == getNextDirState(normalState, preferredDirection) + gridCells_ * preferredDirection)
        return preferredPathProbability;

    for ( int i = 0; i < 4; ++i ) {
        if ( i == preferredDirection ) continue;
        if ( s1 == getNextDirState(normalState, i) + gridCells_ * i ) return nonPreferredPathProbability;
    }
    return 0.0;
}

// MOVEMENT AND CAMERA CODE

size_t CameraPathModel::getNextDirState(size_t s, unsigned dir) const {
    size_t s1;
    switch ( dir ) {
        case 0:
            s1 = s >= gridSize_ ? s - gridSize_ : s;
            break;
        case 1:
            s1 = ( !((s+1)%gridSize_) ? s : s + 1 );
            // Special case for going out outside
            if ( s1 == s && ( s1 == entranceA_ || s1 == entranceB_ ) ) s1 = S - 1;
            break;
        case 2:
            s1 = ( (s >= gridCells_ - gridSize_ ) ? s : s + gridSize_ );
            break;
        case 3:
            s1 = ( !(s%gridSize_) ? s : s - 1 );
            break;
        default: assert(false); s1 = 0;
    }
    return s1;
}

size_t CameraPathModel::checkCameraField(size_t a, size_t s) const {
    assert(a < A);
    if ( s == S - 1 ) return 0;

    int x, y;
    std::tie(x,y) = stateToCoord(s);

    int positionUnderCameraX, positionUnderCameraY;
    positionUnderCameraX = x - static_cast<int>(cameraData[a][1]);
    positionUnderCameraY = y - static_cast<int>(cameraData[a][2]);

    size_t positionUnderCamera;
    if ( positionUnderCameraX < 0 || positionUnderCameraY < 0 || positionUnderCameraX >= cameraField || positionUnderCameraY >= cameraField )
        positionUnderCamera = 0;
    else
        positionUnderCamera = positionUnderCameraX + positionUnderCameraY * cameraData[a][3] + 1;

    return positionUnderCamera;
}

bool CameraPathModel::isTerminal(size_t) const { return false; }

void CameraPathModel::visualize(const std::vector<size_t> & positions, size_t camera) const {
#ifdef HALF_VISIBILITY
    camera *= 2;
    if ( !(A % 2) && ( camera / cameraSize_ ) % 2 ) ++camera;
#endif

    for ( unsigned y = 0; y < gridSize_; ++y ) {
        for ( unsigned x = 0; x < gridSize_; ++x ) {
            size_t s = coordToState(x, y);
            unsigned counter = 0;
            for( auto x : positions ) if ( convertToNormalState(x) == s ) ++counter;
            if ( counter ) std::cout << counter << " ";
            else {
                if ( checkCameraField(camera, s) != 0 ) std::cout << "* ";
                else                                    std::cout << ". ";
            }
        }
        std::cout << "\n";
    }
}

int CameraPathModel::getPreferredDirectionFromState(size_t s) const {
    return s == S-1 ? 0 : s / gridCells_;
}

size_t CameraPathModel::convertToNormalState(size_t s) const {
    return s == S-1 ? s : s % gridCells_;
}
