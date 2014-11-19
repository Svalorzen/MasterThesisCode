#include <MasterThesis/CameraBasic/cameraBasicProblem.hpp>

#include <cassert>
#include <AIToolbox/Impl/Seeder.hpp>

constexpr int cameraField = 10;

CameraBasicModel::CameraBasicModel(unsigned gridSize, double d) : gridSize_(gridSize), gridCells_(gridSize_*gridSize_), S(gridCells_+1),
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
size_t CameraBasicModel::getS() const { return S; }

size_t CameraBasicModel::getA() const {
    return A;
//    return (A+1)/2;
}

size_t CameraBasicModel::getO() const { return cameraField * cameraField + 1; }
double CameraBasicModel::getDiscount() const { return discount_; }

// SAMPLING FUNCTIONS

std::tuple<size_t, size_t, double> CameraBasicModel::sampleSOR(size_t s, size_t a) const {
    //a *= 2;
    //if ( !(A % 2) && ( a / cameraSize_ ) % 2 ) ++a;

    size_t s1 = sampleTransition(s);
    size_t o = sampleObservation(s1, a);

    return std::make_tuple(s1, o, 0.0);
}

std::tuple<size_t, double> CameraBasicModel::sampleOR(size_t, size_t a, size_t s1) const {
//    a *= 2;
//    if ( !(A % 2) && ( a / cameraSize_ ) % 2 ) ++a;

    return std::make_tuple(sampleObservation(s1, a), 0.0);
}

std::tuple<size_t, double> CameraBasicModel::sampleSR(size_t s, size_t) const {
    return std::make_tuple(sampleTrajectoryTransition(s), 0.0);
}

// IMPLEMENTATIONS

size_t CameraBasicModel::sampleTransition(size_t s) const {
    static std::uniform_int_distribution<unsigned> dist1(1, 20);
    static std::uniform_int_distribution<unsigned> dist2(0, 3);

    // From outside
    if ( s == S-1 ) {
        // 0.05 chance for both
        auto dice = dist1(rand_);
        if ( dice == 19 ) return entranceA_;
        if ( dice == 20 ) return entranceB_;
        return s;
    }

    return getNextDirState(s, dist2(rand_));
}

size_t CameraBasicModel::sampleTrajectoryTransition(size_t s) const {
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

size_t CameraBasicModel::sampleObservation(size_t s1, size_t a) const {
    static std::uniform_int_distribution<unsigned> dist1(0, 4);
    static std::uniform_real_distribution<double>  prob(0, 1);

    size_t positionUnderCamera = checkCameraField(a, s1);

    // If the person is under the camera, we see it more correctly
    // towards the center, and worse towards the edges (not exactly
    // like this, since we compute imprecision from center as if the
    // camera sees a line rather than an area, but close enough.
    if ( positionUnderCamera != 0 ) {
        // 0.5-1
        double precision = 1.0 - (std::abs(positionUnderCamera - cameraData[a][0]/2 - 1) / (double) cameraData[a][0]);

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

double CameraBasicModel::getObservationProbability(size_t s1, size_t a, size_t o) const {
    size_t positionUnderCamera = checkCameraField(a, s1);

    // If the target is not under the camera, we cannot see it
    if ( !positionUnderCamera )
        return o ? 0.0 : 1.0;
    // Precision with used camera
    double precision = 1.0 - (std::abs(positionUnderCamera - cameraData[a][0]/2 - 1) / (double) cameraData[a][0]);
    double error = (1.0 - precision)/5.0;

    if ( o == positionUnderCamera )
        return precision + error;

    double retvalue = 0.0;
    // We accumulate due to non-movements around the edges.
    for ( unsigned i = 0; i < 4; ++i )
        if ( o == checkCameraField(a, getNextDirState(s1, i)) ) retvalue += error;

    return retvalue;
}

double CameraBasicModel::getTransitionProbability(size_t s, size_t, size_t s1) const {
    if ( s == S-1 ) {
        if ( s1 == s ) return 0.9;
        if ( s1 == entranceA_ || s1 == entranceB_ ) return 0.05;
    }
    double retvalue = 0.0;
    for ( unsigned i = 0; i < 4; ++i ) {
        if ( s1 == getNextDirState(s, i) ) retvalue += 0.25;
    }
    return retvalue;
}

// MOVEMENT AND CAMERA CODE

size_t CameraBasicModel::getNextDirState(size_t s, unsigned dir) const {
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

size_t CameraBasicModel::checkCameraField(size_t a, size_t s) const {
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

bool CameraBasicModel::isTerminal(size_t) const { return false; }

void CameraBasicModel::visualize(const std::vector<size_t> & positions, size_t camera) const {
//    camera *= 2;
//    if ( !(A % 2) && ( camera / cameraSize_ ) % 2 ) ++camera;

    for ( unsigned y = 0; y < gridSize_; ++y ) {
        for ( unsigned x = 0; x < gridSize_; ++x ) {
            size_t s = coordToState(x, y);
            unsigned counter = 0;
            for( auto x : positions ) if ( x == s ) ++counter;
            if ( counter ) std::cout << counter << " ";
            else {
                if ( checkCameraField(camera, s) != 0 ) std::cout << "* ";
                else                                    std::cout << ". ";
            }
        }
        std::cout << "\n";
    }
}
