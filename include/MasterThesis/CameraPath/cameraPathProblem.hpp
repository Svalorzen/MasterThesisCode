#ifndef MASTER_THESIS_CAMERA_PATH_MODEL_HEADER_FILE
#define MASTER_THESIS_CAMERA_PATH_MODEL_HEADER_FILE

#include <cstddef>
#include <tuple>
#include <random>
#include <array>

#include <iostream>

// In this model we multiply the statespace of the CameraBasicModel
// by four. In this way we can track the way a target is moving, and
// assume that if it was moving in a particular direction, than it is
// more probable that it will follow it rather than change it. This
// increase in state space does not increase the branching factor of
// the model, so the MCTS family of algorithms should not be hindered
// by it.
class CameraPathModel {
    public:
        enum { UP = 0, RIGHT = 1, DOWN = 2, LEFT = 3 };

        CameraPathModel(unsigned gridSize, double discount);

        size_t getS() const;
        size_t getA() const;
        size_t getO() const;
        double getDiscount() const;
        std::tuple<size_t, size_t, double> sampleSOR(size_t, size_t) const;

        // In this class we use sampleSR in order to produce trajectories
        // which are not actually sampled from the true model distribution.
        // This works because sampleSR is not used anywhere else but to
        // make trajectories. The reason we do this is to maintain a random
        // model for the targets, which makes somewhat sense, but to have
        // targets move non-randomly, which also makes sense.
        std::tuple<size_t, double> sampleSR(size_t, size_t) const;

        bool isTerminal(size_t) const;

        std::tuple<size_t, double> sampleOR(size_t,size_t, size_t) const;

        inline size_t coordToState(unsigned x, unsigned y) const {
            return static_cast<size_t>(x + gridSize_ * y);
        }

        inline std::tuple<unsigned, unsigned> stateToCoord(size_t s) const {
            if ( s == S-1 ) return std::make_tuple( 1000, 1000 );
            return std::make_tuple( s % gridSize_, s / gridSize_ );
        }

        double getTransitionProbability(size_t, size_t, size_t) const;
        double getObservationProbability(size_t, size_t, size_t) const;
        // Unused
        double getExpectedReward(size_t, size_t, size_t) const;

        void visualize(const std::vector<size_t> & positions, size_t camera) const;

        // We use this so that we can let guessing methods guess the target
        // position, not target + direction.
        size_t getTrueState(size_t s) const { return convertToNormalState(s); }
    private:
        size_t sampleTransition(size_t) const;
        // This is the function that creates non-fully-random transitions
        // to make targets move in a believable fashion. The idea is to
        // make each target select a random cell and go there. When he arrives,
        // he selects a new target and so on.
        size_t sampleTrajectoryTransition(size_t) const;
        size_t sampleObservation(size_t, size_t) const;

        // This function tells us which is the preferred direction
        // that a target wants to move.
        int getPreferredDirectionFromState(size_t) const;
        // This function removes the preferred part from the state
        // to keep previous code.
        size_t convertToNormalState(size_t) const;

        size_t getNextDirState(size_t, unsigned) const;
        size_t checkCameraField(size_t, size_t) const;

        unsigned gridSize_, gridCells_;
        size_t entranceA_, entranceB_;

        unsigned cameraSize_;
        std::vector<std::array<unsigned, 4>> cameraData; // Observed count, x1, y1, width

        size_t S, A;
        double discount_;

        mutable std::default_random_engine rand_;
};

#endif
