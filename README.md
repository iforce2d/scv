# S-curve trajectory calculator

This code calculates a jerk limited path in 3D with blended corners. The path is initially formed by joining a sequence of points with straight line moves, each a 7-segment "s-curve". Limits on velocity, acceleration and jerk can be specified for each axis as global constraints, and each move can be given a separate limit on velocity, acceleration and jerk.

Adjacent moves can be blended to turn the corner smoothly, where the limits allow. Whether a corner can be blended depends on the velocities of the previous and next moves, the acceleration and jerk allowed, the length of the moves, and the angle between them.

The output of the algorithm is a sequence of constant-jerk segments, each defined by a starting position/velocity/acceleration, with a jerk value and duration.

Partially based on this very useful explanation:
[Constant jerk equations for a trajectory generator](http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf)

This repo includes a visualizer app to experiment with settings and view an animated movement along the path.

![alt text](https://www.iforce2d.net/tmp/scv/S-Curve_visualizer_004.png)

## Building the visualizer app

To run the visualizer app, after cloning this repo you'll need to pull the required imgui and implot submodules:

    git submodule update --init
    cd src/visualizer
    make
    ./visualizer

## Usage

To set up a plan and calculate the path:

        using namespace scv;
    planner plan;

    plan.setPositionLimits(0, 0, 0, 10, 10, 7);
    plan.setVelocityLimits(100, 100, 100);
    plan.setAccelerationLimits(200, 200, 200);
    plan.setJerkLimits(500, 500, 500);

    scv::move m;
    m.vel = 12;
    m.acc = 200;
    m.jerk = 400;
    m.blendType = CBT_MAX_JERK;
    m.src = vec3(1, 1, 0);
    m.dst = vec3(1, 1, 6);
    plan.appendMove(m);

    m.vel = 4;
    m.dst = vec3(1, 9, 6);
    plan.appendMove(m);

    m.vel = 8;
    m.dst = vec3(1, 9, 0);
    plan.appendMove(m);

        plan.calculateMoves();

        double totalTime = plan.getTotalTime();

The `scv::move` has an explicit namespace qualifier to avoid ambiguity with std::move.

`setVelocityLimits / setAccelerationLimits / setJerkLimits` each take xyz values to set the global constraints in each axis. `setPositionLimits` is currently only used in the visualizer to show the bounding box, and for stress testing the randomized paths.

Each individual move should be given non-zero limits, these will be applied in addition to the global limits, but can be different from move to move. The per-move limits are a magnitude along the direction of travel, not along an axis. So for example if the global velocity limits for x and y are both 10, a diagonal movement at 45 degrees would result in a maximum velocity of 14.142 in the direction of travel. But with a per-move velocity limit of 10, the maximum velocity in the direction of travel will be 10.

blendType can be one of `CBT_MAX_JERK`, `CBT_MIN_JERK`, `CBT_NONE`. Maximum jerk will attempt the tightest turn allowable, coming as close to the original corner as possible. Minimum jerk will take as wider arc to turn the corner as gradually as possible.

To access the calculated path, you can use getTrajectoryState to find all state variables at any given time point:

    double time = 1.23;
    int segmentIndex;
    vec3 p, v, a, j; // position, velocity, acceleration, jerk
    plan.getTrajectoryState(t, &segmentIndex, &p, &v, &a, &j);

`getTrajectoryState` returns true if the given time is within the path. `segmentIndex` will be set to the index of the individual constant-jerk segment at the given time t. If t is negative the pos/vel/acc/jerk values will be zero vectors. If t exceeds the total time of the path, the values will be set to the final position on the path.

`getTrajectoryState` could be considered a 'random access' method and is mostly useful for the visualizer, to check that the results are as intended. It works by looping through the individual segments every time which is not optimal. In an actual usage scenario typically only the position value is needed, it will only be accessed sequentially, and it should be returned as fast as possible. You can use `resetTraverse` and `advanceTraverse` to get just the position value in increasing increments of time:

        bool stillGoing = plan.advanceTraverse( 0.01, &p );

This uses some internal variables in the planner class internally for convenience to keep track of the total time, and will be slightly faster. `resetTraverse` will revert the time back to the start.

        vec3 p;
        plan.resetTraverse();
        while ( plan.advanceTraverse( 0.01, &p ) ) {
                // do something with p
        }
        // do something with p

Note that `advanceTraverse` will return false when the total time is reached, which will exit the while loop. You'll need to 'do something with p' one more time after the loop to ensure the final point is actually processed. The same applies for `getTrajectoryPoint`.

## Floating point type

The `scv_float` preprocessor define is used as the type for floating point values. It is set to `double`, so if you want `float` you can change it in vec3.h

## Trajectory generation notes

This only handles moves from a stationary start point, to a stationary end point. It cannot calculate moves from or to a non-zero-velocity state.

Corner blend segments can only extend out as far as the middle of the move on each side. This makes the calculation easier because the mid-point of each move cannot be altered by a previous blend. This gives the convenient feature that the mid-point of each move will still be at the target velocity even after blending (assuming sufficient acceleration and jerk). It also allows concurrent (multi-threaded) calculation, although that's hardly likely to be necessary.

Corner blends will smoothly transition from the velocity of the previous move, to the velocity of the next move. If the acceleration or jerk are different between blended moves, the minimum of each will be used for the blend.
