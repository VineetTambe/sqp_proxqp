# sqp_proxqp

A [Sequential Quardratic Program](https://en.wikipedia.org/wiki/Sequential_quadratic_programming) Solver using [ProxQP](https://github.com/Simple-Robotics/proxsuite)

The library depends on [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) andn [ProxQP](https://github.com/Simple-Robotics/proxsuite).

### Running Instructions

0. clone the repo

```
git clone git@github.com:VineetTambe/sqp_proxqp.git
```

1. Make a new build directory

```
cd sqp_proxqp
mkdir build
cd build
```

2. run cmake

```
cmake ..
```

3. run make

```
make
```

### Example

The project currently has 1 simple example simulating a brick falling in a 2D world.

The state is the velocity vector with the axis being the y axis and z axis in the world frame.

The initial velocity is `[1,4] m/s`

The initial position is `[0,1] m`

We simulate the motion for 5 seconds.

To run the example build the project and cd into the `build` directory and run the `brick_falling_sqp` executable by running:

```
./brick_falling_sqp
```

There is another example `brick_sqp.cpp` which solves the same problem as above but without using the library and directly calling proxQP.
