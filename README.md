# Planning Project (RO47005)

## Table of contents
- [Planning Project (RO47005)](#planning-project-ro47005)
  - [Table of contents](#table-of-contents)
  - [Objective](#objective)
  - [Labour distribution](#labour-distribution)
  - [Directories](#directories)
  - [Some achievements](#some-achievements)
    - [Hybrid A\*](#hybrid-a)
    - [Nonlinear mpc](#nonlinear-mpc)
      - [The encountering situation](#the-encountering-situation)
      - [The following up situation](#the-following-up-situation)
  - [To Run Our Code](#to-run-our-code)

## Objective

Solve parking problem in an unstructured (i.e. no specific traffic rules for the vehicle) parking lot based on this [simulation environment](https://github.com/maxspahn/gym_envs_urdf).

## Labour distribution

The added functionalities by us are:

- Simulation environment and graph building
- Hybrid A* path planner
- Nonlinear MPC formulation

For more details, please refer to our [report](https://github.com/runyuma/Planning_project/blob/main/ro47005_pdm_proj.pdf).

## Directories
- `hybrid_astar`              : where hybrid A* path planner is implemented
- `motion_planning_scenes`    : adapted (unmodified) from [motion planning scenes repository](https://github.com/maxspahn/motion_planning_scenes.git)
- `obstacled_environments`   : where the parking lot (obstacled) environments are defined in `gym`
  <!-- - `/complex_parking_lot`    : defines the complex (harder) parking place finding environment
  - `/simple_parking_lot`     : defines the simple (easier) parking place finding environment
  - `/common`                 : shared files for the two environments -->
- `mpc_controller`            : where linear mpc controller is implemented
- `nonlinear_mpc`             : where nonlinear mpc controller is implemented based on the [acados](https://github.com/acados/acados)
- `scenarios`                 : where the final files stored to get all together
    - `dummy_examples`    : where we use some simple example to test the previous functions
  - `parking_tasks`     : where the final parking tests filed stored


## Some achievements

With this project, we manage to solve the parking problem in an unstructured parking lot using the Hybrid A* algorithm as the path planner and the nonlinear model predictive control to further optimize the solution.

### Hybrid A*
With Hybrid A* path planner, we manage to get a path from the initial place to the desired destination on our parking lot.

![](https://github.com/runyuma/Planning_project/blob/main/fig_gif/playground1.png)

### Nonlinear mpc
We use four different situations to test our code, separately without unexpected obstacle, static obstacle, encountering obstacle and following up obstacle. The plots for path and velocity of our self-driving car is shown below.

![](https://github.com/runyuma/Planning_project/blob/main/fig_gif/all%20in%20one.png)

#### The encountering situation

![](https://github.com/runyuma/Planning_project/blob/main/fig_gif/encountering.gif)

#### The following up situation

![](https://github.com/runyuma/Planning_project/blob/main/fig_gif/following.gif)

## To Run Our Code

To run our code, please refer to the guidance [here](https://github.com/runyuma/Planning_project/blob/main/to_run_our_code.md).

