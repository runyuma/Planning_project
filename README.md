# Planning_project

## Objective

Solve parking problem in an unstructured (i.e. no specific traffic rules for the vehicle) parking lot.

## Labour distribution

- MPC formulation
- Hybrid A* adaptation
- Simulation environment and graph building

## Directories
- `motion_planning_scenes`    : adapted (unmodified) from *https://github.com/maxspahn/motion_planning_scenes.git*
- `/obstacled_environments`   : where the parking lot (obstacled) environments are defined in `gym`
  - `/complex_parking_lot`    : defines the complex (harder) parking place finding environment
  - `/simple_parking_lot`     : defines the simple (easier) parking place finding environment
  - `/common`                 : shared files for the two environments
- 'scenarios'                 : each scenario of parking is shown

## TODO:
- modify `SimpleUrdfEnv` and `ComplexUrdfEnv` in `urdf_simple_env.py` and `urdf_complex_env.py`

