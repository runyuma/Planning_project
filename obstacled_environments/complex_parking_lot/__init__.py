from gym.envs.registration import register
from obstacled_environments.complex_parking_lot.urdf_complex_env import ComplexUrdfEnv
register(
    id='complex-parking-lot-env-v0',
    entry_point='obstacled_environments.complex_parking_lot:ComplexUrdfEnv'
)
