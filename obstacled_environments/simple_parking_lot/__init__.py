from gym.envs.registration import register
from obstacled_environments.simple_parking_lot.urdf_simple_env import SimpleUrdfEnv
register(
    id='simple-parking-lot-env-v0',
    entry_point='obstacled_environments.common:SimpleUrdfEnv'
)
