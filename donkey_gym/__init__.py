from gym.envs.registration import register


register(
    id='donkey-generated-roads-v0',
    entry_point='donkey_gym.envs:GeneratedRoadsEnv',
)

register(
    id='donkey-warehouse-v0',
    entry_point='donkey_gym.envs:WarehouseEnv',
)

register(
    id='donkey-avc-sparkfun-v0',
    entry_point='donkey_gym.envs:AvcSparkfunEnv',
)

register(
    id='donkey-generated-track-v0',
    entry_point='donkey_gym.envs:GeneratedTrackEnv',
)
