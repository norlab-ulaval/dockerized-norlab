#!/usr/bin/env python3
import numpy as np


def verify_gym_classic_control_install() -> None:
    """
    Minimal OpenAi gym classic_control install verification.
    :return: None
    """

    print("\n> Start Gym classic control install check")

    try:
        import gym

        print(f'> Gym version:    {gym.__version__}')
        # env: gym.wrappers.time_limit.TimeLimit = gym.make('Pendulum-v1')
        env: gym.wrappers.time_limit.TimeLimit = gym.make('CartPole-v1')
        env.reset()
        # output = env.render("rgb_array")
        action = env.action_space.sample()
        next_obs = env.step(action=action)
        assert isinstance(next_obs[0], np.ndarray)
        env.close()
        print("\n> Gym classic control install is good to go!\n")
    except Exception as e:
        # Note: The exception scope is large on purpose

        # (PRIORITY) ToDo:validate >> next bloc ↓↓
        raise Exception(
                f"> (!) Something is wrong with Gym classic control.\n"
                f"{e}"
                ) from e

    return None


def verify_gym_box2d_install() -> None:
    """
    Minimal OpenAi gym classic_control install verification.
    :return: None
    """

    print("\n> Start Gym Box2D install check")

    try:
        import gym

        print(f'> Gym version:    {gym.__version__}')
        env: gym.wrappers.time_limit.TimeLimit = gym.make('CarRacing-v0')
        env.reset()
        # output = env.render("rgb_array")
        action = env.action_space.sample()
        next_obs = env.step(action=action)
        assert isinstance(next_obs[0], np.ndarray)
        env.close()
        print("\n> Gym Box2D install is good to go!\n")
    except gym.error.DeprecatedEnv as e:
        env: gym.wrappers.time_limit.TimeLimit = gym.make('CarRacing-v2')
        env.reset()
        # output = env.render("rgb_array")
        action = env.action_space.sample()
        next_obs = env.step(action=action)
        assert isinstance(next_obs[0], np.ndarray)
        env.close()
        print("\n> Gym Box2D install is good to go!\n")
    except Exception as e:
        # Note: The exception scope is large on purpose

        # (Priority) ToDo:validate >> next bloc ↓↓
        raise Exception(
                f"> (!) Something is wrong with Gym Box2D.\n"
                f"{e}"
                ) from e

    return None


if __name__ == "__main__":
    verify_gym_classic_control_install()
    verify_gym_box2d_install()
