#!/usr/bin/env python3
import numpy as np


def verify_gym_classic_control_install() -> None:
    """
    Minimal OpenAi gym classic_control install verification.
    :return: None
    """
    
    # (NICE TO HAVE) ToDo: refactor >> with `tools.debug_tools.message.consol_msg()`
    print("\n> Start Gym classic control install check")
    
    try:
        import gym
        
        print(f"> Gym version:    {gym.__version__}")
        env: gym.wrappers.time_limit.TimeLimit = gym.make("CartPole-v1")
        env.reset()
        ## Note: Dont render for unit-test
        action = env.action_space.sample()
        next_obs = env.step(action=action)
        assert isinstance(next_obs[0], np.ndarray)
        env.close()
        print("\n> Gym classic control install is good to go!\n")
    except Exception as e:
        # Note: The exception scope is large on purpose
        
        # (Nice to have) ToDo:validate >> next bloc ↓↓
        raise Exception(
                "> (!) Something is wrong with Gym classic control.\n"
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
        env: gym.wrappers.time_limit.TimeLimit = gym.make('BipedalWalker-v3')
        env.reset()
        ## Note: Dont render for unit-test
        action = env.action_space.sample()
        next_obs = env.step(action=action)
        assert isinstance(next_obs[0], np.ndarray)
        env.close()
        print("\n> Gym Box2D install is good to go!\n")
    except Exception as e:
        # Note: The exception scope is large on purpose
        
        raise Exception(
                f"> (!) Something is wrong with Gym Box2D.\n"
                f"{e}"
                ) from e
    
    return None


if __name__ == "__main__":
    verify_gym_classic_control_install()
    verify_gym_box2d_install()
