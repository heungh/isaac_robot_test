# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Interactive Spot locomotion demo with WASD keyboard control.

Uses a trained RSL-RL checkpoint to drive a Spot robot with keyboard velocity commands.

Controls:
    W: Move forward
    S: Move backward
    A: Turn left
    D: Turn right
    Q: Strafe left
    E: Strafe right
    SPACE: Stop
    C: Toggle third-person / perspective camera
    ESC: Exit third-person view

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/demos/spot_locomotion.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
import scripts.reinforcement_learning.rsl_rl.cli_args as cli_args  # isort: skip

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Interactive Spot locomotion demo with WASD keyboard control.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
from rsl_rl.runners import OnPolicyRunner

import carb
import omni
from omni.kit.viewport.utility import get_viewport_from_window_name
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sim.utils.stage import get_current_stage
from isaaclab.utils.math import quat_apply

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

from isaaclab_tasks.manager_based.locomotion.velocity.config.spot.flat_env_cfg import SpotFlatEnvCfg_PLAY
from isaaclab_tasks.utils import get_checkpoint_path

TASK = "Isaac-Velocity-Flat-Spot-v0"
RL_LIBRARY = "rsl_rl"


class SpotLocomotionDemo:
    """Interactive demo for the Spot flat terrain environment with WASD keyboard control.

    Controls:
        W: Move forward
        S: Move backward
        A: Turn left
        D: Turn right
        Q: Strafe left
        E: Strafe right
        SPACE: Stop all motion
        C: Toggle third-person / perspective camera
        ESC: Exit third-person view
    """

    def __init__(self):
        agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(TASK, args_cli)

        # find checkpoint
        if args_cli.checkpoint is not None:
            checkpoint = args_cli.checkpoint
        else:
            log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
            log_root_path = os.path.abspath(log_root_path)
            checkpoint = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
        print(f"[INFO]: Loading model checkpoint from: {checkpoint}")

        # create environment
        env_cfg = SpotFlatEnvCfg_PLAY()
        env_cfg.scene.num_envs = args_cli.num_envs
        env_cfg.episode_length_s = 1000000
        env_cfg.curriculum = None
        # set all commands to zero initially (robot stands still)
        env_cfg.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        env_cfg.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        env_cfg.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        # wrap around environment for rsl-rl
        self.env = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg=env_cfg))
        self.device = self.env.unwrapped.device

        # load previously trained model
        ppo_runner = OnPolicyRunner(self.env, agent_cfg.to_dict(), log_dir=None, device=self.device)
        ppo_runner.load(checkpoint)
        self.policy = ppo_runner.get_inference_policy(device=self.device)

        # command buffer: (num_envs, 3) for [lin_vel_x, lin_vel_y, ang_vel_z]
        self.commands = torch.zeros(env_cfg.scene.num_envs, 3, device=self.device)

        self.create_camera()
        self.set_up_keyboard()
        self._camera_local_transform = torch.tensor([-2.5, 0.0, 0.8], device=self.device)
        self._follow_robot = True

    def create_camera(self):
        """Creates a camera for third-person view."""
        stage = get_current_stage()
        self.viewport = get_viewport_from_window_name("Viewport")
        self.camera_path = "/World/Camera"
        self.perspective_path = "/OmniverseKit_Persp"
        camera_prim = stage.DefinePrim(self.camera_path, "Camera")
        camera_prim.GetAttribute("focalLength").Set(8.5)
        coi_prop = camera_prim.GetProperty("omni:kit:centerOfInterest")
        if not coi_prop or not coi_prop.IsValid():
            camera_prim.CreateAttribute(
                "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
            ).Set(Gf.Vec3d(0, 0, -10))
        # start in third-person view
        self.viewport.set_active_camera(self.camera_path)

    def set_up_keyboard(self):
        """Sets up keyboard input with WASD + Spacebar controls."""
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        FORWARD_SPEED = 1.5
        BACKWARD_SPEED = 0.8
        STRAFE_SPEED = 0.8
        TURN_SPEED = 1.5

        # Each value: [lin_vel_x, lin_vel_y, ang_vel_z]
        self._key_to_control = {
            "W": torch.tensor([FORWARD_SPEED, 0.0, 0.0], device=self.device),
            "S": torch.tensor([-BACKWARD_SPEED, 0.0, 0.0], device=self.device),
            "A": torch.tensor([0.0, 0.0, TURN_SPEED], device=self.device),
            "D": torch.tensor([0.0, 0.0, -TURN_SPEED], device=self.device),
            "Q": torch.tensor([0.0, STRAFE_SPEED, 0.0], device=self.device),
            "E": torch.tensor([0.0, -STRAFE_SPEED, 0.0], device=self.device),
            "SPACE": torch.tensor([0.0, 0.0, 0.0], device=self.device),
        }
        # track which keys are currently held
        self._active_keys: set[str] = set()

        print("\n" + "=" * 60)
        print("  Spot Keyboard Locomotion Demo")
        print("=" * 60)
        print("  W: Forward    S: Backward")
        print("  A: Turn Left  D: Turn Right")
        print("  Q: Strafe Left  E: Strafe Right")
        print("  SPACE: Stop")
        print("  C: Toggle camera view")
        print("=" * 60 + "\n")

    def _on_keyboard_event(self, event):
        """Handles keyboard press/release events."""
        key = event.input.name

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if key == "SPACE":
                # spacebar: stop everything
                self._active_keys.clear()
                self.commands[:] = 0.0
            elif key in self._key_to_control:
                self._active_keys.add(key)
                self._update_commands()
            elif key == "C":
                if self.viewport.get_active_camera() == self.camera_path:
                    self.viewport.set_active_camera(self.perspective_path)
                    self._follow_robot = False
                else:
                    self.viewport.set_active_camera(self.camera_path)
                    self._follow_robot = True
            elif key == "ESCAPE":
                self.viewport.set_active_camera(self.perspective_path)
                self._follow_robot = False

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if key in self._active_keys:
                self._active_keys.discard(key)
                self._update_commands()

        return True

    def _update_commands(self):
        """Recompute command from all currently held keys (allows combining W+A, etc.)."""
        cmd = torch.zeros(3, device=self.device)
        for key in self._active_keys:
            cmd += self._key_to_control[key]
        # clamp to reasonable ranges
        cmd[0] = cmd[0].clamp(-2.0, 3.0)   # lin_vel_x
        cmd[1] = cmd[1].clamp(-1.5, 1.5)   # lin_vel_y
        cmd[2] = cmd[2].clamp(-2.0, 2.0)   # ang_vel_z
        self.commands[:] = cmd

    def update_camera(self):
        """Updates third-person camera to follow robot 0."""
        if not self._follow_robot:
            return
        base_pos = self.env.unwrapped.scene["robot"].data.root_pos_w[0, :]
        base_quat = self.env.unwrapped.scene["robot"].data.root_quat_w[0, :]

        camera_pos = quat_apply(base_quat, self._camera_local_transform) + base_pos

        camera_state = ViewportCameraState(self.camera_path, self.viewport)
        eye = Gf.Vec3d(camera_pos[0].item(), camera_pos[1].item(), camera_pos[2].item())
        target = Gf.Vec3d(base_pos[0].item(), base_pos[1].item(), base_pos[2].item() + 0.4)
        camera_state.set_position_world(eye, True)
        camera_state.set_target_world(target, True)


def main():
    """Main function."""
    demo = SpotLocomotionDemo()
    obs, _ = demo.env.reset()
    # start with zero commands
    obs[:, 9:12] = demo.commands

    while simulation_app.is_running():
        demo.update_camera()
        with torch.inference_mode():
            action = demo.policy(obs)
            obs, _, _, _ = demo.env.step(action)
            # overwrite velocity commands in observation with keyboard input
            obs[:, 9:12] = demo.commands


if __name__ == "__main__":
    main()
    simulation_app.close()
