"""
Spot Robot RL Controller - 강화학습 정책을 적용한 로봇 제어
- 훈련된 RSL-RL 정책을 로드하여 로봇 제어
- 키보드 입력으로 속도 명령 전달 (WASD)
- 정책이 관절 각도를 계산하여 자연스러운 보행 생성
"""

import argparse
import sys
import os

# IsaacLab 앱 런처 설정
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spot Robot RL Controller with Keyboard")
parser.add_argument("--num_envs", type=int, default=1, help="Number of robots (default: 1)")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to checkpoint")
parser.add_argument("--video", action="store_true", default=False, help="Record video")
parser.add_argument("--video_length", type=int, default=500, help="Video length in steps")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# GUI 모드로 실행 (headless=False가 기본값)
args_cli.headless = False

# 카메라 활성화 (비디오 녹화 시)
if args_cli.video:
    args_cli.enable_cameras = True

# Isaac Sim 앱 실행
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""메인 코드"""

import numpy as np
import torch
import gymnasium as gym
from datetime import datetime

# 키보드 입력
import carb.input
import omni.appwindow

# IsaacLab 임포트
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

import isaaclab_tasks  # 태스크 등록
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# ============================================================
# 설정
# ============================================================
TASK_NAME = "Isaac-Velocity-Flat-Spot-v0"
EXPERIMENT_NAME = "spot_flat"

print("=" * 60)
print("  Spot Robot RL Controller")
print("  - Trained Policy for Natural Walking")
print("=" * 60)

# ============================================================
# 키보드 상태 클래스
# ============================================================
class KeyboardController:
    def __init__(self):
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.strafe_left = False
        self.strafe_right = False

        # 속도 명령 (x: 전진, y: 횡이동, yaw: 회전)
        self.cmd_vx = 0.0  # 전진/후진 속도
        self.cmd_vy = 0.0  # 횡이동 속도
        self.cmd_yaw = 0.0  # 회전 속도

        # 속도 설정
        self.max_vx = 1.0   # 최대 전진 속도 (m/s)
        self.max_vy = 0.5   # 최대 횡이동 속도 (m/s)
        self.max_yaw = 1.0  # 최대 회전 속도 (rad/s)

    def update_commands(self):
        """키보드 상태에 따라 속도 명령 업데이트"""
        # 전진/후진
        if self.forward:
            self.cmd_vx = self.max_vx
        elif self.backward:
            self.cmd_vx = -self.max_vx
        else:
            self.cmd_vx = 0.0

        # 횡이동 (Q/E)
        if self.strafe_left:
            self.cmd_vy = self.max_vy
        elif self.strafe_right:
            self.cmd_vy = -self.max_vy
        else:
            self.cmd_vy = 0.0

        # 회전 (A/D)
        if self.left:
            self.cmd_yaw = self.max_yaw
        elif self.right:
            self.cmd_yaw = -self.max_yaw
        else:
            self.cmd_yaw = 0.0

    def get_command_tensor(self, device, num_envs=1):
        """속도 명령을 텐서로 반환"""
        cmd = torch.tensor([[self.cmd_vx, self.cmd_vy, self.cmd_yaw]],
                          dtype=torch.float32, device=device)
        return cmd.repeat(num_envs, 1)

keyboard = KeyboardController()

def on_keyboard_event(event, *args, **kwargs):
    """키보드 이벤트 처리"""
    global keyboard

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.W:
            keyboard.forward = True
            print("🔵 Forward")
        elif event.input == carb.input.KeyboardInput.S:
            keyboard.backward = True
            print("🔵 Backward")
        elif event.input == carb.input.KeyboardInput.A:
            keyboard.left = True
            print("🔵 Turn Left")
        elif event.input == carb.input.KeyboardInput.D:
            keyboard.right = True
            print("🔵 Turn Right")
        elif event.input == carb.input.KeyboardInput.Q:
            keyboard.strafe_left = True
            print("🔵 Strafe Left")
        elif event.input == carb.input.KeyboardInput.E:
            keyboard.strafe_right = True
            print("🔵 Strafe Right")

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if event.input == carb.input.KeyboardInput.W:
            keyboard.forward = False
        elif event.input == carb.input.KeyboardInput.S:
            keyboard.backward = False
        elif event.input == carb.input.KeyboardInput.A:
            keyboard.left = False
        elif event.input == carb.input.KeyboardInput.D:
            keyboard.right = False
        elif event.input == carb.input.KeyboardInput.Q:
            keyboard.strafe_left = False
        elif event.input == carb.input.KeyboardInput.E:
            keyboard.strafe_right = False

    keyboard.update_commands()
    return True

# ============================================================
# 메인 함수
# ============================================================
@hydra_task_config(TASK_NAME, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg):
    """RL 정책을 사용한 Spot 로봇 제어"""

    # 환경 설정 - 로봇 1마리만
    env_cfg.scene.num_envs = 1  # 강제로 1로 설정
    env_cfg.sim.device = "cuda:0"

    # 뷰어 카메라 설정 (로봇이 보이도록)
    env_cfg.viewer.eye = (3.0, 3.0, 2.0)  # 카메라 위치
    env_cfg.viewer.lookat = (0.0, 0.0, 0.0)  # 바라보는 지점
    env_cfg.viewer.origin_type = "env"  # 환경 원점 기준

    # 로그 디렉토리에서 체크포인트 찾기
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)

    if args_cli.checkpoint:
        resume_path = args_cli.checkpoint
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    print(f"\n[INFO] Loading checkpoint: {resume_path}")

    # 환경 생성
    print("[INFO] Creating environment...")
    env = gym.make(TASK_NAME, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # 비디오 녹화 설정
    if args_cli.video:
        log_dir = os.path.dirname(resume_path)
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "keyboard_control"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print(f"[INFO] Recording video to: {video_kwargs['video_folder']}")
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # RSL-RL 래퍼
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # 정책 로드
    print("[INFO] Loading trained policy...")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)

    # 추론용 정책 획득
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # 정책 신경망
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    print("\n" + "=" * 60)
    print("  🐕 Spot Robot Ready! (RL Policy Control)")
    print("=" * 60)
    print("\nKeyboard Controls:")
    print("  W/S: Forward/Backward")
    print("  A/D: Turn Left/Right")
    print("  Q/E: Strafe Left/Right")
    print("\nThe robot uses trained RL policy for walking!")
    print("=" * 60 + "\n")

    # 키보드 입력 설정
    appwindow = omni.appwindow.get_default_app_window()
    input_interface = carb.input.acquire_input_interface()
    keyboard_device = appwindow.get_keyboard()
    sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard_device, on_keyboard_event)

    # 초기 관측값
    obs = env.get_observations()

    # 메인 루프
    step_count = 0

    try:
        while simulation_app.is_running():
            # 키보드 명령 업데이트
            keyboard.update_commands()

            # 속도 명령을 환경에 주입
            # IsaacLab 환경에서 명령 관리자를 통해 속도 명령 설정
            unwrapped_env = env.unwrapped
            if hasattr(unwrapped_env, 'command_manager'):
                cmd_tensor = keyboard.get_command_tensor(
                    device=unwrapped_env.device,
                    num_envs=unwrapped_env.num_envs
                )
                # 명령 버퍼에 직접 쓰기 (velocity command)
                if hasattr(unwrapped_env.command_manager, '_terms'):
                    for term_name, term in unwrapped_env.command_manager._terms.items():
                        if 'velocity' in term_name.lower():
                            if hasattr(term, 'vel_command_b'):
                                term.vel_command_b[:, :3] = cmd_tensor

            # 정책으로 액션 계산
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = env.step(actions)
                policy_nn.reset(dones)

            step_count += 1

            # 상태 출력 (100 스텝마다)
            if step_count % 100 == 0:
                status = "🚶 Walking" if (abs(keyboard.cmd_vx) > 0.01 or
                                          abs(keyboard.cmd_vy) > 0.01 or
                                          abs(keyboard.cmd_yaw) > 0.01) else "🛑 Standing"
                print(f"Step {step_count:5d} | Cmd: vx={keyboard.cmd_vx:+.2f} vy={keyboard.cmd_vy:+.2f} "
                      f"yaw={keyboard.cmd_yaw:+.2f} | {status}")

            # 비디오 녹화 완료 확인
            if args_cli.video and step_count >= args_cli.video_length:
                print(f"\n[INFO] Video recording complete ({step_count} steps)")
                break

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    # 정리
    print("\nCleaning up...")
    input_interface.unsubscribe_to_keyboard_events(keyboard_device, sub_keyboard)
    env.close()
    print("Done!")

if __name__ == "__main__":
    main()
    simulation_app.close()
