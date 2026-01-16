"""
Spot Robot RL Standalone Controller
- 원본 Isaac Sim 렌더링 사용 (로봇이 보임)
- 훈련된 RSL-RL 정책 적용
- 키보드로 속도 명령 제어
"""

import numpy as np
import torch
import os
from datetime import datetime

# Isaac Sim 시작
from isaacsim import SimulationApp
print("Starting Isaac Sim...")
simulation_app = SimulationApp({"headless": False})

# Isaac Sim 임포트
import carb.input
import omni.appwindow
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from pxr import UsdPhysics

# ============================================================
# 설정
# ============================================================
CHECKPOINT_PATH = os.path.expanduser(
    "~/isaac-sim/IsaacLab/logs/rsl_rl/spot_flat/2026-01-15_02-09-06/model_2350.pt"
)

# v20과 동일한 설정
DECIMATION = 10           # 정책 실행 주기 (10 physics step마다)
PHYSICS_DT = 0.002        # 500Hz physics
ACTION_SCALE = 0.2        # 액션 스케일
KP = 60.0                 # PD 제어 stiffness
KD = 1.5                  # PD 제어 damping

JOINT_NAMES = ["fl_hx", "fr_hx", "hl_hx", "hr_hx", "fl_hy", "fr_hy", "hl_hy", "hr_hy", "fl_kn", "fr_kn", "hl_kn", "hr_kn"]

# v20과 동일한 기본 자세 (IsaacLab 훈련 시 사용)
ISAACLAB_DEFAULT_POS = np.array([
    0.1, -0.1, 0.1, -0.1,    # hip_x (fl, fr, hl, hr)
    0.9, 0.9, 1.1, 1.1,      # hip_y (fl, fr, hl, hr)
    -1.5, -1.5, -1.5, -1.5   # knee (fl, fr, hl, hr)
], dtype=np.float32)

print("=" * 60)
print("  Spot Robot RL Standalone Controller")
print("  - Trained Policy + Original Rendering")
print("=" * 60)

# ============================================================
# 정책 로드
# ============================================================
print(f"\nLoading policy from: {CHECKPOINT_PATH}")

# RSL-RL 체크포인트 로드
checkpoint = torch.load(CHECKPOINT_PATH, map_location="cuda:0")
model_state = checkpoint["model_state_dict"]

# RSL-RL Actor 네트워크 (정확한 구조: 48 → 512 → 256 → 128 → 12)
class ActorNetwork(torch.nn.Module):
    def __init__(self):
        super().__init__()
        # RSL-RL과 동일한 구조
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(48, 512),   # actor.0
            torch.nn.ELU(),             # actor.1
            torch.nn.Linear(512, 256),  # actor.2
            torch.nn.ELU(),             # actor.3
            torch.nn.Linear(256, 128),  # actor.4
            torch.nn.ELU(),             # actor.5
            torch.nn.Linear(128, 12),   # actor.6
        )

    def forward(self, obs):
        return self.actor(obs)

# 정책 네트워크 생성
policy = ActorNetwork().cuda()

# Actor 가중치만 로드
actor_state = {}
for key, value in model_state.items():
    if key.startswith('actor.'):
        actor_state[key] = value
        print(f"  Loading: {key} {value.shape}")

policy.load_state_dict(actor_state)
print(f"\nPolicy loaded successfully! ({len(actor_state)} layers)")
policy.eval()

# ============================================================
# World 생성
# ============================================================
print("\nCreating World...")
world = World(physics_dt=PHYSICS_DT, rendering_dt=1.0/60.0)

# Ground Plane 추가
print("Adding Ground Plane...")
world.scene.add_default_ground_plane()

# Spot 로봇 로드
print("\nLoading Spot Robot...")
assets_root_path = get_assets_root_path()

if assets_root_path is None:
    print("ERROR: Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit(1)

spot_usd_path = assets_root_path + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
spot_prim_path = "/World/Spot"

print(f"Asset path: {spot_usd_path}")

# Spot 로봇 추가
add_reference_to_stage(usd_path=spot_usd_path, prim_path=spot_prim_path)

spot = world.scene.add(
    Articulation(
        prim_path=spot_prim_path,
        name="spot_robot",
        position=np.array([0.0, 0.0, 0.7])  # 원본 컨트롤러와 동일
    )
)

print("\n" + "=" * 60)
print("  SUCCESS! Spot Robot Loaded")
print("=" * 60)

# World 리셋
print("\nResetting World...")
world.reset()

# PD 게인 설정 (v20과 동일)
print("\nSetting joint PD gains via USD DriveAPI...")
stage = world.stage
for joint_name in JOINT_NAMES:
    joint_path = f"/World/Spot/{joint_name}"
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim.IsValid():
        drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if drive_api:
            drive_api.GetStiffnessAttr().Set(KP)
            drive_api.GetDampingAttr().Set(KD)
print(f"  PD gains set: Kp={KP}, Kd={KD}")

# 관절 정보
num_dof = spot.num_dof
joint_names = spot.dof_names
print(f"\nSpot Robot Details:")
print(f"  - DOF: {num_dof}")
print(f"  - Joints: {joint_names}")

# v20과 동일: 하드코딩된 기본 자세 사용
world.play()
default_joint_pos = ISAACLAB_DEFAULT_POS.copy()
print(f"  - Using ISAACLAB default pose: {default_joint_pos}")

# 안정화 (원본 컨트롤러와 동일한 방식)
print("\nStabilizing robot (500 steps)...")
for i in range(500):
    action = ArticulationAction(joint_positions=default_joint_pos)
    spot.apply_action(action)
    world.step(render=False)
    if i % 100 == 0:
        pos, _ = spot.get_world_pose()
        joint_pos = spot.get_joint_positions()
        error = np.mean(np.abs(joint_pos - default_joint_pos))
        print(f"  Step {i}/500 | Height: {pos[2]:.3f}m | Joint Error: {error:.4f}")
print("  Stabilization complete!")

# ============================================================
# 키보드 컨트롤러
# ============================================================
class KeyboardController:
    def __init__(self):
        self.cmd_vx = 0.0   # 전진/후진
        self.cmd_vy = 0.0   # 횡이동
        self.cmd_yaw = 0.0  # 회전

        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.strafe_left = False
        self.strafe_right = False

        # v20 테스트와 동일한 속도
        self.default_vx = 1.0   # 기본 전진 속도 (정책이 정지 상태 미지원)
        self.max_vx = 1.0
        self.max_vy = 0.5
        self.max_yaw = 0.5

    def update(self):
        # 기본값: 전진 (정책이 정지 상태를 지원하지 않음)
        self.cmd_vx = self.default_vx
        self.cmd_vy = 0.0
        self.cmd_yaw = 0.0

        if self.forward:
            self.cmd_vx = self.max_vx
        if self.backward:
            self.cmd_vx = 0.0  # 정지 (후진 대신)
        if self.strafe_left:
            self.cmd_vy = self.max_vy
        if self.strafe_right:
            self.cmd_vy = -self.max_vy
        if self.left:
            self.cmd_yaw = self.max_yaw
        if self.right:
            self.cmd_yaw = -self.max_yaw

    def get_command(self):
        # v20과 동일하게 반환
        return np.array([self.cmd_vx, self.cmd_vy, self.cmd_yaw])

keyboard = KeyboardController()

def on_keyboard_event(event, *args, **kwargs):
    global keyboard

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.W:
            keyboard.forward = True
            print("[CMD] Forward")
        elif event.input == carb.input.KeyboardInput.S:
            keyboard.backward = True
            print("[CMD] Backward")
        elif event.input == carb.input.KeyboardInput.A:
            keyboard.left = True
            print("[CMD] Turn Left")
        elif event.input == carb.input.KeyboardInput.D:
            keyboard.right = True
            print("[CMD] Turn Right")
        elif event.input == carb.input.KeyboardInput.Q:
            keyboard.strafe_left = True
            print("[CMD] Strafe Left")
        elif event.input == carb.input.KeyboardInput.E:
            keyboard.strafe_right = True
            print("[CMD] Strafe Right")

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

    return True

# 키보드 구독
appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
keyboard_device = appwindow.get_keyboard()
sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard_device, on_keyboard_event)

# ============================================================
# 관측값 계산 (v20과 동일)
# ============================================================
def get_quat(spot):
    """v20과 동일 - 변환 없이 그대로 반환"""
    return spot.get_world_pose()[1]

def quat_rot_inv(q, v):
    """쿼터니언의 역회전을 벡터에 적용 (월드 → 로봇 좌표계)"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    qv = np.array([x, y, z])
    return v * (2.0 * w * w - 1.0) - np.cross(qv, v) * w * 2.0 + qv * np.dot(qv, v) * 2.0

def compute_observation(spot, cmd, last_action, default_pos):
    """
    관측값 계산 (v20과 동일한 방식)

    관측 구조 (48차원):
    - base_lin_vel: 3 (로봇 기준 선형 속도)
    - base_ang_vel: 3 (로봇 기준 각속도)
    - projected_gravity: 3 (로봇 기준 중력 방향)
    - velocity_commands: 3 (vx, vy, yaw)
    - joint_pos: 12 (기본값 대비 상대 위치)
    - joint_vel: 12 (관절 속도)
    - last_action: 12 (이전 액션)
    """
    # 로봇 상태 획득
    lin_vel = spot.get_linear_velocity()
    ang_vel = spot.get_angular_velocity()
    q = get_quat(spot)
    grav = np.array([0., 0., -1.])
    joint_pos = spot.get_joint_positions()
    joint_vel = spot.get_joint_velocities()

    # 관측값 조합 (v20과 동일)
    obs = np.concatenate([
        quat_rot_inv(q, lin_vel),      # 로봇 좌표계 선형 속도
        quat_rot_inv(q, ang_vel),      # 로봇 좌표계 각속도
        quat_rot_inv(q, grav),         # 로봇 좌표계 중력 방향
        cmd,                           # 속도 명령
        joint_pos - default_pos,       # 상대 관절 위치
        joint_vel,                     # 관절 속도
        last_action                    # 이전 액션
    ])

    return obs.astype(np.float32)

# ============================================================
# 메인 루프
# ============================================================
print("\n" + "=" * 60)
print("  Spot Robot Ready! (RL Policy Control)")
print("=" * 60)
print("\nKeyboard Controls:")
print("  W/S: Forward/Backward")
print("  A/D: Turn Left/Right")
print("  Q/E: Strafe Left/Right")
print("\nThe robot uses trained RL policy!")
print("=" * 60 + "\n")

steps = 0
last_action = np.zeros(12, dtype=np.float32)
current_action = np.zeros(12, dtype=np.float32)
min_height = 0.25  # 최소 높이

try:
    while simulation_app.is_running():
        # 키보드 명령 업데이트
        keyboard.update()
        cmd = keyboard.get_command()

        # v20과 동일: DECIMATION마다 정책 실행 및 액션 적용
        if steps % DECIMATION == 0:
            # 관측값 계산
            obs = compute_observation(spot, cmd, last_action, default_joint_pos)

            # 정책으로 액션 계산
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0).cuda()
                action_tensor = policy(obs_tensor)
                current_action = action_tensor.squeeze().cpu().numpy()

            # 마지막 액션 저장
            last_action = current_action.copy()

            # 액션 적용
            target_pos = default_joint_pos + current_action * ACTION_SCALE
            spot_action = ArticulationAction(joint_positions=target_pos)
            spot.apply_action(spot_action)

        # Physics step (render 없이)
        world.step(render=False)
        steps += 1

        # 렌더링은 별도로 낮은 빈도로 (10Hz)
        if steps % 50 == 0:  # 500Hz / 50 = 10Hz
            simulation_app.update()

        # 높이 체크 - 넘어지면 경고
        position, _ = spot.get_world_pose()
        if position[2] < min_height:
            print(f"WARNING: Robot falling! Height: {position[2]:.2f}m")

        # 상태 출력 (100 스텝마다)
        if steps % 100 == 0:
            velocity = spot.get_linear_velocity()
            vel_mag = np.linalg.norm(velocity[:2])

            is_moving = abs(cmd[0]) > 0.01 or abs(cmd[1]) > 0.01 or abs(cmd[2]) > 0.01
            status = "Walking" if is_moving else "Standing"

            action_mag = np.linalg.norm(current_action)
            print(f"Step {steps:5d} | Height:{position[2]:.2f}m | Vel:{vel_mag:.2f}m/s | Action:{action_mag:.2f} | {status}")

except KeyboardInterrupt:
    print("\n\nStopped by user.")

# 정리
print("\nCleaning up...")
input_interface.unsubscribe_to_keyboard_events(keyboard_device, sub_keyboard)
world.stop()
simulation_app.close()
print("Done!")
