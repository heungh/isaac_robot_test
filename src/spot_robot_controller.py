"""
Spot Robot Standalone Simulation with Physics-based PD Control
- Isaac Sim을 시작하고 Spot 로봇과 Ground Plane을 자동으로 로드
- 물리 기반 PD Controller로 자연스러운 보행
- 키보드 입력으로 로봇 제어 (WASD: 이동, Q/E: 회전, Space: 정지)
"""

import numpy as np
from isaacsim import SimulationApp

# Isaac Sim 시작
print("Starting Isaac Sim...")
simulation_app = SimulationApp({"headless": False})

# 키보드 입력 처리를 위한 임포트
import carb.input
import omni.appwindow

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

print("="*60)
print("  Spot Robot Simulation - Physics-based PD Control")
print("="*60)

# World 생성
print("\nCreating World...")
world = World(physics_dt=1.0/120.0, rendering_dt=1.0/60.0)  # 물리 120Hz, 렌더링 60Hz

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

# Articulation으로 추가 - 충분히 높은 위치에서 시작 (땅 관통 방지)
spot = world.scene.add(
    Articulation(
        prim_path=spot_prim_path,
        name="spot_robot",
        position=np.array([0.0, 0.0, 0.7])  # 높게 시작해서 떨어지게
    )
)

print("\n" + "="*60)
print("  SUCCESS! Spot Robot Loaded")
print("="*60)

# World 리셋
print("\nResetting World...")
world.reset()

# 관절 정보 출력
num_dof = spot.num_dof
joint_names = spot.dof_names
print(f"\nSpot Robot Details:")
print(f"  - Degrees of Freedom: {num_dof}")
print(f"  - Joint Names: {joint_names}")

# ============================================================
# 관절 인덱스 매핑 (실제 순서 기반)
# ============================================================
# 실제 순서: ['fl_hx', 'fr_hx', 'hl_hx', 'hr_hx', 'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy', 'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn']
# 인덱스 0-3: hip_x (fl, fr, hl, hr)
# 인덱스 4-7: hip_y (fl, fr, hl, hr)
# 인덱스 8-11: knee (fl, fr, hl, hr)

print("\nMapping joint indices...")
joint_idx = {
    'fl_hx': 0, 'fr_hx': 1, 'hl_hx': 2, 'hr_hx': 3,
    'fl_hy': 4, 'fr_hy': 5, 'hl_hy': 6, 'hr_hy': 7,
    'fl_kn': 8, 'fr_kn': 9, 'hl_kn': 10, 'hr_kn': 11
}

# 다리별 인덱스 그룹
leg_indices = {
    'FL': {'hx': 0, 'hy': 4, 'kn': 8},
    'FR': {'hx': 1, 'hy': 5, 'kn': 9},
    'HL': {'hx': 2, 'hy': 6, 'kn': 10},
    'HR': {'hx': 3, 'hy': 7, 'kn': 11}
}
print(f"  Joint indices: {joint_idx}")

# ============================================================
# Standing Pose 정의 - USD 파일의 기본값 사용
# ============================================================
print("\nGetting default pose from USD file...")

# 시뮬레이션 시작
print("Starting simulation...")
world.play()

# USD의 기본 자세를 standing pose로 사용! (가장 안정적)
standing_pose = spot.get_joint_positions().copy()
print(f"  USD default pose (using as standing): {standing_pose}")

# USD 기본값이 모두 0이면 수동 설정 (fallback)
if np.allclose(standing_pose, 0):
    print("  WARNING: USD default is all zeros, using manual values...")
    # 인덱스 0-3: hip_x
    standing_pose[0:4] = [0.1, -0.1, 0.1, -0.1]
    # 인덱스 4-7: hip_y (앞다리 0.9, 뒷다리 1.1)
    standing_pose[4:8] = [0.9, 0.9, 1.1, 1.1]
    # 인덱스 8-11: knee
    standing_pose[8:12] = [-1.5, -1.5, -1.5, -1.5]
    print(f"  Manual standing pose: {standing_pose}")

# ============================================================
# 안정화 단계 - USD 기본 자세 유지
# ============================================================
print("\nStabilizing robot (500 steps)...")
print("  Maintaining USD default pose...")

for i in range(500):
    # USD 기본 자세 유지
    action = ArticulationAction(joint_positions=standing_pose)
    spot.apply_action(action)
    world.step(render=True)

    if i % 100 == 0:
        position, _ = spot.get_world_pose()
        joint_pos = spot.get_joint_positions()
        error = np.mean(np.abs(joint_pos - standing_pose))
        print(f"  Step {i:3d}/500 | Height: {position[2]:.3f}m | Error: {error:.4f}")

print("  Stabilization complete!")

# ============================================================
# 키보드 제어 설정
# ============================================================
print("\n" + "="*60)
print("  🐕 Spot Robot is Ready! (Physics-based Control)")
print("="*60)
print("\nKeyboard Controls:")
print("  W: Forward")
print("  S: Backward")
print("  A: Turn Left")
print("  D: Turn Right")
print("  Space: Stop")
print("\nNote: Robot moves by walking, not teleportation!")
print("="*60)

# 키보드 입력 상태
class KeyboardState:
    def __init__(self):
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.stop = False

keyboard_state = KeyboardState()

def on_keyboard_event(event, *args, **kwargs):
    global keyboard_state

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.W:
            keyboard_state.forward = True
            print("🔵 Forward")
        elif event.input == carb.input.KeyboardInput.S:
            keyboard_state.backward = True
            print("🔵 Backward")
        elif event.input == carb.input.KeyboardInput.A:
            keyboard_state.left = True
            print("🔵 Turn Left")
        elif event.input == carb.input.KeyboardInput.D:
            keyboard_state.right = True
            print("🔵 Turn Right")
        elif event.input == carb.input.KeyboardInput.SPACE:
            keyboard_state.stop = True
            print("🔵 Stop")

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if event.input == carb.input.KeyboardInput.W:
            keyboard_state.forward = False
        elif event.input == carb.input.KeyboardInput.S:
            keyboard_state.backward = False
        elif event.input == carb.input.KeyboardInput.A:
            keyboard_state.left = False
        elif event.input == carb.input.KeyboardInput.D:
            keyboard_state.right = False
        elif event.input == carb.input.KeyboardInput.SPACE:
            keyboard_state.stop = False

    return True

# 키보드 입력 구독
appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()
sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard_event)

# ============================================================
# 물리 기반 보행 파라미터
# ============================================================
gait_phase = 0.0
gait_speed = 0.15         # 보행 속도 (증가)
step_height = 0.3         # 발 들어올리는 높이 (크게 증가)
step_length = 0.4         # 걸음 길이 (크게 증가)

def compute_walking_pose(phase, cmd_x, cmd_yaw):
    """
    물리 기반 보행을 위한 전체 관절 각도 계산

    phase: 0 ~ 2π
    cmd_x: 전진/후진 명령 (-1 ~ 1)
    cmd_yaw: 회전 명령 (-1 ~ 1)

    Returns: numpy array of 12 joint positions
    """
    target = standing_pose.copy()

    # 움직임 강도
    movement_scale = abs(cmd_x) + abs(cmd_yaw) * 0.5
    if movement_scale < 0.1:
        return target  # Standing pose

    # 각 다리에 대해 계산
    # 다리 순서: FL=0, FR=1, HL=2, HR=3
    legs = ['FL', 'FR', 'HL', 'HR']

    for i, leg in enumerate(legs):
        idx = leg_indices[leg]

        # Trotting gait: 대각선 다리 동기화
        # FL(0) + HR(3) 같이, FR(1) + HL(2) 같이
        if i in [0, 3]:  # FL, HR
            leg_phase = phase
        else:  # FR, HL
            leg_phase = (phase + np.pi) % (2 * np.pi)

        # Swing vs Stance phase
        is_swing = leg_phase > np.pi

        if is_swing:
            # Swing phase: 발이 공중으로 올라가며 앞으로 이동
            progress = (leg_phase - np.pi) / np.pi  # 0 to 1

            # hip_y: 앞으로 스윙
            hip_y_offset = step_length * (progress - 0.5) * np.sign(cmd_x)

            # knee: 위로 들기
            knee_offset = step_height * np.sin(progress * np.pi)

            target[idx['hy']] = standing_pose[idx['hy']] + hip_y_offset
            target[idx['kn']] = standing_pose[idx['kn']] + knee_offset
        else:
            # Stance phase: 발이 땅에서 뒤로 밀기
            progress = leg_phase / np.pi  # 0 to 1

            # hip_y: 뒤로 밀기 (추진력!)
            hip_y_offset = step_length * (0.5 - progress) * np.sign(cmd_x)

            target[idx['hy']] = standing_pose[idx['hy']] + hip_y_offset

        # 회전 명령 적용 (좌우 다리 비대칭) - 힘 증가
        if abs(cmd_yaw) > 0.1:
            if i in [0, 2]:  # Left legs (FL, HL)
                target[idx['hy']] += cmd_yaw * 0.2
            else:  # Right legs (FR, HR)
                target[idx['hy']] -= cmd_yaw * 0.2

    return target

# ============================================================
# 메인 시뮬레이션 루프
# ============================================================
steps = 0
print("\nSimulation running. Use W/S/A/D to control Spot!")
print("Robot will walk using physics - no teleportation!\n")

try:
    while simulation_app.is_running():
        world.step(render=True)
        steps += 1

        # 키보드 입력에 따른 명령
        cmd_x = 0.0
        cmd_yaw = 0.0

        if keyboard_state.forward:
            cmd_x = 1.0
        if keyboard_state.backward:
            cmd_x = -1.0
        if keyboard_state.left:
            cmd_yaw = 1.0
        if keyboard_state.right:
            cmd_yaw = -1.0
        if keyboard_state.stop:
            cmd_x = cmd_yaw = 0.0

        # 움직임 여부 확인
        is_moving = abs(cmd_x) > 0.01 or abs(cmd_yaw) > 0.01

        # 보행 phase 업데이트
        if is_moving:
            gait_phase += gait_speed
            if gait_phase >= 2 * np.pi:
                gait_phase = 0.0
        else:
            gait_phase = 0.0

        # 목표 관절 각도 계산 (새로운 함수 사용)
        target_positions = compute_walking_pose(gait_phase, cmd_x, cmd_yaw)

        # 🎯 관절 제어 (물리 기반!)
        # - Base를 직접 움직이지 않음
        # - 관절에 힘이 가해져서 자연스럽게 전진
        action = ArticulationAction(joint_positions=target_positions)
        spot.apply_action(action)

        # 50 스텝마다 상태 출력
        if steps % 50 == 0:
            position, orientation = spot.get_world_pose()
            velocity = spot.get_linear_velocity()
            vel_mag = np.linalg.norm(velocity[:2])  # XY 속도만

            status = "🚶 Walking" if is_moving else "🛑 Standing"
            phase_pct = gait_phase / (2 * np.pi) * 100

            print(f"Step {steps:4d} | Pos:[{position[0]:5.2f},{position[1]:5.2f},{position[2]:5.2f}] | "
                  f"Vel:{vel_mag:.2f}m/s | {status} | Phase:{phase_pct:3.0f}%")

except KeyboardInterrupt:
    print("\n\nSimulation stopped by user.")

# 정리
print("\nCleaning up...")
input_interface.unsubscribe_to_keyboard_events(keyboard, sub_keyboard)
world.stop()
simulation_app.close()
print("Simulation ended. Goodbye!")
