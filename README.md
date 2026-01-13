# Spot Robot Simulation with AI-Powered Movement Optimization

AWS 클라우드 환경에서 NVIDIA Isaac Sim을 활용한 Boston Dynamics Spot 로봇 시뮬레이션 및 AI 기반 움직임 최적화 시스템

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              AWS Cloud Architecture                                   │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                       │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐             │
│  │   EC2 g5.4xlarge │     │   Amazon S3      │     │  Amazon Bedrock  │             │
│  │   ┌────────────┐ │     │   ┌────────────┐ │     │  ┌────────────┐  │             │
│  │   │ Isaac Sim  │ │     │   │ Video Data │ │     │  │Claude 4.5  │  │             │
│  │   │ + Spot     │─┼────▶│   │ + Logs     │─┼────▶│  │ Sonnet     │  │             │
│  │   │ Robot      │ │     │   │            │ │     │  │(CRIS API)  │  │             │
│  │   └────────────┘ │     │   └────────────┘ │     │  └─────┬──────┘  │             │
│  │   NVIDIA A10G    │     │                  │     │        │         │             │
│  └────────┬─────────┘     └────────┬─────────┘     └────────┼─────────┘             │
│           │                        │                        │                        │
│           │    ┌───────────────────┼────────────────────────┘                        │
│           │    │                   │                                                 │
│           ▼    ▼                   ▼                                                 │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐             │
│  │  NICE DCV        │     │ Kinesis Data     │     │ Amazon Athena    │             │
│  │  Remote Desktop  │     │ Firehose         │     │ Log Analysis     │             │
│  │  (8443 port)     │     │ (Real-time)      │     │                  │             │
│  └──────────────────┘     └──────────────────┘     └──────────────────┘             │
│                                                                                       │
└─────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                         Video Understanding & Analysis                                │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                       │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐             │
│  │  Twelve Labs     │     │  Movement        │     │  Optimized       │             │
│  │  Pegasus         │────▶│  Pattern         │────▶│  Control         │             │
│  │  (Video AI)      │     │  Analysis        │     │  Parameters      │             │
│  └──────────────────┘     └──────────────────┘     └──────────────────┘             │
│         │                         │                         │                        │
│         │    Video Analysis       │   Joint/Position Logs   │   New Gait Patterns   │
│         └─────────────────────────┴─────────────────────────┴───────────────────────│
│                                                                                       │
└─────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              Feedback Loop                                            │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                       │
│   1. Simulation Video + Logs  ──▶  2. Pegasus Analysis  ──▶  3. LLM Processing      │
│                                                                                       │
│   4. Parameter Optimization   ◀──  5. Movement Improvement  ◀──  Pattern Detection  │
│                                                                                       │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Isaac Sim  │───▶│   S3        │───▶│  Firehose   │───▶│  Athena     │
│  Simulation │    │   Storage   │    │  Streaming  │    │  Query      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                                      │
      │                  ▼                                      │
      │           ┌─────────────┐                              │
      │           │ Twelve Labs │                              │
      │           │   Pegasus   │                              │
      │           └──────┬──────┘                              │
      │                  │                                      │
      │                  ▼                                      ▼
      │           ┌─────────────────────────────────────────────┐
      │           │        Amazon Bedrock (Claude 4.5)          │
      │           │   - Video understanding results             │
      │           │   - Joint position/velocity logs            │
      │           │   - Movement pattern analysis               │
      │           └──────────────────┬──────────────────────────┘
      │                              │
      │                              ▼
      │                    ┌─────────────────┐
      └───────────────────│  Optimized Gait │
           Feedback       │   Parameters    │
                          └─────────────────┘
```

## Project Structure

```
48.robot_nvidia/
├── README.md                    # This file
├── setup/                       # Installation scripts
│   ├── 01_ec2_setup.sh         # EC2 instance setup
│   ├── 02_nvidia_driver.sh     # NVIDIA driver installation
│   ├── 03_isaac_sim_install.sh # Isaac Sim installation
│   └── 04_nicedcv_setup.sh     # NICE DCV configuration
├── src/                         # Source code
│   └── spot_robot_controller.py # Main robot control script
└── docs/                        # Additional documentation
    └── ...
```

## Quick Start

### 1. EC2 Instance Setup

```bash
# Instance Type: g5.4xlarge (NVIDIA A10G GPU)
# OS: Ubuntu 22.04 LTS
# Security Group: SSH(22), DCV(8443)

# SSH Connection
ssh -i your-key.pem ubuntu@<EC2-PUBLIC-IP>
```

### 2. Environment Setup

```bash
# Upload and run setup scripts
scp -i your-key.pem setup/*.sh ubuntu@<EC2-IP>:~/
ssh -i your-key.pem ubuntu@<EC2-IP>

chmod +x *.sh
./01_ec2_setup.sh
./02_nvidia_driver.sh
sudo reboot

# After reboot
./03_isaac_sim_install.sh
./04_nicedcv_setup.sh
```

### 3. NICE DCV Connection

```bash
# Server-side: Create DCV session
sudo dcv create-session --type=virtual --owner ubuntu ubuntu-session

# Client-side: Connect via NICE DCV Viewer
# Address: <EC2-IP>:8443
# Username: ubuntu
# Password: <your-password>
```

### 4. Run Spot Robot Simulation

```bash
cd ~/isaac-sim/IsaacSim/_build/linux-x86_64/release
./python.sh ~/spot_robot_controller.py
```

## Core Components

### Spot Robot Control Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Spot Robot Controller                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │  Keyboard   │───▶│   Gait      │───▶│ Articulation│         │
│  │  Input      │    │  Generator  │    │   Action    │         │
│  │  (WASD)     │    │             │    │             │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│                            │                   │                 │
│                            ▼                   ▼                 │
│                     ┌─────────────┐    ┌─────────────┐         │
│                     │  Trotting   │    │  12 DOF     │         │
│                     │  Gait       │    │  Joint      │         │
│                     │  Pattern    │    │  Control    │         │
│                     └─────────────┘    └─────────────┘         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Joint Configuration (12 DOF)

```
Spot Robot Joints:
├── Front Left (FL)     ├── Front Right (FR)
│   ├── fl_hx (hip_x)   │   ├── fr_hx (hip_x)
│   ├── fl_hy (hip_y)   │   ├── fr_hy (hip_y)
│   └── fl_kn (knee)    │   └── fr_kn (knee)
│                       │
├── Hind Left (HL)      └── Hind Right (HR)
│   ├── hl_hx (hip_x)       ├── hr_hx (hip_x)
│   ├── hl_hy (hip_y)       ├── hr_hy (hip_y)
│   └── hl_kn (knee)        └── hr_kn (knee)

Joint Index Mapping:
  Index 0-3:  hip_x [fl, fr, hl, hr]
  Index 4-7:  hip_y [fl, fr, hl, hr]
  Index 8-11: knee  [fl, fr, hl, hr]
```

### Trotting Gait Pattern

```python
# Diagonal leg pairs move together
# Phase 0:   FL + HR (Front-Left, Hind-Right)
# Phase π:   FR + HL (Front-Right, Hind-Left)

def compute_walking_pose(phase, cmd_x, cmd_yaw):
    """
    Trotting gait computation
    - phase: 0 ~ 2π
    - cmd_x: forward/backward (-1 to 1)
    - cmd_yaw: rotation (-1 to 1)
    """
    target = standing_pose.copy()

    for leg in ['FL', 'FR', 'HL', 'HR']:
        # Diagonal synchronization
        if leg in ['FL', 'HR']:
            leg_phase = phase
        else:  # FR, HL
            leg_phase = (phase + np.pi) % (2 * np.pi)

        # Swing phase: leg in air, moving forward
        # Stance phase: leg on ground, pushing backward
        is_swing = leg_phase > np.pi

        # Apply hip_y and knee offsets based on phase
        ...

    return target
```

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Physics DT | 1/120 sec | Physics simulation timestep |
| Rendering DT | 1/60 sec | Rendering timestep |
| Gait Speed | 0.15 | Walking phase increment |
| Step Height | 0.3 rad | Knee lift during swing |
| Step Length | 0.4 rad | Hip movement range |

## USD Default Pose (Stable Standing)

```python
# Values from Isaac Sim Spot USD file
standing_pose = {
    'hip_x': [0.1, -0.1, 0.1, -0.1],    # FL, FR, HL, HR
    'hip_y': [0.9, 0.9, 1.1, 1.1],      # Front legs: 0.9, Hind legs: 1.1
    'knee':  [-1.5, -1.5, -1.5, -1.5]   # All legs
}
```

## AWS Services Integration

### Amazon S3
- Simulation video storage
- Robot movement logs (position, velocity, joint angles)

### Amazon Kinesis Data Firehose
- Real-time log streaming
- Automatic S3 delivery

### Amazon Athena
- SQL queries on movement logs
- Pattern analysis

### Amazon Bedrock (Claude 4.5 Sonnet)
- CRIS endpoint for API access
- Movement pattern analysis
- Gait parameter optimization suggestions

### Twelve Labs Pegasus
- Video understanding model
- Visual movement analysis
- Anomaly detection in robot motion

## Keyboard Controls

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Turn Left |
| D | Turn Right |
| Space | Stop |

## Troubleshooting

### Robot Falls at Startup
- Use USD default pose instead of custom values
- Start at appropriate height (0.7m)
- Apply gradual stabilization phase

### No Movement Response
- Increase gait parameters (step_height, step_length)
- Check keyboard focus on 3D viewport

### DCV Connection Issues
- Remove `web-url-path` from `/etc/dcv/dcv.conf`
- Ensure `~/.xsession` exists with XFCE4 startup

## References

- [NVIDIA Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Boston Dynamics Spot](https://www.bostondynamics.com/spot)
- [NICE DCV User Guide](https://docs.aws.amazon.com/dcv/)
- [Twelve Labs Pegasus](https://twelvelabs.io/)
- [Amazon Bedrock](https://aws.amazon.com/bedrock/)

## License

MIT License

## Author

Spot Robot Simulation Project Team
