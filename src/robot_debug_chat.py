"""
Robot Debug Chat Interface
- Athena를 통한 로그 조회
- Twelve Labs Pegasus 비디오 분석
- Bedrock Claude 4.5 Sonnet 종합 분석
- 코드 수정 제안 및 자동 적용
- /reflect 명령으로 AI 제안사항 실제 반영
"""

import os
import sys
import json
import time
import re
import boto3
import argparse
from datetime import datetime, timedelta
from typing import Optional, Dict, List, Any
from dataclasses import dataclass, field

# ============================================================
# Configuration
# ============================================================
AWS_REGION = os.environ.get('AWS_REGION', 'ap-northeast-2')
S3_BUCKET = os.environ.get('S3_BUCKET', '')
GLUE_DATABASE = os.environ.get('GLUE_DATABASE', 'spot-robot-debug_db')
ATHENA_WORKGROUP = os.environ.get('ATHENA_WORKGROUP', 'spot-robot-debug-workgroup')
TWELVELABS_API_KEY = os.environ.get('TWELVELABS_API_KEY', '')
DYNAMODB_TABLE = os.environ.get('DYNAMODB_TABLE', 'spot-robot-debug-parameter-history')

# Bedrock model ID
BEDROCK_MODEL_ID = "anthropic.claude-sonnet-4-5-20250514-v1:0"

# Standalone 코드 경로 (EC2 환경)
STANDALONE_CODE_PATH = os.environ.get('STANDALONE_CODE_PATH', 'src/spot_rl_standalone.py')


# ============================================================
# Pending Suggestions (AI 제안 사항 저장)
# ============================================================
@dataclass
class ParameterSuggestion:
    """AI가 제안한 파라미터 변경"""
    param_name: str
    old_value: str
    new_value: str
    reason: str
    confidence: str = "medium"  # low, medium, high


@dataclass
class CodeBlockSuggestion:
    """AI가 제안한 코드 블록 변경"""
    description: str
    original_code: str
    modified_code: str
    file_path: str
    reason: str


@dataclass
class PendingSuggestions:
    """AI 분석 결과에서 추출한 대기 중인 제안 사항"""
    parameters: List[ParameterSuggestion] = field(default_factory=list)
    code_blocks: List[CodeBlockSuggestion] = field(default_factory=list)
    summary: str = ""
    timestamp: str = ""

    def clear(self):
        """제안 사항 초기화"""
        self.parameters = []
        self.code_blocks = []
        self.summary = ""
        self.timestamp = ""

    def has_suggestions(self) -> bool:
        """대기 중인 제안이 있는지 확인"""
        return len(self.parameters) > 0 or len(self.code_blocks) > 0

    def __str__(self) -> str:
        lines = []
        if self.parameters:
            lines.append(f"파라미터 변경 ({len(self.parameters)}개):")
            for i, p in enumerate(self.parameters, 1):
                lines.append(f"  {i}. {p.param_name}: {p.old_value} -> {p.new_value}")
                lines.append(f"     이유: {p.reason}")
        if self.code_blocks:
            lines.append(f"코드 블록 변경 ({len(self.code_blocks)}개):")
            for i, c in enumerate(self.code_blocks, 1):
                lines.append(f"  {i}. {c.description}")
                lines.append(f"     파일: {c.file_path}")
        return "\n".join(lines) if lines else "대기 중인 제안 없음"


# ============================================================
# AWS Clients
# ============================================================
class AWSClients:
    """AWS 서비스 클라이언트 관리"""

    def __init__(self, region: str):
        self.region = region
        self.athena = boto3.client('athena', region_name=region)
        self.s3 = boto3.client('s3', region_name=region)
        self.bedrock = boto3.client('bedrock-runtime', region_name=region)
        self.secrets = boto3.client('secretsmanager', region_name=region)
        self.dynamodb = boto3.resource('dynamodb', region_name=region)

    def get_secret(self, secret_name: str) -> dict:
        """Secrets Manager에서 비밀 조회"""
        try:
            response = self.secrets.get_secret_value(SecretId=secret_name)
            return json.loads(response['SecretString'])
        except Exception as e:
            print(f"[ERROR] Failed to get secret: {e}")
            return {}


# ============================================================
# Athena Log Query
# ============================================================
class LogQueryEngine:
    """Athena를 통한 로그 조회"""

    def __init__(self, clients: AWSClients, database: str, workgroup: str, s3_bucket: str):
        self.clients = clients
        self.database = database
        self.workgroup = workgroup
        self.output_location = f"s3://{s3_bucket}/athena-results/"

    def execute_query(self, query: str, timeout: int = 60) -> List[Dict]:
        """Athena 쿼리 실행 및 결과 반환"""
        try:
            # 쿼리 실행
            response = self.clients.athena.start_query_execution(
                QueryString=query,
                QueryExecutionContext={'Database': self.database},
                WorkGroup=self.workgroup
            )
            query_id = response['QueryExecutionId']

            # 쿼리 완료 대기
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.clients.athena.get_query_execution(
                    QueryExecutionId=query_id
                )['QueryExecution']['Status']

                state = status['State']
                if state == 'SUCCEEDED':
                    break
                elif state in ['FAILED', 'CANCELLED']:
                    raise Exception(f"Query failed: {status.get('StateChangeReason', 'Unknown')}")

                time.sleep(1)
            else:
                raise Exception("Query timeout")

            # 결과 조회
            results = self.clients.athena.get_query_results(QueryExecutionId=query_id)

            # 결과 파싱
            columns = [col['Label'] for col in results['ResultSet']['ResultSetMetadata']['ColumnInfo']]
            rows = []
            for row in results['ResultSet']['Rows'][1:]:  # Skip header
                row_data = {}
                for i, cell in enumerate(row['Data']):
                    row_data[columns[i]] = cell.get('VarCharValue', '')
                rows.append(row_data)

            return rows

        except Exception as e:
            print(f"[ERROR] Query failed: {e}")
            return []

    def get_session_logs(self, session_id: str, limit: int = 100) -> List[Dict]:
        """특정 세션의 로그 조회"""
        query = f"""
        SELECT timestamp, step, height, velocity_x, velocity_y, action_norm,
               cmd_vx, cmd_vy, cmd_yaw, status, is_fallen
        FROM robot_logs
        WHERE session_id = '{session_id}'
        ORDER BY step DESC
        LIMIT {limit}
        """
        return self.execute_query(query)

    def get_fall_events(self, session_id: str = None, hours: int = 24) -> List[Dict]:
        """넘어진 이벤트 조회"""
        where_clause = "WHERE is_fallen = true"
        if session_id:
            where_clause += f" AND session_id = '{session_id}'"

        query = f"""
        SELECT session_id, timestamp, step, height, velocity_x, velocity_y,
               action_norm, cmd_vx, cmd_vy, cmd_yaw
        FROM robot_logs
        {where_clause}
        ORDER BY timestamp DESC
        LIMIT 50
        """
        return self.execute_query(query)

    def get_logs_around_time(self, session_id: str, step: int, window: int = 50) -> List[Dict]:
        """특정 시점 전후 로그 조회"""
        query = f"""
        SELECT timestamp, step, height, velocity_x, velocity_y, action_norm,
               cmd_vx, cmd_vy, cmd_yaw, status, observation, action
        FROM robot_logs
        WHERE session_id = '{session_id}'
          AND step BETWEEN {step - window} AND {step + window}
        ORDER BY step
        """
        return self.execute_query(query)

    def get_recent_sessions(self, limit: int = 10) -> List[Dict]:
        """최근 세션 목록 조회"""
        query = f"""
        SELECT session_id,
               MIN(timestamp) as start_time,
               MAX(timestamp) as end_time,
               MAX(step) as total_steps,
               SUM(CASE WHEN is_fallen THEN 1 ELSE 0 END) as fall_count
        FROM robot_logs
        GROUP BY session_id
        ORDER BY start_time DESC
        LIMIT {limit}
        """
        return self.execute_query(query)


# ============================================================
# Twelve Labs Video Analysis
# ============================================================
class VideoAnalyzer:
    """Twelve Labs Pegasus를 통한 비디오 분석"""

    def __init__(self, api_key: str):
        self.api_key = api_key
        self.enabled = bool(api_key)

        if self.enabled:
            try:
                import requests
                self.requests = requests
                print("[Twelve Labs] API configured")
            except ImportError:
                print("[WARNING] requests library not found")
                self.enabled = False

    def analyze_video(self, video_url: str, query: str) -> str:
        """비디오 분석 요청"""
        if not self.enabled:
            return "Video analysis not available (API key not configured)"

        # Twelve Labs Pegasus API 호출
        # Note: 실제 API 엔드포인트와 형식은 Twelve Labs 문서 참조
        try:
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            payload = {
                "video_url": video_url,
                "query": query
            }

            response = self.requests.post(
                "https://api.twelvelabs.io/v1/analyze",
                headers=headers,
                json=payload,
                timeout=60
            )

            if response.status_code == 200:
                return response.json().get('analysis', 'No analysis result')
            else:
                return f"Video analysis failed: {response.status_code}"

        except Exception as e:
            return f"Video analysis error: {e}"

    def analyze_robot_fall(self, video_url: str) -> str:
        """로봇 넘어짐 분석"""
        query = """
        Analyze this robot simulation video and identify:
        1. The exact moment when the robot starts to lose balance
        2. The direction of the fall (forward, backward, left, right)
        3. Any abnormal leg movements or poses before the fall
        4. Estimated cause of the instability
        """
        return self.analyze_video(video_url, query)


# ============================================================
# Bedrock Claude Analysis
# ============================================================
class ClaudeAnalyzer:
    """Bedrock Claude를 통한 종합 분석"""

    def __init__(self, clients: AWSClients, model_id: str = BEDROCK_MODEL_ID):
        self.clients = clients
        self.model_id = model_id

        # 시스템 프롬프트
        self.system_prompt = """You are an expert robotics engineer specializing in quadruped robot locomotion and reinforcement learning policy debugging.

Your expertise includes:
- Boston Dynamics Spot robot kinematics and dynamics
- RSL-RL (Robotic Systems Lab Reinforcement Learning) framework
- Isaac Sim/Lab simulation environment
- PD control and actuator modeling
- Observation space design and action scaling

When analyzing robot simulation logs and videos, you should:
1. Identify patterns that indicate instability or failure
2. Correlate observations with actions to find problematic sequences
3. Suggest specific parameter adjustments (PD gains, action scale, etc.)
4. Provide code modifications when necessary

Always provide actionable recommendations with specific values or code changes.

IMPORTANT: When suggesting parameter changes, ALWAYS include a JSON block in this exact format:
```json
{
  "suggestions": {
    "parameters": [
      {
        "name": "ACTION_SCALE",
        "old_value": "0.2",
        "new_value": "0.15",
        "reason": "Reduce oscillation during walking",
        "confidence": "high"
      }
    ],
    "code_blocks": [
      {
        "description": "Update PD gains for stability",
        "file": "spot_rl_standalone.py",
        "original": "KP = 60.0",
        "modified": "KP = 50.0",
        "reason": "Lower Kp reduces overshoot"
      }
    ],
    "summary": "Brief summary of all changes"
  }
}
```

Parameters you can modify:
- ACTION_SCALE: Action scaling factor (default 0.2)
- KP: Proportional gain (default 60.0)
- KD: Derivative gain (default 1.5)
- DECIMATION: Control decimation (default 10)
- CMD_VX_SCALE, CMD_VY_SCALE, CMD_YAW_SCALE: Command scaling"""

    def analyze(self, user_message: str, context: dict = None) -> str:
        """Claude로 분석 요청"""
        try:
            # 컨텍스트 구성
            full_message = user_message
            if context:
                if context.get('logs'):
                    full_message += f"\n\n## Log Data\n```json\n{json.dumps(context['logs'], indent=2)}\n```"
                if context.get('video_analysis'):
                    full_message += f"\n\n## Video Analysis\n{context['video_analysis']}"
                if context.get('session_id'):
                    full_message += f"\n\n## Session ID: {context['session_id']}"

            # Bedrock API 호출
            body = json.dumps({
                "anthropic_version": "bedrock-2023-05-31",
                "max_tokens": 4096,
                "system": self.system_prompt,
                "messages": [
                    {"role": "user", "content": full_message}
                ]
            })

            response = self.clients.bedrock.invoke_model(
                modelId=self.model_id,
                body=body,
                contentType="application/json",
                accept="application/json"
            )

            result = json.loads(response['body'].read())
            return result['content'][0]['text']

        except Exception as e:
            return f"Analysis error: {e}"

    def analyze_fall_cause(self, logs: List[Dict], video_analysis: str = None) -> str:
        """넘어짐 원인 분석"""
        message = """Analyze the following robot simulation data and identify the root cause of the fall.

Please provide:
1. **Root Cause Analysis**: What caused the robot to fall?
2. **Key Observations**: Which observation values were abnormal before the fall?
3. **Action Analysis**: Were the actions appropriate for the situation?
4. **Recommended Fixes**: Specific parameter changes or code modifications

Focus on these potential issues:
- PD gain settings (current: Kp=60, Kd=1.5)
- Action scale (current: 0.2)
- Observation calculation errors
- Decimation timing issues
"""

        context = {
            'logs': logs,
            'video_analysis': video_analysis
        }

        return self.analyze(message, context)

    def suggest_code_fix(self, issue_description: str, current_code: str = None) -> str:
        """코드 수정 제안"""
        message = f"""Based on the following issue, suggest specific code modifications:

## Issue
{issue_description}

"""
        if current_code:
            message += f"""## Current Code
```python
{current_code}
```

"""

        message += """Please provide:
1. Explanation of what needs to change
2. The modified code with comments
3. Expected impact of the changes

IMPORTANT: Include the JSON suggestions block as specified in the system prompt.
"""

        return self.analyze(message)

    def parse_suggestions(self, response: str) -> PendingSuggestions:
        """Claude 응답에서 제안 사항 추출"""
        pending = PendingSuggestions()
        pending.timestamp = datetime.utcnow().isoformat() + "Z"

        # JSON 블록 찾기
        json_pattern = r'```json\s*(\{[\s\S]*?"suggestions"[\s\S]*?\})\s*```'
        match = re.search(json_pattern, response)

        if match:
            try:
                data = json.loads(match.group(1))
                suggestions = data.get('suggestions', {})

                # 파라미터 제안 파싱
                for param in suggestions.get('parameters', []):
                    pending.parameters.append(ParameterSuggestion(
                        param_name=param.get('name', ''),
                        old_value=param.get('old_value', ''),
                        new_value=param.get('new_value', ''),
                        reason=param.get('reason', ''),
                        confidence=param.get('confidence', 'medium')
                    ))

                # 코드 블록 제안 파싱
                for block in suggestions.get('code_blocks', []):
                    pending.code_blocks.append(CodeBlockSuggestion(
                        description=block.get('description', ''),
                        original_code=block.get('original', ''),
                        modified_code=block.get('modified', ''),
                        file_path=block.get('file', STANDALONE_CODE_PATH),
                        reason=block.get('reason', '')
                    ))

                pending.summary = suggestions.get('summary', '')

            except json.JSONDecodeError as e:
                print(f"[WARNING] Failed to parse suggestions JSON: {e}")

        # JSON이 없는 경우 기존 패턴 매칭으로 파라미터 추출
        if not pending.has_suggestions():
            param_pattern = r'(\w+_?\w*)\s*[=:]\s*([\d.]+)\s*(?:->|→|to)\s*([\d.]+)'
            for match in re.finditer(param_pattern, response):
                pending.parameters.append(ParameterSuggestion(
                    param_name=match.group(1),
                    old_value=match.group(2),
                    new_value=match.group(3),
                    reason="Extracted from analysis",
                    confidence="medium"
                ))

        return pending


# ============================================================
# Parameter History Manager (DynamoDB)
# ============================================================
class ParameterHistoryManager:
    """DynamoDB에 파라미터 변경 이력 기록"""

    def __init__(self, clients: AWSClients, table_name: str = DYNAMODB_TABLE):
        self.clients = clients
        self.table_name = table_name
        self.table = None
        self.enabled = False

        try:
            self.table = clients.dynamodb.Table(table_name)
            # Test connection
            self.table.table_status
            self.enabled = True
            print(f"[DynamoDB] Connected: {table_name}")
        except Exception as e:
            print(f"[DynamoDB] Connection failed: {e}")

    def record_change(self, session_id: str, param_name: str, old_value: str,
                      new_value: str, reason: str = "", ai_suggested: bool = False) -> bool:
        """파라미터 변경 기록"""
        if not self.enabled:
            print("[DynamoDB] Not enabled, skipping record")
            return False

        try:
            timestamp = datetime.utcnow().isoformat() + "Z"
            ttl = int((datetime.utcnow() + timedelta(days=90)).timestamp())  # 90일 후 만료

            item = {
                'session_id': session_id,
                'timestamp': timestamp,
                'parameter_name': param_name,
                'old_value': str(old_value),
                'new_value': str(new_value),
                'reason': reason,
                'ai_suggested': ai_suggested,
                'ttl': ttl
            }

            self.table.put_item(Item=item)
            print(f"[DynamoDB] Recorded: {param_name} {old_value} -> {new_value}")
            return True

        except Exception as e:
            print(f"[DynamoDB] Record failed: {e}")
            return False

    def get_history(self, session_id: str = None, limit: int = 50) -> List[Dict]:
        """파라미터 변경 이력 조회"""
        if not self.enabled:
            return []

        try:
            if session_id:
                response = self.table.query(
                    KeyConditionExpression='session_id = :sid',
                    ExpressionAttributeValues={':sid': session_id},
                    ScanIndexForward=False,  # 최신순
                    Limit=limit
                )
            else:
                response = self.table.scan(Limit=limit)

            return response.get('Items', [])

        except Exception as e:
            print(f"[DynamoDB] Query failed: {e}")
            return []

    def get_parameter_history(self, param_name: str, limit: int = 20) -> List[Dict]:
        """특정 파라미터의 변경 이력 조회"""
        if not self.enabled:
            return []

        try:
            response = self.table.scan(
                FilterExpression='parameter_name = :pname',
                ExpressionAttributeValues={':pname': param_name},
                Limit=limit
            )

            items = response.get('Items', [])
            # timestamp로 정렬
            items.sort(key=lambda x: x['timestamp'], reverse=True)
            return items

        except Exception as e:
            print(f"[DynamoDB] Query failed: {e}")
            return []

    def get_recent_changes(self, hours: int = 24, limit: int = 50) -> List[Dict]:
        """최근 변경 이력 조회"""
        if not self.enabled:
            return []

        try:
            cutoff = (datetime.utcnow() - timedelta(hours=hours)).isoformat() + "Z"
            response = self.table.scan(
                FilterExpression='#ts > :cutoff',
                ExpressionAttributeNames={'#ts': 'timestamp'},
                ExpressionAttributeValues={':cutoff': cutoff},
                Limit=limit
            )

            items = response.get('Items', [])
            items.sort(key=lambda x: x['timestamp'], reverse=True)
            return items

        except Exception as e:
            print(f"[DynamoDB] Query failed: {e}")
            return []


# ============================================================
# Code Modifier
# ============================================================
class CodeModifier:
    """코드 자동 수정 도구"""

    def __init__(self, project_root: str):
        self.project_root = project_root
        self.backup_dir = os.path.join(project_root, 'backup', 'auto_reflect')
        os.makedirs(self.backup_dir, exist_ok=True)

    def read_file(self, filepath: str) -> str:
        """파일 읽기"""
        full_path = os.path.join(self.project_root, filepath)
        try:
            with open(full_path, 'r') as f:
                return f.read()
        except Exception as e:
            return f"Error reading file: {e}"

    def write_file(self, filepath: str, content: str, backup: bool = True) -> bool:
        """파일 쓰기 (백업 포함)"""
        full_path = os.path.join(self.project_root, filepath)
        try:
            # 백업 생성
            if backup and os.path.exists(full_path):
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                backup_filename = f"{os.path.basename(filepath)}.{timestamp}.bak"
                backup_path = os.path.join(self.backup_dir, backup_filename)
                with open(full_path, 'r') as f:
                    with open(backup_path, 'w') as b:
                        b.write(f.read())
                print(f"[Backup] Created: {backup_path}")

            # 파일 쓰기
            with open(full_path, 'w') as f:
                f.write(content)

            return True
        except Exception as e:
            print(f"Error writing file: {e}")
            return False

    def get_parameter_value(self, filepath: str, param_name: str) -> Optional[str]:
        """파라미터 현재 값 조회"""
        content = self.read_file(filepath)
        if content.startswith("Error"):
            return None

        pattern = rf"^{param_name}\s*=\s*(.+)$"
        match = re.search(pattern, content, flags=re.MULTILINE)
        if match:
            return match.group(1).strip()
        return None

    def update_parameter(self, filepath: str, param_name: str, new_value: str) -> tuple:
        """특정 파라미터 값 업데이트, (success, old_value) 반환"""
        content = self.read_file(filepath)
        if content.startswith("Error"):
            return (False, None)

        old_value = self.get_parameter_value(filepath, param_name)

        # 파이썬 변수 할당 패턴 매칭
        pattern = rf"^({param_name}\s*=\s*)(.+)$"
        new_content = re.sub(pattern, rf"\g<1>{new_value}", content, flags=re.MULTILINE)

        if new_content == content:
            print(f"Parameter {param_name} not found")
            return (False, old_value)

        success = self.write_file(filepath, new_content)
        return (success, old_value)

    def update_code_block(self, filepath: str, original: str, modified: str) -> bool:
        """코드 블록 교체"""
        content = self.read_file(filepath)
        if content.startswith("Error"):
            return False

        # 원본 코드 찾기 (공백 무시 버전)
        original_normalized = ' '.join(original.split())
        content_normalized = ' '.join(content.split())

        if original_normalized not in content_normalized:
            # 정확한 매칭 시도
            if original.strip() not in content:
                print(f"Original code block not found")
                return False

        new_content = content.replace(original.strip(), modified.strip())

        if new_content == content:
            print(f"Code block not changed")
            return False

        return self.write_file(filepath, new_content)

    def apply_all_suggestions(self, suggestions: PendingSuggestions,
                               param_history: 'ParameterHistoryManager',
                               session_id: str) -> Dict[str, Any]:
        """모든 제안사항 한번에 적용"""
        results = {
            'parameters_applied': [],
            'parameters_failed': [],
            'code_blocks_applied': [],
            'code_blocks_failed': [],
            'total_changes': 0
        }

        # 파라미터 변경 적용
        for param in suggestions.parameters:
            filepath = STANDALONE_CODE_PATH
            success, old_value = self.update_parameter(filepath, param.param_name, param.new_value)

            if success:
                results['parameters_applied'].append(param)
                results['total_changes'] += 1

                # DynamoDB에 기록
                param_history.record_change(
                    session_id=session_id,
                    param_name=param.param_name,
                    old_value=old_value or param.old_value,
                    new_value=param.new_value,
                    reason=param.reason,
                    ai_suggested=True
                )
                print(f"[Applied] {param.param_name}: {old_value} -> {param.new_value}")
            else:
                results['parameters_failed'].append(param)
                print(f"[Failed] {param.param_name}")

        # 코드 블록 변경 적용
        for block in suggestions.code_blocks:
            filepath = block.file_path if block.file_path else STANDALONE_CODE_PATH
            success = self.update_code_block(filepath, block.original_code, block.modified_code)

            if success:
                results['code_blocks_applied'].append(block)
                results['total_changes'] += 1

                # DynamoDB에 기록
                param_history.record_change(
                    session_id=session_id,
                    param_name=f"CODE_BLOCK:{block.description[:30]}",
                    old_value=block.original_code[:50] + "...",
                    new_value=block.modified_code[:50] + "...",
                    reason=block.reason,
                    ai_suggested=True
                )
                print(f"[Applied] Code block: {block.description}")
            else:
                results['code_blocks_failed'].append(block)
                print(f"[Failed] Code block: {block.description}")

        return results


# ============================================================
# Chat Interface
# ============================================================
class RobotDebugChat:
    """로봇 디버그 채팅 인터페이스"""

    def __init__(self, region: str = AWS_REGION):
        print("Initializing Robot Debug Chat...")

        self.clients = AWSClients(region)
        self.log_query = LogQueryEngine(
            self.clients,
            GLUE_DATABASE,
            ATHENA_WORKGROUP,
            S3_BUCKET
        )

        # Twelve Labs API 키 가져오기
        twelvelabs_key = TWELVELABS_API_KEY
        if not twelvelabs_key:
            secret = self.clients.get_secret('spot-robot-debug/twelvelabs-api-key')
            twelvelabs_key = secret.get('api_key', '')

        self.video_analyzer = VideoAnalyzer(twelvelabs_key)
        self.claude = ClaudeAnalyzer(self.clients)
        self.code_modifier = CodeModifier(os.path.dirname(os.path.dirname(__file__)))
        self.param_history = ParameterHistoryManager(self.clients, DYNAMODB_TABLE)

        self.current_session = None
        self.pending_suggestions = PendingSuggestions()  # AI 제안사항 저장
        self.last_response = ""  # 마지막 AI 응답 저장

        print("Robot Debug Chat initialized!")
        print("=" * 60)

    def set_session(self, session_id: str):
        """현재 분석 세션 설정"""
        self.current_session = session_id
        print(f"Session set: {session_id}")

    def list_sessions(self) -> List[Dict]:
        """최근 세션 목록"""
        return self.log_query.get_recent_sessions()

    def analyze_session(self, session_id: str = None) -> str:
        """세션 분석"""
        sid = session_id or self.current_session
        if not sid:
            return "No session specified. Use set_session() or provide session_id."

        # 로그 조회
        logs = self.log_query.get_session_logs(sid)
        falls = self.log_query.get_fall_events(sid)

        # 분석 요청
        analysis = self.claude.analyze(
            f"Analyze session {sid} and provide a summary of robot performance.",
            {'logs': logs, 'session_id': sid}
        )

        return analysis

    def analyze_fall(self, session_id: str = None, step: int = None) -> str:
        """넘어짐 분석"""
        sid = session_id or self.current_session
        if not sid:
            return "No session specified."

        # 넘어진 이벤트 찾기
        if step is None:
            falls = self.log_query.get_fall_events(sid)
            if not falls:
                return "No fall events found in this session."
            step = int(falls[0]['step'])

        # 해당 시점 전후 로그
        logs = self.log_query.get_logs_around_time(sid, step)

        # 분석
        return self.claude.analyze_fall_cause(logs)

    def suggest_fix(self, issue: str) -> str:
        """수정 제안"""
        # 현재 코드 읽기
        current_code = self.code_modifier.read_file('src/spot_rl_standalone.py')

        return self.claude.suggest_code_fix(issue, current_code)

    def apply_fix(self, param: str, value: str, reason: str = "", ai_suggested: bool = False) -> str:
        """파라미터 수정 적용 및 DynamoDB에 이력 기록"""
        # 현재 값 읽기
        import re
        current_code = self.code_modifier.read_file('src/spot_rl_standalone.py')
        old_value = "unknown"

        if not current_code.startswith("Error"):
            pattern = rf"^{param}\s*=\s*(.+)$"
            match = re.search(pattern, current_code, flags=re.MULTILINE)
            if match:
                old_value = match.group(1).strip()

        # 파라미터 업데이트
        success = self.code_modifier.update_parameter(
            'src/spot_rl_standalone.py',
            param,
            value
        )

        if success:
            # DynamoDB에 변경 이력 기록
            session_id = self.current_session or f"manual_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            self.param_history.record_change(
                session_id=session_id,
                param_name=param,
                old_value=old_value,
                new_value=value,
                reason=reason,
                ai_suggested=ai_suggested
            )
            return f"Parameter {param} updated: {old_value} -> {value} (recorded to DynamoDB)"
        else:
            return f"Failed to update {param}"

    def get_param_history(self, param_name: str = None) -> List[Dict]:
        """파라미터 변경 이력 조회"""
        if param_name:
            return self.param_history.get_parameter_history(param_name)
        else:
            return self.param_history.get_recent_changes()

    def chat(self, message: str, auto_parse: bool = True) -> str:
        """자연어 채팅"""
        # 컨텍스트 구성
        context = {}
        if self.current_session:
            context['session_id'] = self.current_session
            context['logs'] = self.log_query.get_session_logs(self.current_session, limit=20)

        response = self.claude.analyze(message, context)
        self.last_response = response

        # AI 응답에서 제안사항 자동 파싱
        if auto_parse:
            new_suggestions = self.claude.parse_suggestions(response)
            if new_suggestions.has_suggestions():
                self.pending_suggestions = new_suggestions
                print(f"\n[AI] {len(new_suggestions.parameters)}개 파라미터, "
                      f"{len(new_suggestions.code_blocks)}개 코드블록 제안 감지됨")
                print("     '/reflect' 또는 '반영해줘'로 적용 가능\n")

        return response

    def reflect_suggestions(self, confirm: bool = True) -> str:
        """대기 중인 AI 제안사항을 실제 코드에 반영"""
        if not self.pending_suggestions.has_suggestions():
            return "반영할 제안 사항이 없습니다. 먼저 분석을 요청하세요."

        # 대기 중인 제안 표시
        print("\n" + "=" * 60)
        print("📋 적용할 제안 사항:")
        print("=" * 60)
        print(str(self.pending_suggestions))
        print("=" * 60)

        if confirm:
            user_confirm = input("\n위 변경사항을 적용하시겠습니까? (y/n): ").strip().lower()
            if user_confirm not in ['y', 'yes', '예', 'ㅇ']:
                self.pending_suggestions.clear()
                return "변경사항 적용이 취소되었습니다."

        # 세션 ID 설정
        session_id = self.current_session or f"reflect_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # 모든 제안사항 적용
        print("\n🔧 변경사항 적용 중...")
        results = self.code_modifier.apply_all_suggestions(
            self.pending_suggestions,
            self.param_history,
            session_id
        )

        # 결과 메시지 생성
        msg_lines = ["\n✅ 반영 완료!"]
        msg_lines.append(f"   총 {results['total_changes']}개 변경 적용됨")

        if results['parameters_applied']:
            msg_lines.append(f"\n   파라미터 변경:")
            for p in results['parameters_applied']:
                msg_lines.append(f"     - {p.param_name}: {p.old_value} -> {p.new_value}")

        if results['code_blocks_applied']:
            msg_lines.append(f"\n   코드 블록 변경:")
            for c in results['code_blocks_applied']:
                msg_lines.append(f"     - {c.description}")

        if results['parameters_failed'] or results['code_blocks_failed']:
            msg_lines.append(f"\n   ⚠️ 실패한 항목:")
            for p in results['parameters_failed']:
                msg_lines.append(f"     - {p.param_name} (파라미터)")
            for c in results['code_blocks_failed']:
                msg_lines.append(f"     - {c.description} (코드블록)")

        msg_lines.append(f"\n   백업 위치: {self.code_modifier.backup_dir}")
        msg_lines.append("   변경 이력: DynamoDB에 기록됨 (/history로 조회)")

        # 제안사항 초기화
        self.pending_suggestions.clear()

        return "\n".join(msg_lines)

    def show_pending(self) -> str:
        """대기 중인 제안사항 표시"""
        if not self.pending_suggestions.has_suggestions():
            return "대기 중인 제안 사항이 없습니다."
        return str(self.pending_suggestions)

    def interactive_mode(self):
        """대화형 모드"""
        print("\nRobot Debug Chat - Interactive Mode")
        print("=" * 60)
        print("Commands:")
        print("  /sessions       - List recent sessions")
        print("  /session <id>   - Set current session")
        print("  /analyze        - Analyze current session")
        print("  /falls          - Show fall events")
        print("  /fix <issue>    - Get fix suggestion")
        print("  /apply <p> <v>  - Apply parameter change")
        print("  /history [p]    - Show parameter change history")
        print("  /pending        - Show pending suggestions")
        print("  /reflect        - Apply AI suggestions to code (반영해줘)")
        print("  /quit           - Exit")
        print("=" * 60)
        print("\n한국어 명령어: '반영해줘', '적용해줘', '분석해줘'")
        print("Or just type your question in natural language.\n")

        while True:
            try:
                user_input = input("You: ").strip()

                if not user_input:
                    continue

                # 한국어 명령어 처리
                korean_reflect = ['반영해줘', '적용해줘', '반영', '적용해', '코드수정해줘', '수정해줘']
                korean_analyze = ['분석해줘', '분석해', '분석']

                if any(cmd in user_input for cmd in korean_reflect):
                    print(self.reflect_suggestions())
                    continue
                elif user_input in korean_analyze:
                    print("\n세션 분석 중...")
                    print(self.analyze_session())
                    continue

                if user_input.startswith('/'):
                    parts = user_input.split(maxsplit=2)
                    cmd = parts[0].lower()

                    if cmd == '/quit':
                        print("Goodbye!")
                        break
                    elif cmd == '/sessions':
                        sessions = self.list_sessions()
                        print("\nRecent Sessions:")
                        for s in sessions:
                            print(f"  {s['session_id']}: {s.get('total_steps', 'N/A')} steps, "
                                  f"{s.get('fall_count', 0)} falls")
                    elif cmd == '/session' and len(parts) > 1:
                        self.set_session(parts[1])
                    elif cmd == '/analyze':
                        print("\nAnalyzing session...")
                        response = self.analyze_session()
                        print(response)
                    elif cmd == '/falls':
                        falls = self.log_query.get_fall_events(self.current_session)
                        print(f"\nFall events ({len(falls)}):")
                        for f in falls[:10]:
                            print(f"  Step {f['step']}: height={f['height']}, "
                                  f"action_norm={f['action_norm']}")
                    elif cmd == '/fix' and len(parts) > 1:
                        print("\nGenerating fix suggestion...")
                        response = self.suggest_fix(parts[1])
                        self.last_response = response
                        # 제안사항 파싱
                        new_suggestions = self.claude.parse_suggestions(response)
                        if new_suggestions.has_suggestions():
                            self.pending_suggestions = new_suggestions
                        print(response)
                    elif cmd == '/apply' and len(parts) > 2:
                        result = self.apply_fix(parts[1], parts[2])
                        print(result)
                    elif cmd == '/history':
                        param_name = parts[1] if len(parts) > 1 else None
                        history = self.get_param_history(param_name)
                        if history:
                            print(f"\nParameter Change History ({len(history)} records):")
                            for h in history[:20]:
                                ai_tag = "[AI]" if h.get('ai_suggested') else ""
                                print(f"  {h['timestamp'][:19]} | {h['parameter_name']}: "
                                      f"{h['old_value']} -> {h['new_value']} {ai_tag}")
                                if h.get('reason'):
                                    print(f"    Reason: {h['reason']}")
                        else:
                            print("No parameter change history found.")
                    elif cmd == '/pending':
                        print("\n" + self.show_pending())
                    elif cmd == '/reflect':
                        print(self.reflect_suggestions())
                    else:
                        print("Unknown command. Type /quit to exit.")
                else:
                    # 자연어 질문
                    print("\nAnalyzing...")
                    response = self.chat(user_input)
                    print(f"\nClaude: {response}\n")

            except KeyboardInterrupt:
                print("\nGoodbye!")
                break
            except Exception as e:
                print(f"Error: {e}")


# ============================================================
# Main
# ============================================================
def main():
    parser = argparse.ArgumentParser(description='Robot Debug Chat Interface')
    parser.add_argument('--session', '-s', help='Session ID to analyze')
    parser.add_argument('--analyze', '-a', action='store_true', help='Analyze session')
    parser.add_argument('--falls', '-f', action='store_true', help='Show fall events')
    parser.add_argument('--interactive', '-i', action='store_true', help='Interactive mode')

    args = parser.parse_args()

    chat = RobotDebugChat()

    if args.session:
        chat.set_session(args.session)

    if args.analyze:
        print(chat.analyze_session())
    elif args.falls:
        falls = chat.log_query.get_fall_events(args.session)
        for f in falls:
            print(f"Step {f['step']}: height={f['height']}")
    elif args.interactive or not any([args.analyze, args.falls]):
        chat.interactive_mode()


if __name__ == "__main__":
    main()
