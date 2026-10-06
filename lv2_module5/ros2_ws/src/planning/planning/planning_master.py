from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float32MultiArray, String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from collections import deque
from planning.health_monitor import SENSORS, HealthMonitor

# 상태: idle / tracking / searching / lost / fault
#   lost  = 목표 소실 (720° 탐색 실패) → 팔 nominal 복귀 → idle
#   fault = 입력·센서·모터 장애 → 속도 0, 팔 [0, 0] 명령 → 장애 해소 시 idle
# 입력 약속 (/detection z): 0 = 미검출, > 0 = 검출 depth [m]. depth 실패(NaN)는 인지단에서 0으로 보낸다.
# 파라미터는 config/planning.yaml에서 덮어쓴다. 아래 기본값은 yaml이 없을 때 사용.


class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')

        def param(name, default):
            return self.declare_parameter(name, default).value

        # ---------------- 토픽 ----------------
        detection_topic = param('detection_topic', '/detection')
        imu_topic = param('imu_topic', '/control/imu')
        odom_yaw_topic = param('odom_yaw_topic', '/control/odom_yaw_deg')
        joint_states_topic = param('joint_states_topic', '/control/joint_states')
        cmd_vel_topic = param('cmd_vel_topic', '/planning/cmd_vel')
        arm_cmd_topic = param('arm_cmd_topic', '/planning/arm_command')
        status_topic = param('status_topic', '/tracking_status')
        # JointState.name에서 찾을 관절 이름 (control: wheel_left/right, arm_yaw, arm_pitch)
        self.pan_joint_name = param('pan_joint_name', 'arm_yaw_joint')
        self.tilt_joint_name = param('tilt_joint_name', 'arm_pitch_joint')

        # ---------------- 주기 · 타임아웃 ----------------
        self.control_rate_hz = float(param('control_rate_hz', 30.0))  # 제어 루프 주기
        self.detection_timeout_s = float(param('detection_timeout_s', 0.5))  # /detection 침묵 → 안전 정지
        self.imu_timeout_s = float(param('imu_timeout_s', 0.5))  # IMU 침묵 → 엔코더 heading 사용, 둘 다 없으면 정지
        self.odom_timeout_s = float(param('odom_timeout_s', 0.5))
        self.joint_timeout_s = float(param('joint_timeout_s', 0.5))  # 관절 피드백 침묵 → 무효 처리
        self.recover_frames = int(param('recover_frames', 3))  # 연속 검출 프레임 수 → tracking 진입

        # ---------------- 카메라 · 팔 ----------------
        self.nominal_pose = list(param('nominal_pose', [0.0, 0.0]))  # 정면 수평 명령. 실측 [-0.09, 0.00] deg, raw [2047, 2048]
        self.hfov_deg = float(param('hfov_deg', 69.0))  # 수평 화각(deg)
        self.vfov_deg = float(param('vfov_deg', 42.0))  # 수직 화각(deg)
        self.pan_forward_deg = float(param('pan_forward_deg', -0.09))  # 정면 수평에서 측정한 pan 각도(deg), 명령 0과 구분
        # pan/tilt 축과 광학 중심이 일치한다고 근사한다. 기준점 위치는 실측값으로 설정.
        self.camera_origin_xyz = tuple(param('camera_origin_xyz', [0.0, 0.0, 0.0]))  # 로봇 기준 m, 전방/좌측/위쪽 (pan/tilt 축 = 로봇 원점으로 근사)
        # 실험으로 확인한 관절 한계. 모터 피드백과 같은 좌표계의 deg.
        self.pan_deg_limits = tuple(param('pan_deg_limits', [-120.0, 120.0]))  # (하한, 상한), deg
        self.tilt_deg_limits = tuple(param('tilt_deg_limits', [-80.0, 85.0]))  # (하한, 상한), deg
        # 젖힘 제한: tilt = 90 - VFOV/2 이면 화면 위쪽 끝이 이미 수직 위를 본다. 그 이상은 필요 없음.
        self.tilt_up_max_deg = 90.0 - self.vfov_deg / 2.0  # VFOV 42° → 69°
        self.tilt_deg_limits = (
            self.tilt_deg_limits[0],
            min(self.tilt_deg_limits[1], self.tilt_up_max_deg),
        )
        self.nominal_tolerance_deg = float(param('nominal_tolerance_deg', 1.0))  # lost → idle 복귀 판정

        # ---------------- 차체 ----------------
        self.waffle_max_angular_vel = float(param('waffle_max_angular_vel', 1.665))  # rad/s, 바닥/엔코더 실측 최대값
        # 이론 최대 1.794 rad/s (휠 7.8 rad/s). 요청 1.0 → 실측 0.995 rad/s.
        self.waffle_yaw_gain = float(param('waffle_yaw_gain', 0.8))  # 1/s, 분배된 회전각(rad)을 각속도로 변환
        self.pan_yaw_weight = float(param('pan_yaw_weight', 0.5))  # 0~1, pan에 분배할 비율
        if not 0.0 <= self.pan_yaw_weight <= 1.0:
            raise ValueError('pan_yaw_weight must be between 0 and 1')
        self.waffle_yaw_weight = 1.0 - self.pan_yaw_weight  # 0~1, waffle에 분배할 비율
        self.tgt_dis = float(param('tgt_dis', 0.40))  # m, 목표 추종 거리
        self.dis_gain = float(param('dis_gain', 1.0))  # 거리 P제어 gain (1/s), 추후 튜닝
        self.dis_error_threshold = float(param('dis_error_threshold', 0.02))  # m, 목표 거리 정지 허용 오차(임시값)
        self.waffle_linear_vel_limits = tuple(param('waffle_linear_vel_limits', [-0.1, 0.2]))  # m/s, (후진, 전진)
        self.yaw_error_threshold_deg = float(param('yaw_error_threshold_deg', 3.0))  # 차량 정렬 허용 오차
        self.linear_heading_limit_deg = float(param('linear_heading_limit_deg', 60.0))  # 이보다 옆/뒤면 회전 우선
        # wheel align P 게인 [1/s] = 최대 각속도 × 비율 (1 rad 오차 → 0.5·ω_max)
        self.wheel_align_gain_ratio = float(param('wheel_align_gain_ratio', 0.5))
        self.wheel_align_gain = self.wheel_align_gain_ratio * self.waffle_max_angular_vel
        self.search_angular_vel = float(param('search_angular_vel', 0.8))  # rad/s, 720° 탐색 ≈ 15.7 s
        # 탐색 바퀴별 tilt [deg]: 한 바퀴마다 다음 높이로 고개를 바꿔 돈다. 바퀴 수 = 목록 길이 → 다 돌면 lost
        #   목록 항목: 숫자 문자열 = 그 각도, 'nominal' = nominal_pose tilt,
        #              'max' = tilt 상한(min(관절 한계, 90 - VFOV/2)), 'mid' = 첫 항목과 마지막 항목의 중간
        #   기본 ['0', 'mid', 'max'] = [0, 34.5, 69] → 보는 범위 -21~21 / 13.5~55.5 / 48~90 (7.5° 씩 겹침)
        #   ['stack'] = 상한 아래로 VFOV 씩 쌓기: tilt_k = 상한 - (k+1)·VFOV + k·overlap (search_turns 바퀴)
        self.search_tilt_levels_param = [str(v) for v in param('search_tilt_levels', ['0', 'mid', 'max'])]
        self.search_turns = int(param('search_turns', 2))                              # stack 바퀴 수
        self.search_tilt_overlap_deg = float(param('search_tilt_overlap_deg', 5.0))  # 바퀴 사이 화면 겹침
        # 마지막 검출 위치 판정 경계 (정규화 e_x). e_x >= 0.5 ↔ 화면 오른쪽 1/4
        self.lost_side_ex_threshold = float(param('lost_side_ex_threshold', 0.5))
        self.search_tilt_levels = []
        if self.search_tilt_levels_param == ['stack']:
            top = self.tilt_deg_limits[1]
            levels = [top - (k + 1) * self.vfov_deg + k * self.search_tilt_overlap_deg
                      for k in range(max(1, self.search_turns))]
            self.search_tilt_levels = sorted(self._clip(v, self.tilt_deg_limits) for v in levels)
        def resolve(v):
            if v == 'nominal':
                return self.nominal_pose[1]
            if v == 'max':
                return self.tilt_deg_limits[1]
            return float(v)
        items = [] if self.search_tilt_levels_param == ['stack'] else self.search_tilt_levels_param
        ends = [resolve(v) for v in items if v != 'mid']
        for v in items:
            deg = (ends[0] + ends[-1]) / 2.0 if v == 'mid' else resolve(v)
            self.search_tilt_levels.append(self._clip(deg, self.tilt_deg_limits))
        if not self.search_tilt_levels:
            raise ValueError('search_tilt_levels must not be empty')
        self.search_max_turns = len(self.search_tilt_levels)  # 누적 회전 바퀴 수 → lost
        # fault 상태에서 발행할 팔 각도 [pan, tilt] deg
        self.fault_arm_pose = [float(v) for v in param('fault_arm_pose', [0.0, 0.0])]
        # FAULT 해소 직후 이 시간(초) 동안 status reason 을 recovered_from:<원인> 으로 표시
        self.recovery_reason_hold_s = float(param('recovery_reason_hold_s', 2.0))
        # 영상 촬영 시각의 팔 자세로 물체 위치를 복원 (영상·관절 지연 차이로 생기는 tilt/pan 진동 방지)
        self.sync_arm_to_image = bool(param('sync_arm_to_image', True))
        self.arm_pose_hist = deque(maxlen=200)   # (stamp_s, pan_deg, tilt_deg)
        self.cur_cam_stamp_s = None

        # ---------------- 퍼블리셔 · 구독자 ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, arm_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        # perception_master 발행 QoS와 동일: best-effort, depth 1
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cam_sub = self.create_subscription(PointStamped, detection_topic, self.cam_cb, detection_qos)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 10)
        self.odom_yaw_sub = self.create_subscription(Float32, odom_yaw_topic, self.odom_yaw_cb, detection_qos)
        self.motor_sub = self.create_subscription(JointState, joint_states_topic, self.motor_cb, 10)
        # HealthGate의 별도 상태 발행 대신 기존 /tracking_status에 판정 사유를 통합한다.
        self.health_enabled = bool(param('health.enabled', True))
        self.health_monitor = HealthMonitor(self) if self.health_enabled else None
        self.health = None
        self.health_reason = 'initializing'
        self.health_blocked = False
        self.last_fault_reason = None   # 가장 최근 FAULT 원인
        self.recovered_from = None      # 표시용: 방금 복구된 FAULT 원인
        self.recovered_until = 0.0
        self.arm_publish_enabled = True

        # ---------------- 상태 변수 ----------------
        self.tgt_arm_pose = self.nominal_pose.copy()
        self.cur_arm_pose = self.nominal_pose.copy()  # 초기값, motor_cb에서 실제 절대각 수신
        self.cur_arm_pose_valid = False
        self.cur_object_xyz = None  # 로봇 기준 (x 전방, y 좌측, z 위쪽), m
        self.cur_object_planar_dis = None  # 로봇 기준 수평 거리, m
        self.cur_object_yaw_deg = None  # 로봇 전방 기준 방위각, 반시계+
        self.tgt_yaw_rot_deg = 0.0  # deg, 현재 방향 대비 보정량
        self.tgt_pan_deg = self.nominal_pose[0]  # deg, 분배 및 관절 한계를 적용한 절대 목표각
        self.tgt_tilt_deg = self.nominal_pose[1]  # deg, 관절 한계를 적용한 절대 목표각
        self.tgt_pan_rot_deg = 0.0  # deg, pan 기본 분배량
        self.tgt_waffle_rot_deg = 0.0  # deg, waffle 기본 분배량 + pan residual
        self.pan_residual = 0.0  # deg, pan 한계로 처리하지 못한 회전량
        self.tgt_waffle_angular_vel = 0.0  # rad/s, 분배 및 속도 제한을 적용한 각속도
        self.tgt_waffle_linear_vel = 0.0  # m/s, 계산 및 제한을 적용한 종속도
        self.cur_camera_object_yaw_deg = 0.0
        self.cur_camera_object_tilt_deg = 0.0

        self.cur_bbox_error_x = 0.0
        self.cur_bbox_error_y = 0.0
        self.cur_depth = 0.0  # 미검출
        self.cur_cam_timestamp = None
        self.detect_streak = 0  # 연속 검출(depth > 0) 프레임 수
        self.last_detected_ex = 0.0  # 마지막으로 검출된 프레임의 e_x
        self.lost_side = 'middle'  # searching 진입 시점의 마지막 검출 위치

        self.cur_yaw_deg = 0.0
        self.cur_imu_yaw_deg = 0.0
        self.cur_encoder_yaw_deg = 0.0
        self.heading_source = 'none'
        self.heading_stale = True
        self.search_cnt = 0
        self.search_rot_deg = 0.0
        self.prev_yaw_deg = self.cur_yaw_deg

        # 마지막 수신 시각 (None = 아직 수신 안 함)
        self.last_cam_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        self.last_joint_time = None
        self.detection_stale = True
        self.imu_stale = True
        self.odom_stale = True
        self.camera_input_valid = False

        self.cmd_vel_msg = Twist()
        self.state = 'idle'

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.run)

    # ================= 콜백 =================
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def imu_cb(self, msg):  # sensor_msgs/Imu
        """orientation 쿼터니언에서 yaw(deg, 반시계+, -180~180)를 구한다."""
        q = msg.orientation
        if msg.orientation_covariance[0] == -1 or not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 0.0):
            self.last_imu_time = None
            return
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cur_imu_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.last_imu_time = self._now_s()

    def odom_yaw_cb(self, msg):
        """control이 계산한 엔코더 yaw(deg). IMU 장애/침묵 시 대체 입력."""
        if not math.isfinite(msg.data):
            self.last_odom_time = None
            return
        self.cur_encoder_yaw_deg = msg.data
        self.last_odom_time = self._now_s()

    def cam_cb(self, msg):  # PointStamped
        now = self._now_s()
        if self.last_cam_time is None or not 0 <= now - self.last_cam_time <= self.detection_timeout_s:
            self.detect_streak = 0
        self.cur_bbox_error_x = msg.point.x
        self.cur_bbox_error_y = msg.point.y
        self.cur_depth = msg.point.z
        self.cur_cam_stamp_s = self._stamp_s(msg.header.stamp)
        self.cur_cam_timestamp = msg.header.stamp
        self.last_cam_time = now
        self.camera_input_valid = all(math.isfinite(v) for v in (
            self.cur_bbox_error_x, self.cur_bbox_error_y, self.cur_depth,
        )) and self.cur_depth >= 0

        if self.camera_input_valid and self.cur_depth > 0.0:  # 검출: 화면상 위치 저장 (lost_pose용)
            if math.isfinite(self.cur_bbox_error_x):
                self.last_detected_ex = self.cur_bbox_error_x
        # 유효한 depth > 0 프레임만 연속 검출로 인정. NaN 등은 check_health에서 입력 오류로 처리.
        if self.camera_input_valid and self.cur_depth > 0.0:
            self.detect_streak = min(self.detect_streak + 1, self.recover_frames)
        else:
            self.detect_streak = 0

    def motor_cb(self, msg):  # sensor_msgs/JointState, position [rad]
        names = list(msg.name)
        if self.pan_joint_name not in names or self.tilt_joint_name not in names:
            return  # 팔 관절이 없는 메시지는 무시 (갱신 없으면 joint_timeout 후 무효 처리)
        idx = (names.index(self.pan_joint_name), names.index(self.tilt_joint_name))
        self.cur_arm_pose_valid = len(msg.position) > max(idx) and all(
            math.isfinite(msg.position[i]) for i in idx
        )
        if self.cur_arm_pose_valid:
            self.cur_arm_pose = [math.degrees(msg.position[i]) for i in idx]  # [pan deg, tilt deg]
            self.last_joint_time = self._now_s()
            t = self._stamp_s(msg.header.stamp)
            if t is not None:
                if self.arm_pose_hist and t < self.arm_pose_hist[-1][0]:
                    self.arm_pose_hist.clear()      # 시계가 뒤로 감 (bag 재생 등)
                self.arm_pose_hist.append((t, self.cur_arm_pose[0], self.cur_arm_pose[1]))

    @staticmethod
    def _stamp_s(stamp):
        """builtin_interfaces/Time → 초. 비어 있으면(0) None."""
        if stamp is None:
            return None
        t = getattr(stamp, 'sec', 0) + getattr(stamp, 'nanosec', 0) * 1e-9
        return t if t > 0 else None

    def arm_pose_at(self, t):
        """시각 t 의 팔 [pan, tilt] (관절 기록 선형 보간). 기록이 없거나 t 를 모르면 최신 측정값."""
        h = self.arm_pose_hist
        if not self.sync_arm_to_image or t is None or not h:
            return list(self.cur_arm_pose)
        if t >= h[-1][0]:
            return [h[-1][1], h[-1][2]]
        if t <= h[0][0]:
            return [h[0][1], h[0][2]]
        for (t0, p0, q0), (t1, p1, q1) in zip(reversed(list(h)[:-1]), reversed(list(h)[1:])):
            if t0 <= t <= t1:
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return [p0 + a * (p1 - p0), q0 + a * (q1 - q0)]
        return list(self.cur_arm_pose)

    def check_timeouts(self):
        """입력별 마지막 수신 시각으로 침묵 여부를 갱신한다."""
        now = self._now_s()

        def stale(last, timeout):
            return last is None or not 0 <= now - last <= timeout

        self.detection_stale = stale(self.last_cam_time, self.detection_timeout_s)
        self.imu_stale = stale(self.last_imu_time, self.imu_timeout_s)
        self.odom_stale = stale(self.last_odom_time, self.odom_timeout_s)
        if stale(self.last_joint_time, self.joint_timeout_s):
            self.cur_arm_pose_valid = False
        if self.detection_stale:
            self.detect_streak = 0

    def check_health(self):
        """진단 메시지 + 실제 데이터 신선도로 정지/heading fallback을 결정한다."""
        self.health = self.health_monitor.evaluate() if self.health_monitor else None
        h = self.health
        imu_ok = not self.imu_stale and (h is None or h.imu_ok)
        encoder_ok = not self.odom_stale and (h is None or (h.mcu_ok and h.motor_ok))
        source = 'imu' if imu_ok else ('encoder' if encoder_ok else 'none')
        self.heading_stale = source == 'none'
        if source != 'none':
            yaw = self.cur_imu_yaw_deg if source == 'imu' else self.cur_encoder_yaw_deg
            if source != self.heading_source:
                # IMU/엔코더의 영점 차이를 탐색 회전량에 더하지 않는다.
                self.prev_yaw_deg = yaw
            self.cur_yaw_deg = yaw
        self.heading_source = source

        reason = None
        if h is not None and h.reason in ('mcu_fault', 'motor_fault', 'camera_stale'):
            reason = h.reason
        elif self.detection_stale:
            reason = 'detection_timeout'
        elif not self.camera_input_valid:
            reason = 'camera_invalid'
        elif not self.cur_arm_pose_valid:
            reason = 'arm_invalid'
        elif self.heading_stale:
            reason = 'heading_unavailable'

        self.health_blocked = reason is not None
        self.health_reason = reason or ('imu_fallback' if source == 'encoder' else (
            'ok' if self.health_enabled else 'health_disabled'))
        self.arm_publish_enabled = True
        if self.health_blocked:
            # 장애(모터·팔·센서·입력 무관): 속도 0, 팔 각도 fault_arm_pose([0, 0]) 발행
            self.state = 'fault'
            self.last_fault_reason = reason
            self.recovered_from = None
            self.detect_streak = 0
            self._stop_vehicle()
            self.tgt_waffle_linear_vel = self.tgt_waffle_angular_vel = 0.0
            self.tgt_pan_deg, self.tgt_tilt_deg = self.fault_arm_pose
            self.tgt_arm_pose = list(self.fault_arm_pose)

    # ================= 계산 =================
    @staticmethod
    def _matmul(a, b):
        """행렬 곱. 입력은 행 단위 리스트, 벡터는 열벡터로 전달한다."""
        columns = list(zip(*b))
        return [[sum(x * y for x, y in zip(row, col)) for col in columns] for row in a]

    def calc_cur_object_pos(self):
        """광축 depth와 bbox 오차를 복원하고 tilt/pan을 적용해 상대 위치를 저장한다."""
        self.cur_object_xyz = None
        self.cur_object_planar_dis = None
        self.cur_object_yaw_deg = None
        if not self.cur_arm_pose_valid or not all(math.isfinite(v) for v in (
            self.cur_depth, self.cur_bbox_error_x, self.cur_bbox_error_y,
        )) or self.cur_depth <= 0:
            return

        # 인지 오차는 오른쪽/아래쪽이 +인 정규화 값.
        # -(u-W/2)/(W/2) * HFOV/2 == (W/2-u)/W * HFOV (선형 화각 근사).
        bbox_yaw_rad = math.radians(-self.cur_bbox_error_x * self.hfov_deg / 2)
        bbox_tilt_rad = math.radians(-self.cur_bbox_error_y * self.vfov_deg / 2)
        cam_forward = self.cur_depth
        cam_left = self.cur_depth * math.tan(bbox_yaw_rad)
        cam_up = self.cur_depth * math.tan(bbox_tilt_rad)

        img_pan, img_tilt = self.arm_pose_at(self.cur_cam_stamp_s)   # 촬영 시각의 팔 자세
        pan_rad = math.radians(img_pan - self.pan_forward_deg)
        tilt_rad = math.radians(img_tilt)  # 실측: 수평 0°, 위쪽+
        cp, sp = math.cos(pan_rad), math.sin(pan_rad)
        ct, st = math.cos(tilt_rad), math.sin(tilt_rad)
        ox, oy, oz = self.camera_origin_xyz

        # 전방/좌측/위쪽 좌표. tilt 위쪽+이므로 Ry(-tilt)를 사용한다.
        t_tilt = [
            [ct, 0, -st, 0],
            [0,  1,   0, 0],
            [st, 0,  ct, 0],
            [0,  0,   0, 1],
        ]
        # Rz(pan)과 로봇 기준 카메라 원점 이동을 결합한다.
        t_pan_origin = [
            [cp, -sp, 0, ox],
            [sp,  cp, 0, oy],
            [0,    0, 1, oz],
            [0,    0, 0,  1],
        ]
        # T_robot_camera = Translation @ Rz(pan) @ Ry(-tilt)
        t_robot_camera = self._matmul(t_pan_origin, t_tilt)
        pos_camera = [[cam_forward], [cam_left], [cam_up], [1.0]]
        pos_robot = self._matmul(t_robot_camera, pos_camera)

        x, y, z = (row[0] for row in pos_robot[:3])
        self.cur_object_xyz = (x, y, z)
        self.cur_object_planar_dis = math.hypot(x, y)
        self.cur_object_yaw_deg = math.degrees(math.atan2(y, x))

    def lost_pose(self):
        """마지막 검출 프레임의 e_x로 사라진 위치를 판정한다.

        e_x = (c_x - W/2)/(W/2), 오른쪽 +. 화면 1/4 경계 ↔ |e_x| = 0.5.
        """
        ex = self.last_detected_ex
        if ex >= self.lost_side_ex_threshold:
            return 'right'
        if ex <= -self.lost_side_ex_threshold:
            return 'left'
        return 'middle'

    @staticmethod
    def _clip(value, limits):
        """값을 (하한, 상한) 튜플 범위로 제한한다."""
        lower, upper = limits
        return max(lower, min(upper, value))

    def calc_tgt_pan_deg(self):
        """현재 pan 절대각에 분배된 보정각을 더하고 관절 한계를 적용한다."""
        self.tgt_pan_deg = self._clip(self.cur_arm_pose[0] + self.tgt_pan_rot_deg, self.pan_deg_limits)

    @staticmethod
    def _wrap_deg(deg):
        return (deg + 180.0) % 360.0 - 180.0

    def calc_tracking_targets(self, object_xyz, vehicle_yaw_deg, pan_deg, tilt_deg):
        """차량 기준 물체 위치(m)와 측정각(deg)에서 목표 속도/관절각을 저장한다.

        object_xyz: x 전방, y 좌측, z 위쪽. 속도 단위는 m/s와 rad/s.
        차량 yaw는 월드 기준이며 상대위치에 다시 회전 적용하지 않는다.
        """
        self.tgt_waffle_linear_vel = 0.0
        self.tgt_waffle_angular_vel = 0.0
        if object_xyz is None or len(object_xyz) != 3 or not all(
            math.isfinite(v) for v in (*object_xyz, vehicle_yaw_deg, pan_deg, tilt_deg)
        ):
            return
        self.cur_object_xyz = tuple(object_xyz)
        self.cur_yaw_deg = vehicle_yaw_deg
        self.cur_arm_pose = [pan_deg, tilt_deg]
        x, y, z = object_xyz
        self.cur_object_planar_dis = math.hypot(x, y)
        if self.cur_object_planar_dis < 1e-9:
            return  # 차량 바로 위/아래에서는 수평 방향을 정의할 수 없다.
        self.cur_object_yaw_deg = math.degrees(math.atan2(y, x))
        ox, oy, oz = self.camera_origin_xyz
        cx, cy, cz = x - ox, y - oy, z - oz
        self.cur_camera_object_yaw_deg = math.degrees(math.atan2(cy, cx))
        self.cur_camera_object_tilt_deg = math.degrees(math.atan2(cz, math.hypot(cx, cy)))

        self.calc_tgt_yaw_deg()
        self.distribute_tgt_yaw_deg()
        self.calc_tgt_tilt_deg()
        self.calc_waffle_linear_vel()
        self.calc_waffle_angular_vel()
        # 목표 거리 도달 후 방위 오차가 남으면 차체만으로 정렬
        if (
            abs(self.cur_object_planar_dis - self.tgt_dis) <= self.dis_error_threshold
            and abs(self.tgt_yaw_rot_deg) > self.yaw_error_threshold_deg
        ):
            self.wheel_align()
        self.tgt_arm_pose = [self.tgt_pan_deg, self.tgt_tilt_deg]

    def calc_tgt_yaw_deg(self):
        """차량 기준 상대 방위각을 최단 회전각으로 사용한다."""
        self.tgt_yaw_rot_deg = self._wrap_deg(self.cur_object_yaw_deg)

    def distribute_tgt_yaw_deg(self):
        """차량 회전 몫을 먼저 정하고 남은 시선 방향을 pan에 할당한다."""
        self.tgt_waffle_rot_deg = self.tgt_yaw_rot_deg * self.waffle_yaw_weight
        desired_pan_deg = self._wrap_deg(
            self.cur_camera_object_yaw_deg - self.tgt_waffle_rot_deg
        ) + self.pan_forward_deg
        self.tgt_pan_rot_deg = desired_pan_deg - self.cur_arm_pose[0]
        self.calc_tgt_pan_deg()
        self.pan_residual = desired_pan_deg - self.tgt_pan_deg
        self.tgt_waffle_rot_deg += self.pan_residual

    def calc_tgt_tilt_deg(self):
        self.tgt_tilt_deg = self._clip(self.cur_camera_object_tilt_deg, self.tilt_deg_limits)

    def calc_waffle_linear_vel(self):
        """수평 거리 P제어. 옆/뒤 물체에는 회전 우선, 나머지는 방향에 따라 감속한다."""
        dis_error = self.cur_object_planar_dis - self.tgt_dis
        if abs(dis_error) <= self.dis_error_threshold or abs(self.tgt_yaw_rot_deg) >= self.linear_heading_limit_deg:
            self.tgt_waffle_linear_vel = 0.0
        else:
            self.tgt_waffle_linear_vel = self._clip(
                self.dis_gain * dis_error * math.cos(math.radians(self.tgt_yaw_rot_deg)),
                self.waffle_linear_vel_limits,
            )

    def calc_waffle_angular_vel(self):
        """분배된 회전각(deg)을 P제어 각속도(rad/s)로 변환해 저장한다."""
        tgt_waffle_yaw_rad = math.radians(self.tgt_waffle_rot_deg)
        self.tgt_waffle_angular_vel = self._clip(
            self.waffle_yaw_gain * tgt_waffle_yaw_rad,
            (-self.waffle_max_angular_vel, self.waffle_max_angular_vel),
        )

    def wheel_align(self):
        """tracking 기능: bbox 중심이 아닌 차량 기준 물체 방위각으로 정렬한다."""
        self.tgt_waffle_linear_vel = 0.0
        self.tgt_waffle_angular_vel = self._clip(
            self.wheel_align_gain * math.radians(self.tgt_yaw_rot_deg),
            (-self.waffle_max_angular_vel, self.waffle_max_angular_vel),
        )

    # ================= 상태 머신 =================
    def _stop_vehicle(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0

    def _arm_to_nominal(self):
        self.tgt_pan_deg, self.tgt_tilt_deg = self.nominal_pose
        self.tgt_arm_pose = self.nominal_pose.copy()

    def _enter_searching(self):
        self.state = 'searching'
        self.lost_side = self.lost_pose()
        self.search_cnt = 0
        self.search_rot_deg = 0.0
        self.prev_yaw_deg = self.cur_yaw_deg
        self._search_arm_target()

    def _search_arm_target(self):
        """탐색 중 팔: pan 은 nominal(차체 회전으로 훑음), tilt 는 현재 바퀴의 높이."""
        level = self.search_tilt_levels[min(self.search_cnt, len(self.search_tilt_levels) - 1)]
        self.tgt_pan_deg, self.tgt_tilt_deg = self.nominal_pose[0], level
        self.tgt_arm_pose = [self.tgt_pan_deg, self.tgt_tilt_deg]

    def state_machine_run(self):
        if self.health_blocked:
            # fault 유지: check_health에서 정한 정지 명령(속도 0, 팔 fault_arm_pose)을 그대로 발행
            self._stop_vehicle()
            return
        if self.state == 'fault':
            # 장애 해소 → idle (같은 주기에 idle 동작: 팔 nominal, 검출 대기)
            self.state = 'idle'
            self.recovered_from = self.last_fault_reason
            self.recovered_until = self._now_s() + self.recovery_reason_hold_s
        # 인지 토픽 침묵: 미검출(z=0 수신)과 구분해 상태를 유지한 채 차체 정지
        if self.detection_stale and self.state in ('idle', 'tracking', 'searching'):
            self._stop_vehicle()
            if self.state == 'idle':
                self._arm_to_nominal()
            return

        if self.state == 'idle':
            self._arm_to_nominal()
            self._stop_vehicle()
            if self.detect_streak >= self.recover_frames:
                self.state = 'tracking'  # tracking 동작은 다음 제어 주기에 실행한다.

        elif self.state == 'tracking':
            if self.cur_depth == 0:
                # 미검출 첫 프레임부터 정지, 탐색은 다음 제어 주기에 실행한다.
                self._enter_searching()
                self._stop_vehicle()
                return

            if not self.cur_arm_pose_valid:
                # 팔 피드백 무효: 위치 계산 불가 → 차체 정지, 팔은 nominal 복귀
                self._stop_vehicle()
                self._arm_to_nominal()
                return

            # 방어: 비정상 값(NaN 등)이면 위치가 None → 목표 속도 0, 팔은 직전 목표 유지
            self.calc_tracking_targets(
                self.cur_object_xyz, self.cur_yaw_deg,
                self.cur_arm_pose[0], self.cur_arm_pose[1],
            )
            self.cmd_vel_msg.linear.x = self.tgt_waffle_linear_vel
            self.cmd_vel_msg.angular.z = self.tgt_waffle_angular_vel

        elif self.state == 'searching':
            if self.detect_streak >= self.recover_frames:
                self.state = 'tracking'
                self._stop_vehicle()
                return

            self.cmd_vel_msg.linear.x = 0.0
            if self.heading_stale:
                # 회전량을 셀 수 없으면 회전하지 않는다.
                self.cmd_vel_msg.angular.z = 0.0
                self.prev_yaw_deg = self.cur_yaw_deg
                return

            # 오른쪽에서 사라짐 → 시계방향(ω < 0), 그 외 → 반시계방향(ω > 0)
            direction = -1.0 if self.lost_side == 'right' else 1.0
            self.cmd_vel_msg.angular.z = direction * self.search_angular_vel

            ddeg = self._wrap_deg(self.cur_yaw_deg - self.prev_yaw_deg)
            self.prev_yaw_deg = self.cur_yaw_deg
            self.search_rot_deg += ddeg
            self.search_cnt = int(abs(self.search_rot_deg) // 360)
            self._search_arm_target()             # 바퀴가 바뀌면 고개 높이도 바뀐다

            if self.search_cnt >= self.search_max_turns:
                self.search_cnt = 0
                self.state = 'lost'
                self.cmd_vel_msg.angular.z = 0.0

        elif self.state == 'lost':
            self._stop_vehicle()
            self._arm_to_nominal()
            trans_done = self.cur_arm_pose_valid and all(
                abs(cur - nominal) < self.nominal_tolerance_deg
                for cur, nominal in zip(self.cur_arm_pose, self.nominal_pose)
            )
            if trans_done:
                self.state = 'idle'

    # ================= 발행 =================
    def status_reason(self):
        """사람이 읽는 reason. 내부 판정값(health_reason)은 그대로 두고 표시만 바꾼다."""
        reason = self.health_reason
        if self.health_blocked:
            return reason
        if self.recovered_from and self._now_s() < self.recovered_until:
            return f'recovered_from:{self.recovered_from}'
        if reason == 'health_disabled':
            reason = 'ok'               # 진단 검사 OFF 여부는 diag 항목으로 따로 표시
        if reason == 'ok' and self.state == 'lost':
            return 'target_lost'
        return reason

    def diag_text(self):
        """health 진단 토픽 검사 상태. OFF 면 무시 중인 진단 목록을 함께 보여준다."""
        if self.health_enabled:
            return 'on'
        return 'off[' + ','.join(SENSORS) + ']'

    def _status_text(self):
        flags = []
        if self.detection_stale:
            flags.append('DETECTION_TIMEOUT')
        if not self.cur_arm_pose_valid:
            flags.append('ARM_INVALID')
        if self.imu_stale:
            flags.append('IMU_TIMEOUT')
        if self.heading_stale:
            flags.append('HEADING_UNAVAILABLE')
        reason = self.status_reason()
        text = self.state.upper()
        text = f'{text}|{",".join(flags)}' if flags else text
        return f'{text} reason={reason} heading={self.heading_source} diag={self.diag_text()}'

    def _publish(self):
        self.cmd_vel_msg.angular.z = self._clip(
            self.cmd_vel_msg.angular.z,
            (-self.waffle_max_angular_vel, self.waffle_max_angular_vel),
        )
        self.cmd_vel_pub.publish(self.cmd_vel_msg)  # /cmd_vel 발행
        if self.arm_publish_enabled:
            self.arm_cmd_pub.publish(Float32MultiArray(data=[float(v) for v in self.tgt_arm_pose]))
        self.status_pub.publish(String(data=self._status_text()))  # /tracking_status 발행

    def run(self):
        self.check_timeouts()
        self.check_health()
        self.calc_cur_object_pos()
        self.state_machine_run()
        self._publish()

    def stop_and_publish(self):
        """종료 시 정지 명령을 한 번 보낸다."""
        self._stop_vehicle()
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


def main():
    rclpy.init()
    p = PlanningMaster()
    try:
        rclpy.spin(p)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            p.stop_and_publish()
        p.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
