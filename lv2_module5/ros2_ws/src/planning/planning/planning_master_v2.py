import math                                                   # [추가] odom_cb 의 쿼터니언 → yaw 변환용
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped                   # [추가] /target 구독용
from std_msgs.msg import Float32, Float32MultiArray
import rclpy
from rclpy.node import Node
from planning.state_machine import HealthGate                 # [추가] 이슈 #6: health 기반 fail-safe 전이
# VFOV = 58 deg

# [추가] 발제 문서 규약값
INPUT_TIMEOUT = 0.5      # [추가] 마지막 /target 수신 후 이 시간(초)이 지나면 입력 끊김
RECOVER_FRAMES = 3       # [추가] 연속 검출 프레임 수가 이 값 이상이어야 TRACKING 진입/복귀
SEARCH_MAX_TURNS = 2     # [추가] 탐색 회전 상한(바퀴). 원본의 search_cnt == 2 조건


class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, '/arm/command', 10)
        # [추가] /target 구독 (cam_cb 연결). QoS는 발제 문서 기준 best-effort, depth 1
        self.create_subscription(PointStamped, '/target', self.cam_cb, rclpy.qos.qos_profile_sensor_data)
        # [수정] imu_cb 구독 연결: imu_driver.py 가 /imu/yaw (Float32) 로 발행
        self.create_subscription(Float32, '/imu/yaw', self.imu_cb, rclpy.qos.qos_profile_sensor_data)
        # [수정] motor_cb 구독 연결. TODO: '/arm/state' 는 가정한 토픽명 — 제어 담당에게 확인 (틀리면 arm_motor_pos 가 [0,0] 으로 남아 LOST 정지 위치가 틀어짐)
        self.create_subscription(Float32MultiArray, '/arm/state', self.motor_cb, 10)
        self.nominal_pose = [0, 0] #TODO: check
        self.arm_target_pose = [0, 0]
        self.arm_motor_pos = [0, 0]

        self.e_x = 0
        self.e_y = 0
        self.depth = -1          # [수정 없음] depth == -1 은 "미검출" (팀장이 튜터에게 확인)
        self.cur_deg = 0

        self.cmd_vel_msg = Twist()
        # [수정] 고정 주기(50Hz)로 run() 호출: spin_once 루프는 메시지 도착에 따라 주기가 변했음
        self.create_timer(0.02, self.run)

        self.last_state = None
        self.state = 'idle'

        # [추가] 초기화되지 않았던 변수들
        self.enabled = True            # [추가] 시작/중지 명령. TODO: 토픽·서비스 연결
        self.search_enabled = False    # [추가] SEARCHING은 선택 기능. 시야 내 재등장 시험(SC3)은 False, SC4~SC6은 True
        self.search_delay = 0.0         # [추가] TODO: LOST가 이 시간(초) 지속되면 SEARCHING 진입
        self.cam_timestamp = None       # [추가]
        self.last_cam_rx = None         # [추가] 마지막 /target 수신 시각 (노드 시계 기준)
        self.detect_cnt = 0             # [추가] 연속 검출 프레임 수
        self.last_valid_ex = 0.0        # [추가] 마지막으로 검출됐을 때의 ex (소실 방향 판단용)
        self.lost_since = None          # [추가]
        self.search_done = False        # [추가] 탐색 상한까지 돌았는데 못 찾음 → 재탐색 금지
        self.search_cnt = 0             # [추가]
        self.search_rot = 0.0           # [추가] 탐색 중 누적 회전각(deg, 부호 있음)
        self.tilt = 0.0                 # [추가]

        # [추가] 이슈 #6: health 4개 구독 + /tracking_status 발행 (토픽명·타임아웃은 planning.yaml)
        self.gate = HealthGate(self)
        # [추가] 이슈 #6: IMU STALE → 엔코더 odometry 로 heading 추정 fallback (선택 기능)
        self.heading_source = 'imu'        # 'imu' | 'encoder' — health 판정이 바꿈 (run 참고)
        self._heading_resync = True        # True 면 다음 샘플은 기준값으로만 쓰고 delta 는 계산하지 않음
        # TODO(이슈 #6): 엔코더 heading 은 control/base_kinematics 가 가공해 발행하는 값을 구독만 하면 됨 (팀장님 답변).
        #   토픽명·메시지 타입·단위(deg/rad)는 아직 미확정 → 확정되면 아래 구독을 켜고 odom_cb 를 그 타입에 맞게 수정,
        #   토픽명은 planning.yaml 의 odom_topic 으로 이동. (nav_msgs 타입이면 package.xml 에 <depend>nav_msgs</depend> 추가)
        #   self.create_subscription(<타입>, '<토픽>', self.odom_cb, rclpy.qos.qos_profile_sensor_data)


    def lost_pose(self):
        # [수정] /target의 ex는 정규화 오차(−1~1, 오른쪽 +). 픽셀 폭(640) 불필요
        # [수정] 소실 시점의 e_x는 의미가 없으므로 마지막으로 검출됐을 때의 값을 사용
        bbox_x_ratio = (self.last_valid_ex + 1.0) / 2.0   # [수정] 0(왼쪽 끝) ~ 1(오른쪽 끝)
        if  bbox_x_ratio < 1/4: # 왼쪽에서 사라지는 경우
            result = "left"
        elif bbox_x_ratio < 3/4: # 중앙에서 사라지는 경우 
            result = "middle"
        else: # 오른쪽에서 사라지는 경우
            result = "right"

        return result

    
    # [추가] heading 샘플 공통 처리. 현재 heading_source 와 다른 출처의 값은 버린다
    def _heading_sample(self, deg, source):
        if source != self.heading_source:
            return
        if self._heading_resync:           # 출처가 바뀐 직후: 좌표계가 다를 수 있어 첫 샘플은 기준값으로만 사용
            self.cur_deg = deg
            self._heading_resync = False
            return
        # [수정] SEARCHING 중에는 샘플마다 회전량을 누적 (wrap 처리: −180~180 로 접기)
        delta = (deg - self.cur_deg + 180.0) % 360.0 - 180.0
        if self.state == 'searching':
            self.search_rot += delta
        self.cur_deg = deg

    def imu_cb(self, msg): # Float32
        self._heading_sample(msg.data, 'imu')

    # [추가] TODO(이슈 #6): 구독은 아직 꺼져 있음(__init__ 참고). 임시로 nav_msgs/Odometry(쿼터니언→yaw) 를 가정한 코드 —
    #   base_kinematics 가 실제 발행하는 타입·단위로 확정되면 이 함수를 맞게 수정할 것
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._heading_sample(math.degrees(yaw), 'encoder')

    def cam_cb(self, msg): # PointStamped
        self.e_x = msg.point.x
        self.e_y = msg.point.y
        self.depth = msg.point.z # depth 대체 
        self.cam_timestamp = msg.header.stamp
        self.last_cam_rx = self.get_clock().now()      # [추가] 수신 시각 기록

        # [추가] 프레임 단위 연속 검출 카운트 (제어 주기가 아니라 /target 수신마다 센다)
        if self.depth != -1:
            self.detect_cnt += 1
            self.last_valid_ex = self.e_x              # [추가]
        else:
            self.detect_cnt = 0

    def motor_cb(self, msg): # Float32MultiArray
        self.arm_motor_pos = list(msg.data)   # [수정] array 가 아닌 list 로 보관

    # [추가] 입력이 신선한가 (마지막 /target 수신 후 INPUT_TIMEOUT 이내)
    def input_fresh(self):
        if self.last_cam_rx is None:
            return False
        elapsed = (self.get_clock().now() - self.last_cam_rx).nanoseconds * 1e-9
        return elapsed < INPUT_TIMEOUT

    # [추가] 상태 전이를 한 곳에서 관리 (원본은 calc_* 안에서 state를 직접 바꿨음)
    def _set_state(self, new_state):
        if new_state == self.state:
            return
        self.state = new_state
        if new_state == 'tracking':
            self.search_done = False
        elif new_state == 'lost':
            self.lost_since = self.get_clock().now()
        elif new_state == 'searching':
            self.search_rot = 0.0
            self.search_cnt = 0

    # [추가]
    def update_state(self):
        fresh = self.input_fresh()
        if not fresh:
            self.detect_cnt = 0   # 끊겼다가 재개되면 새로 3프레임을 세도록 리셋
        detected = fresh and self.depth != -1
        reacquired = fresh and self.detect_cnt >= RECOVER_FRAMES

        if not self.enabled:
            self._set_state('idle')

        elif self.state == 'idle':
            if reacquired:
                self._set_state('tracking')

        elif self.state == 'tracking':
            if not detected:                       # 미검출(z=-1) 또는 입력 타임아웃
                self._set_state('lost')

        elif self.state == 'lost':
            if reacquired:
                self._set_state('tracking')
            elif self.search_enabled and not self.search_done:
                lost_time = (self.get_clock().now() - self.lost_since).nanoseconds * 1e-9
                if lost_time >= self.search_delay:
                    self._set_state('searching')

        elif self.state == 'searching':
            self.search_cnt = int(abs(self.search_rot) // 360.0)
            if reacquired:
                self._set_state('tracking')
            elif abs(self.search_rot) >= 360.0 * SEARCH_MAX_TURNS:
                self.search_done = True            # 상한까지 못 찾음 → 정지, 자동 재탐색 안 함
                self._set_state('lost')
        
    def calc_arm_cmd(self):
        if self.state in ['idle']:
            self.arm_target_pose = list(self.nominal_pose)   # [수정] 별칭 방지: 복사
            
        elif self.state in ['tracking']:
            # TODO: IRS 기준 1px이 몇도에 해당하는지 계산(x, y 둘다) << 민식큄
            # TODO: e_x, e_y에 곱해서 목표 회전각도 생성 << 민식큄
            # TODO: 목표 회전각도를 와플과 팔에 분배(N:M)
            pass
                
        elif self.state in ['searching']:
            # TODO: tilt 간 회전 정지
            if self.search_cnt < 1:
                self.tilt = 0.00000000000 #TODO: 적절한 값 찾기
            elif self.search_cnt < 2:
                self.tilt = 0.00000000000 #TODO: 적절한 값 찾기
            # [수정] tilt 를 팔 목표(상하 축 [1])에 반영. 나머지 축은 현재 목표 유지
            self.arm_target_pose = [self.arm_target_pose[0], self.tilt]
            
        elif self.state in ['lost']:
            # [수정] nominal_pose 로 이동하지 않고, LOST 진입 순간의 팔 위치에서 즉시 정지
            if self.last_state != 'lost':
                self.arm_target_pose = list(self.arm_motor_pos)   # [수정] 한 번만 고정, 복사
            # [삭제] trans_done 판정과 self.state = 'idle' 전이 (전이는 update_state 에서)
        
            
        
    def calc_cmd_vel(self):                 
        if self.state in ['idle']:
            # 와플
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0     # [수정] angular.x → angular.z (회전은 z축)
            
        
        if self.state in ['tracking']:
            # [삭제] depth == -1 분기: -1은 미검출이므로 TRACKING 중에는 발생하지 않음 (update_state에서 LOST로 전이)
            self.cmd_vel_msg.linear.x = 0.0000000000 # TODO: 와플에 분배된 회전각도를 와플 종속도로 저장
            self.cmd_vel_msg.angular.z = 0.0000000000 # [수정] angular.x → angular.z / TODO: 와플에 분배된 회전각도를 와플 각속도로 저장

        
        elif self.state in ['searching']:
            if self.last_state != 'searching':
                self.search_start_deg = self.arm_motor_pos[0]#arm 하단 모터: from perception

            # [삭제] puck_detected 판정과 state = 'tracking' 전이 (Time - Time 은 Duration 이라 float 비교 불가였음. update_state 로 이동)
            self.cmd_vel_msg.linear.x = 0
            if self.lost_pose() == 'left':
                self.cmd_vel_msg.angular.z = +0.0000000000 # [수정] angular.x → angular.z / TODO: 사전 지정 각속도 저장
            elif self.lost_pose() == 'middle':
                self.cmd_vel_msg.angular.z = +0.0000000000 # [수정] angular.x → angular.z / TODO: 사전 지정 각속도 저장
            elif self.lost_pose() == 'right':
                self.cmd_vel_msg.angular.z = -0.0000000000 # [수정] linear.x → angular.z (오타) / TODO: 사전 지정 각속도 저장
            else:
                raise ValueError("lost_pose() error")
                    
            # [삭제] ddeg = last_deg - cur_deg / search_cnt 증감 / search_cnt == 2 → lost 전이
            #        (last_deg 미정의, 355° 비교가 wrap 을 처리하지 못했음 → imu_cb 누적 + update_state 로 대체)
                    

        elif self.state in ['lost']:
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0     # [수정] angular.x → angular.z
        
        
    def _publish(self):
        self.cmd_vel_pub.publish(self.cmd_vel_msg)  # /cmd_vel 발행
        self.arm_cmd_pub.publish(Float32MultiArray(data=[float(v) for v in self.arm_target_pose]))  # /arm/command 발행
        
    def run(self):                      
        self.update_state()             # [추가] 상태 전이를 먼저, 한 곳에서
        self._set_state(self.gate.decide(self.state))   # [추가] 이슈 #6: health 반영 (센서 이상 → lost, 사유는 /tracking_status)
        # [추가] 이슈 #6: IMU 이상이면 heading 출처를 엔코더로, 복구되면 IMU 로 (전환 시 기준값 재설정)
        src = 'encoder' if (self.gate.health and self.gate.health.use_encoder_heading) else 'imu'
        if src != self.heading_source:
            self.heading_source = src
            self._heading_resync = True
            self.get_logger().info(f'heading_source={src}')
        self.calc_arm_cmd()
        self.calc_cmd_vel()
        self._publish() 
        self.last_state = self.state    # [추가] 진입 순간 판정용 (원본은 갱신하지 않았음)
        
        

def main():
    rclpy.init()
    p = PlanningMaster()
    try:
        rclpy.spin(p)                   # [수정] run() 은 create_timer 가 호출
    except KeyboardInterrupt:
        pass
    finally:
        p.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()