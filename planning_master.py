from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
# VFOV = 58 deg
class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, '/arm/command', 10)
        self.nominal_pose = [0, 0] #TODO: check
        self.arm_target_pose = [0, 0]
        self.arm_motor_pos = [0, 0]

        self.e_x = 0
        self.e_y = 0
        self.depth = -1
        self.cur_deg = 0

        self.cmd_vel_msg = Twist()

        self.last_state = None
        self.state = 'idle'


    def lost_pose(self):
        width = 640 # 인지 yaml에서 가져오기
        bbox_x = width / 2 - self.e_x # 민식큄님한테 e_x 계산식 받아오기
        bbox_x_ratio = bbox_x / width
        if  bbox_x_ratio < 1/4: # 왼쪽에서 사라지는 경우
            result = "left"
        elif bbox_x_ratio < 3/4: # 중앙에서 사라지는 경우 
            result = "middle"
        else: # 오른쪽에서 사라지는 경우
            result = "right"

        return result

    
    def imu_cb(self, msg): # Float32
        self.cur_deg = msg.data

    def cam_cb(self, msg): # PointStamped
        self.e_x = msg.point.x
        self.e_y = msg.point.y
        self.depth = msg.point.z # depth 대체 
        self.cam_timestamp = msg.header.stamp

    def motor_cb(self, msg): # Float32MultiArray
        self.arm_motor_pos = msg.data
        
    def calc_arm_cmd(self):
        if self.state in ['idle']:
            self.arm_target_pose = self.nominal_pose
            
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
            
        elif self.state in ['lost']:
            self.arm_target_pose = self.nominal_pose

            trans_done = abs(self.arm_motor_pos[0] - self.nominal_pose[0]) < 1 and abs(self.arm_motor_pos[1] - self.nominal_pose[1]) < 1    
            if trans_done:
                self.state = 'idle'
        
            
        
    def calc_cmd_vel(self):                 
        if self.state in ['idle']:
            # 와플
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.x = 0
            
        
        if self.state in ['tracking']:
            if self.depth == -1:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.x = 0.0000000000 # TODO: 와플에 분배된 회전각도를 와플 종속도로 저장
            else:
                self.cmd_vel_msg.linear.x = 0.0000000000 # TODO: 와플에 분배된 회전각도를 와플 종속도로 저장
                self.cmd_vel_msg.angular.x = 0.0000000000 # TODO: 와플에 분배된 회전각도를 와플 각속도로 저장

        
        elif self.state in ['searching']:
            if self.last_state != 'searching':
                self.search_start_deg = self.arm_motor_pos[0]#arm 하단 모터: from perception

            puck_detected =  self.cam_timestamp - rclpy.clock.Clock().now() < 0.5
            if puck_detected:
                self.state = 'tracking'
            else: 
                self.cmd_vel_msg.linear.x = 0
                if self.lost_pose() == 'left':
                    self.cmd_vel_msg.angular.x = +0.0000000000 # TODO: 사전 지정 각속도 저장
                elif self.lost_pose() == 'middle':
                    self.cmd_vel_msg.angular.x = +0.0000000000 # TODO: 사전 지정 각속도 저장
                elif self.lost_pose() == 'right':
                    self.cmd_vel_msg.linear.x = -0.0000000000 # TODO: 사전 지정 각속도 저장
                else:
                    raise ValueError("lost_pose() error")
                        
                ddeg = self.last_deg - self.cur_deg 
                if ddeg > 355:
                    self.search_cnt += 1
                if ddeg < -355:
                    self.search_cnt -= 1
                
                if self.search_cnt == 2:
                    self.search_cnt = 0
                    self.state = 'lost'
                    

        elif self.state in ['lost']:
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.x = 0
        
        
    def _publish(self):
        self.cmd_vel_pub.publish(self.cmd_vel_msg)  # /cmd_vel 발행
        self.arm_cmd_pub.publish(Float32MultiArray(data=[float(v) for v in self.arm_target_pose]))  # /arm/command 발행
        
    def run(self):                      
        self.calc_arm_cmd()
        self.calc_cmd_vel()
        self._publish() 
        
        

def main():
    rclpy.init()
    p = PlanningMaster()
    while rclpy.ok():
        rclpy.spin_once(p, timeout_sec=0.01)
        p.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
