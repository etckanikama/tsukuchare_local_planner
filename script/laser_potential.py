#!/usr/bin/env python3
# license removed for brevity
import math
from collections import defaultdict
import matplotlib.pyplot as plt


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

""""scanで受け取った生データを座標変換して、"""


# # enemy1_x , enemy1_y = 0.5, 0.2
# enemy2_x , enemy2_y = 0.1, 0.1
# enemy3_x , enemy3_y = -0.5, 1
goal_x , goal_y    = 3, 0
# goals = [[1,0],[2,0],[3,0],[4,0]]


STEP_ANGLE = 0.36
CENTER_ANGLE_INDEX = 363

class ymbcNode:
    def __init__(self):
        rospy.init_node("sampleNode")

        # subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.cbOdom)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.cbScan)
        # publisher
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.x = 0
        self.y = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.scan = LaserScan()

    def cbOdom(self, data):
        # coodinate
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        # orientation
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # change to euler
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
    
    def cbScan(self, data):
        self.scan = data
        # self.scan.header = data.header
        # self.scan.angle_min = data.angle_min
        # self.scan.angle_max = data.angle_max
        # self.scan.angle_increment = data.angle_increment
        # self.scan.time_increment = data.time_increment
        # self.scan.scan_time = data.scan_time
        # self.scan.range_min = data.range_min
        # self.scan.range_max = data.range_max
        # self.scan.ranges = data.ranges
        # self.scan.intensities = data.intensities
    
    def publish(self, data):
        self.pub.publish(data)


def get_pot(x , y,enemy1_x, enemy1_y):
    weight_enemy1 = 1.0
    # weight_enemy2 = 1.0
    # weight_enemy3 = 1.0
    weight_goal = 10.0
    enemy1_pot = 1.0 / math.sqrt((x - enemy1_x)*(x - enemy1_x) + (y - enemy1_y)*(y - enemy1_y))
    # enemy2_pot = 1.0 / math.sqrt((x - enemy2_x)*(x - enemy2_x) + (y - enemy2_y)*(y - enemy2_y))
    # enemy3_pot = 1.0 / math.sqrt((x - enemy3_x)*(x - enemy3_x) + (y - enemy3_y)*(y - enemy3_y))
    goal_pot = -1.0 / math.sqrt((x - goal_x)*(x - goal_x) + (y - goal_y)*(y - goal_y))

    # pot = enemy1_pot * weight_enemy1 + enemy2_pot *weight_enemy2 + enemy3_pot *weight_enemy3 + goal_pot* weight_goal
    pot = enemy1_pot * weight_enemy1 + goal_pot* weight_goal
    
    return pot


def main():
    node = ymbcNode()
    cmd_vel = Twist()

    node.x, node.y = 0.0, 0.0
    delta = 0.1
    # count = 0

    angle_xy = defaultdict(list) #idx_dict:x,y
    # enemy1_x , enemy1_y = 0, 0
    # flag = 0
    # print("node0:",node.x, node.y)    
    # laser_scanでの一点のオブジェクト登録用while
    cmd_vel.linear.x = 0.1
    cmd_vel.angular.z = 0.0

    # 1秒間にpublishする数の設定
    rate = rospy.Rate(10)


        
        # print(math.degrees(node.roll), math.degrees(node.pitch), math.degrees(node.yaw))
        # 0.1 秒スリープする。
    while not rospy.is_shutdown():
        node.cmd_pub.publish(cmd_vel)

        print("node1:",node.x, node.y)  
        # print("scan_range_len_363", node.scan.ranges[int(len(node.scan.ranges)/2)])
        # print("scan_range_len", node.scan.ranges)

        
        # if flag == 1:

        if not list(node.scan.ranges): #if [] continue
            continue
        print("scan_range_len_363", node.scan.ranges[int(len(node.scan.ranges)/2)])

        # # laserscanから受け取った距離データを座標に変換
        for i in range(360, 367):
            
            distance = list(node.scan.ranges)[i]
            
            if not math.isnan(distance):
                if distance < 0.5:
                    current_index = i
                    
                    angle_diff = CENTER_ANGLE_INDEX - current_index
                    theta = STEP_ANGLE * (math.pi / 180) * angle_diff
                    angle_tate = distance*math.cos(theta)
                    angle_yoko = distance*math.sin(theta)
                    angle_coodinate = [angle_tate, angle_yoko]
                    angle_xy[current_index] = angle_coodinate
                # print("i:{}, {} ,{},theta,{}".format(i, distance, type(distance),theta))
        # objectの平均座標登録(enemy1_x, enemy1_y)

        # if len(angle_xy)
        print("aa:",angle_xy)
        
        if len(angle_xy) == 0: #眼の前に障害物がなかったら飛ばす 
            continue

        for k, v in enumerate(angle_xy.items()):
            print(k,v, v[1][0]) # 0 (360, [0.0849849008462796, 0.0016021173926615132]) 0.0849849008462796

            tate = v[1][0] 
            yoko = v[1][1]
            enemy1_x += tate
            enemy1_y += yoko

        print("len_angle_xy",len(angle_xy))
        # enemy1_x /= len(angle_xy)
        # enemy1_y /= len(angle_xy)
        

        print("node.x={}, node.y={},   enemy_x={},enemy_y={}".format(node.x,node.y,enemy1_x,enemy1_y))
        
        vx = -(get_pot(node.x + delta, node.y, enemy1_x, enemy1_y) - get_pot(node.x,node.y, enemy1_x, enemy1_y))/ delta
        vy = -(get_pot(node.x, node.y + delta, enemy1_x, enemy1_y) - get_pot(node.x,node.y, enemy1_x, enemy1_y))/ delta
        v = math.sqrt(vx*vx + vy*vy)

        vx /= v / cmd_vel.linear.x
        vy /= v / cmd_vel.linear.x
        
        next_theta = math.atan2(vy, vx) - node.yaw

        cmd_vel.angular.z = next_theta


        if (goal_x - node.x < 0.02) and (goal_y - node.y < 0.02):
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0 

        
        rate.sleep()

    # print("enemy:{},{}".format(enemy1_x, enemy1_y))



        #goalに来たらループを抜ける。（速度指令を0にする）ゴール判定
    # 描画angle_xy = {} の切り分け
    # x_plt = []
    # y_plt = []
    # for k, v in angle_xy.items():
    #     print(k, v)
    #     x_plt.append(v[1])
    #     y_plt.append(v[0])
    # print(x_plt, y_plt)

    # plt.plot(x_plt, y_plt  , marker = 's', color = 'b', markersize = 15)
    # plt.plot(0, 0  , marker = 's', color = 'r', markersize = 15)
    # plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass