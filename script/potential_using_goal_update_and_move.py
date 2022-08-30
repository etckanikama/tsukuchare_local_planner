#!/usr/bin/env python3
# license removed for brevity
import math 
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

enemy1_x , enemy1_y = 100, 100
# enemy2_x , enemy2_y = 0.1, 0.1
# enemy3_x , enemy3_y = -0.5, 1
WAYPOINT_CSV = '/home/hirayama-d/tsukuchare22_ws/src/potential_planner/data/waypoint_csv/sample_waypoint.csv'
# goals = [[1,0],[1,1],[0,1],[0,0]]


class ymbcNode:
    def __init__(self):
        rospy.init_node("sampleNode")

        # subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.cbOdom)
        # publisher
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.x = 0
        self.y = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # self.odom = Odometry()

    def cbOdom(self, data):
        # coodinate
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        # orientation
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # change to euler
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    
    def publish(self, data):
        self.pub.publish(data)

def get_pot(x , y, goal_x, goal_y):
    weight_enemy1 = 0.0
    # weight_enemy2 = 1.0
    # weight_enemy3 = 1.0
    weight_goal = 50.0
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

    delta = 0.1
    rate = rospy.Rate(10)
    node.x, node.y = 0, 0
    cmd_vel.linear.x = 0.1

    # waypoint_csv to list
    waypoint = [
        list(
            map(float,line.rstrip().split(","))
            ) for line in open(WAYPOINT_CSV).readlines()]

    # goal_x, goal_y = goals[0][0], goals[0][1] 
    goal_x, goal_y = waypoint[0][0], waypoint[0][1] 
    status = waypoint[0][2] # 5は前進中
    idx = 0

    while not rospy.is_shutdown():
        node.cmd_pub.publish(cmd_vel)


        vx = -(get_pot(node.x + delta, node.y, goal_x, goal_y) - get_pot(node.x,node.y,goal_x, goal_y))/ delta
        vy = -(get_pot(node.x, node.y + delta, goal_x, goal_y) - get_pot(node.x,node.y, goal_x, goal_y))/ delta
        # print("vx vy",vx,vy)
        v = math.sqrt(vx*vx + vy*vy)
        #print("vx,vy,v,cmd_vel, node.x, node.y ,goal_x,goal_y ___ ",vx, vy,v, cmd_vel.linear.x,node.x,node.y,goal_x,goal_y)
        # print("odom.x, odom.y ___ ",node.x,node.y)
        print("goal_x, goal_y ___ ",goal_x,goal_y)

        vx /= v / cmd_vel.linear.x
        vy /= v / cmd_vel.linear.x
        next_theta = math.atan2(vy, vx) - node.yaw
        
        # next_tehta ga 3.14 ika ni suru syori sisei no hanashi 
        if next_theta > math.pi:
            next_theta -= 2*math.pi
        elif next_theta <= -math.pi:
            next_theta += 2*math.pi
        # print("yaw",node.yaw)
        print("next_theta",next_theta)

        cmd_vel.angular.z = next_theta
        # print("odometry",node.x, node.y)
        
        euclid_distance = math.sqrt((goal_x - node.x)**2 + (goal_y - node.y)**2)
        # print("distance",euclid_distance)
        

        if euclid_distance < 0.1:
            print("status",waypoint[idx][2])
                

            if waypoint[idx][2] == 8: # When "8" is received, stop for 2 seconds.
                cmd_vel.angular.z = 0.0
                node.cmd_pub.publish(cmd_vel)
                rospy.sleep(2.0)
                
                

            if len(waypoint) == idx: # Finish when you exit the array.
                cmd_vel.angular.z = 0.0
                cmd_vel.linear.x = 0.0  
                break
        
            idx += 1
            goal_x, goal_y = waypoint[idx][0], waypoint[idx][1]


        
   

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass