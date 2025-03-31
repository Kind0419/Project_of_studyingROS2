import rclpy
from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion,quaternion_from_euler
from rclpy.duration import Duration

class PatrolNode(BasicNavigator):
    def __init__(self,node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point',[0.0,0.0,0.0])
        self.declare_parameter('target_points',[0.0,0.0,0.0,1.0,1.0,1.57])
        self.initial_point_=self.get_parameter('initial_point').value
        self.target_points_=self.get_parameter('target_points').value

        #----------------------
        self.buffer_=Buffer()
        self.listener_=TransformListener(self.buffer_,self)

    def get_pose_by_xyyaw(self,x,y,yaw):
        # 通过x，y，yaw合成PoseStanped

        pose=PoseStamped()
        pose.header.frame_id='map'
        pose.pose.position.x=x
        pose.pose.position.y=y
        rotation_quat=quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x=rotation_quat[0]
        pose.pose.orientation.y=rotation_quat[1]
        pose.pose.orientation.z=rotation_quat[2]
        pose.pose.orientation.w=rotation_quat[3]
        return pose
        # pass

    def init_robot_pose(self):
        # 从参数获取初始化点
        self.initial_point_=self.get_parameter('initial_point').value
        # 合成位姿并进行初始化
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0],self.initial_point_[1],self.initial_point_[2]
        ))
        #等待导航直到激活
        self.waitUntilNav2Active()
        #pass

    def get_target_points(self):
        points=[]
        self.target_points_=self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x=self.target_points_[index*3]
            y=self.target_points_[index*3+1]
            yaw=self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'获取到目标点：{index}->({x},{y},{yaw})')
        return points
        #pass


    def nav_to_pose(self,target_pose):
        self.waitUntilNav2Active()
        result=self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback=self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计：{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds/1e9}s后到达')
        #结果判断
        result=self.getResult()
        if result==TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result==TaskResult.CANCELED:
            self.get_logger().info('导航结果：被取消')
        elif result==TaskResult.FAILED:
            self.get_logger().info('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')
        #pass

    def get_current_pose(self):
        while rclpy.ok():
            try:
                tf=self.buffer_.lookup_transform(
                    'map','base_footprint',rclpy.time.Time(seconds=0),rclpy.
                    time.Duration(seconds=1)
                )
                transform=tf.transform
                rotation_euler=euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
        #pass
            except Exception as e:
                self.get_logger().warn(f'不能获取坐标变换，原因：{str(e)}')
    
def main():
    rclpy.init()
    patrol=PatrolNode()
    patrol.init_robot_pose()

    while rclpy.ok():
        points=patrol.get_target_points()
        for point in  points:
            x,y,yaw=point[0],point[1],point[2]
            target_pose=patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.nav_to_pose(target_pose)

    rclpy.shutdown()

