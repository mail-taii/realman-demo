import rospy
from embodied_arm_msgs.msg import Lift_Height,Plan_State

class LiftClient:
    """升降机客户端"""
    
    def __init__(self):
        """初始化升降机客户端"""
        self.pub_height = rospy.Publisher("/l_arm/rm_driver/Lift_SetHeight", Lift_Height, queue_size=10)
        
        self.planState = rospy.Subscriber("/l_arm/rm_driver/Plan_State", Plan_State, self.lift_height_callback)
        self.run_state = False
        rospy.sleep(1)
        
    def lift_height_callback(self, msg):
        if msg.state:
            self.run_state = True
            rospy.loginfo("*******Lift State OK")
        else:
            self.run_state = False
            rospy.loginfo("*******Lift State Fail")

    def lift_height(self, height, speed=30):
        """升降机升降"""
        height_msg = Lift_Height()
        height_msg.height = height
        height_msg.speed = speed
        self.pub_height.publish(height_msg)
        rospy.loginfo(f"发布目标高度: {height_msg.height}")
        self.is_arrive()

    def is_arrive(self):
        """升降机升降成功检测"""
        while 1:
            rospy.sleep(0.1)
            if self.run_state:
                self.run_state = False
                break


if __name__ == "__main__":
    lift_client = LiftClient()
    lift_client.lift_height(200)
    rospy.spin()