from sa_msgs.msg import Pose, Velocity, Plan
    
class robot:

    '''
    A basic robot type class, which carries the sensor in 
    the environment
    '''

    def __init__(self, x: float, y: float, yaw: float) -> None:
        
        self.pose = Pose()
        self.velocity = Velocity()
        # self.plan = Plan()
        self.pose.x = x
        self.pose.y = y
        self.pose.yaw = yaw
    
    def _dynamics(self, dt: float):
        pass
    