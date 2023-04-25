from sa_sensor_planner.util.planner import planner
from sa_msgs.msg import Velocity, Plan, PoseEstimation, PoseArray, Pose, VisibilityArray, RobotState
from sa_trajectory_estimation.util.util import motion_model
from sa_world_model.util.uav import uav
from sa_world_model.util.camera_sensor import camera
from typing import List, Dict, Optional
import numpy as np
import copy
import yaml, os
import itertools
from ament_index_python.packages        import get_package_share_directory
from sa_world_model.util.util import euclidean_dist
from sa_trajectory_estimation.util.util import RangeBearingKF
from sa_sensor_planner.util.heuristic_planner import generate_waypoints
from sa_sensor_planner.util.base_policy import middle_point

class single_sensor_nbo(planner):

    def __init__(
        self, 
        sensor_para_list: dict,
        dynamic_object_list: List[str],
        static_object_list: List[str],
        SampleDt: float,
        horizon: int,
        interest_object_list: List[str],
        robot_id: int,
        v_max: float,
    ) -> None:
        super().__init__()
        self.sensor_para_list = sensor_para_list
        self.dynamic_object_list = dynamic_object_list
        self.static_object_list = static_object_list
        self.interest_object_list = interest_object_list
        
        self.SampleF: Dict[str, np.ndarray] = {}
        self.SampleQ: Dict[str, np.ndarray] = {}
        self.SampleDt = SampleDt
        self.horizon = horizon
        self.traj_dict: Dict[int, dict] = {}
        self._init_motion_model()
        self.robot_id = robot_id
        self.v_max = v_max
        self._init_camera_agents()     
        self.plan_style = "heuristic"
        
    
    def _init_camera_agents(self):
        # init all robots carrying cameras
                
        robot_file = open(os.path.join(
            get_package_share_directory('sa_world_model'),
            'config','robots.yaml'))
        robots = yaml.safe_load(robot_file)

        self.sensor_robots: Dict[int, camera] = dict()
        for instance in range(len(robots)):
        
            robot_para = robots[instance]
            if robot_para['sensor']['type'] != 'camera':
                continue
            
            init_pose = Pose()
            init_pose.x = robot_para['sim_start']['x']
            init_pose.y = robot_para['sim_start']['y']
            init_pose.yaw = robot_para['sim_start']['yaw']
            sensor_obj = camera(robot_para['sensor']['fov']['x'], 
                                robot_para['sensor']['fov']['y'],
                                robot_para['type'] == 'uav', 
                                robot_para['domain_id'], 
                                robot_para['sensor']['alpha'],
                                robot_para['sensor']['r0'], 
                                init_pose)
            self.sensor_robots[robot_para['domain_id']] = sensor_obj
        
        self.width = robots[self.robot_id - 1]['sensor']['fov']['width']
        self.height = robots[self.robot_id - 1]['sensor']['fov']['height']

                
    def update_robot_state(self, msg: RobotState):

        if msg.robot_id in self.sensor_robots.keys():
            self.sensor_robots[msg.robot_id]._adjust_fov(msg.pose)
    
    def _init_motion_model(self):

        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        F_static, Q_static, _ = motion_model(True, self.SampleDt, self.sensor_para_list['1']["sigma_a"])
        for label in self.static_object_list:
            self.SampleF[label] = F_static
            self.SampleQ[label] = Q_static
        
        F_dynamic, Q_dynamic, _ = motion_model(False, self.SampleDt, self.sensor_para_list['1']["sigma_a"])
        for label in self.dynamic_object_list:
            self.SampleF[label] = F_dynamic
            self.SampleQ[label] = Q_dynamic
        
        self.R_dict: Dict[int, np.ndarray] = {}
        self.sensor_qual_dict: Dict[int, float] = {}
        
        for robot_id_str, sensor_para in self.sensor_para_list.copy().items():
            
            # convert key from str to int
            robot_id = int(robot_id_str)
            self.sensor_para_list[robot_id] = sensor_para
            del self.sensor_para_list[robot_id_str]

            self.sensor_qual_dict[robot_id] = sensor_para["quality"]
            self.R_dict[robot_id] = sensor_para["quality"] * \
                np.diag([sensor_para["r"], sensor_para["r"]])

    def plan(
        self, 
        visibility: Optional[List[VisibilityArray]]
    ) -> Plan:
        '''
        Single agent plan without considering other sensor's movement
        '''
        # here we use the heurisitic planning


        plan = Plan()

        if len(self.trajInTime[0]) == 0:
            for _ in range(self.horizon):
                action = Velocity()
                action.vx = 0.0
                action.vy = 0.0
                plan.plan.append(action)
        else:
            
            if self.plan_style == 'heuristic':
                plan = self._target_oriented_spatial_temp_heurisitc(visibility)
            elif self.plan_style == 'myopic':
                plan = self._myopic_plan(visibility)
        return plan

    def _myopic_plan(self, visibility) -> Plan:
        '''
        either following one specific point, 
        or move with the centroid of all points
        '''
        pose = self.sensor_robots[self.robot_id].robot_pose

        action_options: List[List[List]] = []
        values = []

        # follow one of the agent
        for i in range(len(self.trajInTime[0])):
            x_k = [pose.x, pose.y]
            u_vec = Plan()
            for t in range(self.horizon):
                # print(x_k)
                u, x_k = self.heuristic_action(x_k, self.trajInTime[t][i])
                action = Velocity()
                action.vx = u[0]
                action.vy = u[1]
                u_vec.plan.append(action)
            action_options.append(u_vec)
            values.append(self.f_obj(u_vec, visibility))
        
        x_k = [pose.x, pose.y]
        u_vec = Plan()
        for t in range(self.horizon):
            # print(x_k)
            u, x_k = self.heuristic_action(x_k, middle_point(self.trajInTime[t]))
            action = Velocity()
            action.vx = u[0]
            action.vy = u[1]
            u_vec.plan.append(action)
        action_options.append(u_vec)
        values.append(self.f_obj(u_vec, visibility))
        min_value = min(values)
        plan = action_options[values.index(min_value)]
        return plan

    def _target_oriented_spatial_temp_heurisitc(self, visibility) -> Plan:
        '''one method we tried'''
        pose = self.sensor_robots[self.robot_id].robot_pose
        spatial_temp_goal_pt: List[List[List[float]]] = list()
        for t in range(self.horizon):
            list_of_points: List[np.ndarry] = self.trajInTime[t]
            wpts= generate_waypoints(
                            list_of_points,
                            self.width,
                            self.height,)
            spatial_temp_goal_pt.append(wpts)

        # generate action options, exhaustive searching method
        action_options: List[List[List]] = []
        values = []

        for wpts in itertools.product(*spatial_temp_goal_pt):

            x_k = [pose.x, pose.y]
            u_vec = Plan()
            for wpt in wpts:
                # print(x_k)
                u, x_k = self.heuristic_action(x_k, wpt)
                action = Velocity()
                action.vx = u[0]
                action.vy = u[1]
                u_vec.plan.append(action)
            action_options.append(u_vec)
            values.append(self.f_obj(u_vec, visibility))
            # print(("one round ends"))
        # find min and return the action
        min_value = min(values)
        plan = action_options[values.index(min_value)]
        return plan
            
    def heuristic_action(self, x_k: List[float], wpt: List[float]):
        dx = wpt[0] - x_k[0]
        dy = wpt[1] - x_k[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
        v = min(self.v_max, dist/self.SampleDt)

        if np.isclose(v, 0.0):
            vx = 0.0
            vy = 0.0
        else:
            vx = v * dx / dist
            vy = v * dy / dist

        x_k = [x_k[0] + vx * self.SampleDt, x_k[1] + vy * self.SampleDt]
        
        return [vx, vy], x_k
    
    def f_obj(
        self, 
        u: Plan, 
        visibility: Optional[List[VisibilityArray]]
    ) -> float:
        '''
        sequential centralized kf update
        multi-objective - add number of targets in side FoV
        '''
        # print(visibility)
        objValue = 0.0
        tc = 0
        # 1. initiate robots
        agent_dict = copy.deepcopy(self.sensor_robots)
        agent_dict[self.robot_id].move_sensor(u.plan[0], self.SampleDt)
        
        # 2. initiate targets
        tracker: List[RangeBearingKF] = list()

        for index, traj in self.traj_dict.items():
            x0 = self.trajInTime[0][index]
            x0 = np.matrix([[x0[0]],
                            [x0[1]],
                             [0],
                             [0]])
            label, P0 = traj["label"], traj["P0"]
            kf = RangeBearingKF(self.SampleF[label], self.H, x0, P0, \
                                self.SampleQ[label], 
                                copy.deepcopy(self.R_dict[self.robot_id]), \
                                self.sensor_para_list[self.robot_id]["r0"], \
                                quality=self.sensor_qual_dict[self.robot_id])
            tracker.append(kf)
        
        # 3. simulate the NBO

        for i in range(self.horizon):
            t = (i + 1) * self.SampleDt
            z_k = self.trajInTime[i]
            
            # start using a sequential way to estimate KF
            for j in range(len(z_k)):
                z = z_k[j]
                label = self.traj_dict[j]["label"]
                tracker[j].predict(self.SampleF[label], self.SampleQ[label])
                isObserved = False
                if visibility[i].visibilities[j]:
                
                    R = np.matrix(np.zeros((2, 2)))
                    # find any observing agents
                    for _, agent in agent_dict.items():
                        if agent._is_in_fov(z[0], z[1]):
                            R_i = tracker[j].cal_R(z, 
                                        [agent.robot_pose.x, agent.robot_pose.y, agent.robot_pose.yaw],
                                        alpha=self.sensor_qual_dict[agent.robot_id]) 
                            R += np.linalg.inv(R_i)
                            isObserved = True

                # update all agents track
                if isObserved:
                    info_sum = np.dot(np.dot(self.H.T, R), self.H)
                    P_fused = np.linalg.inv(np.linalg.inv(tracker[j].P_k_k_min) + info_sum)
                    tracker[j].P_k_k = P_fused
                    
                else:
                    tracker[j].P_k_k = copy.deepcopy(tracker[j].P_k_k_min)
            
                trace = np.trace(tracker[j].P_k_k[0:2, 0:2])
                                
                objValue += trace # (self.gamma ** i) # * weight
            
            # 4. agent movement policy
            if i <= self.horizon-2:
                tc = t
                # print("at time %s" % (t - self.t))
                agent_dict[self.robot_id].move_sensor(u.plan[i + 1], self.SampleDt)
            
          
        return objValue
    

    def sample(self, est_list: List[PoseEstimation]) -> List[PoseArray]:

        nbo_sample = []
        traj_id = 0

        self.traj_dict.clear()

        for est in est_list:

            if est.detection_label not in self.interest_object_list:
                continue

            trajectory: List[List[float]] = []

            # shape x(4, 1)
            x = np.matrix([est.pose.x, est.pose.y, est.velocity.vx, est.velocity.vy]).T
            
            self.traj_dict[traj_id] = {
                "label": est.detection_label,
                "P0": np.matrix(est.covariance).reshape(4, 4),
            }
            traj_id += 1

            for _ in range(self.horizon):
                # sample one place
                
                x = np.dot(self.SampleF[est.detection_label], x)
                trajectory.append([x[0, 0], x[1, 0]])
                
            nbo_sample.append(trajectory)
        
        self.trajInTime: List[List[np.ndarray]] = []
        traj_array: List[Pose] = []
        for t in range(self.horizon):
            pos_list = []
            pose_array = PoseArray()
            for traj in nbo_sample:
                pos_list.append(traj[t])
                pose = Pose()
                pose.x = traj[t][0]
                pose.y = traj[t][1]
                pose_array.traj.append(pose)
            self.trajInTime.append(pos_list)
            traj_array.append(pose_array)
        
        return traj_array
    