import numpy as np
from numpy.linalg import inv
import copy
from scipy.stats import multivariate_normal
from scipy.stats.distributions import chi2
from sa_trajectory_estimation.util.util import measurement, track, RangeBearingKF, motion_model
from typing import Any, List, Tuple, Dict
from sa_msgs.msg import EstimationMsg, PoseEstimation

def normalize(a_list: List[float]) -> List[float]:
    # normalize a vector
    the_sum = sum(a_list)
    a_list = a_list / the_sum
    return a_list

def checkIfDuplicates(listOfElems: List[int]) -> bool:
    ''' Check if given list contains any duplicates, for
    example A B C is good, but A B B is not '''    
    setOfElems = set()
    # special case, all are false alarm, so the sum should be len * -1
    sum_ = sum(listOfElems)
    if sum_ == -len(listOfElems):
        return False
    for elem in listOfElems:
        if elem in setOfElems:
            if elem > 0: 
                return True
        else:
            setOfElems.add(elem)         
    return False

def common_fact(beta: float, 
    P_D: float,
    N_T: float,
    N_o: float
) -> float:
    if N_o > N_T:
        return beta ** (N_o - N_T)
    else:
        return (1 - P_D) ** (N_T - N_o)

class jpda:

    def __init__(self, 
        dt: float, 
        sensor_para_list: Dict[str, dict], 
        P_G: float = 0.98, 
        P_D: float = 0.9,
        ConfirmationThreshold: List[float] = [6, 8],
        DeletionThreshold: List = [7, 10],
        IsStatic: bool = False,
        label: str = "car",
    ):
        self.gating_size = chi2.ppf(P_G, df = 2)
        self.P_D = P_D  # probability of detection
        # Define gating size, for elliposidal gating, we need parameter P_G
        self.P_G = P_G  
        self.dt = dt
        self.IsStatic = IsStatic
        self.label = label
        self.sensor_para_list = copy.deepcopy(sensor_para_list)
        
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        # self.Q = 1 * np.diag([0.1, 0.1, 0.5, 0.5])
        
        self.common_P = 400
        
        if DeletionThreshold[1] < ConfirmationThreshold[1]:
            print("Deletion threshold period should be larger than confirmation threshold period")
        self.ConfirmationThreshold = ConfirmationThreshold
        self.DeletionThreshold = DeletionThreshold
        self.track_list: List[track] = list()
        self.track_list_next_index: List[int] = list()

        self.R_dict: Dict[int, np.ndarray] = {}
        self.sensor_qual_dict: Dict[int, float] = {}
        self.t = 0

        for robot_id_str, sensor_para in self.sensor_para_list.copy().items():
            
            # convert key from str to int
            robot_id = int(robot_id_str)
            self.sensor_para_list[robot_id] = sensor_para
            del self.sensor_para_list[robot_id_str]

            self.sensor_qual_dict[robot_id] = sensor_para["quality"]
            self.R_dict[robot_id] = sensor_para["quality"] * \
                np.diag([sensor_para["r"], sensor_para["r"]])  # not correct
        # init F, Q, F        
        self.F, self.Q, self.P = motion_model(self.IsStatic, dt, 
                                              self.sensor_para_list[1]["sigma_a"])
    

    def elliposidualGating(self, z_til: np.array, S: np.ndarray) -> int:
    
        value = np.dot(np.dot(z_til.T, inv(S)), z_til)[0,0]
        if value <= self.gating_size:
            return 1
        else:
            return 0
    
    def Gaussian_error(self, x0: np.ndarray, Q0, xi: np.ndarray, Qi) -> float:
        # metrics for similarity of 2 tracks
        # here all inputs are in np.matrix format
        error = x0.flatten() - xi.flatten()
        return np.dot(error, error.T)[0,0]
        # x_error = x0 - xi
        # error = np.dot(np.dot(x_error.T, np.linalg.inv(Q0 + Qi)), x_error) + np.log(np.linalg.det(Q0 + Qi))
        # return error[0,0]

    def checkSimilarity(self, id1: int, id2: int, robot_id: int) -> Tuple[bool, int]:
        '''
        compare trajectories of id1 and id2, check if they are close enough 
        via d0, delete the one with shorter history if found any
        '''
        if id1 != id2:
            x0 = self.track_list[id1].kf.x_k_k
            P0 = self.track_list[id1].kf.P_k_k
            xi = self.track_list[id2].kf.x_k_k
            Pi = self.track_list[id2].kf.P_k_k
            if self.Gaussian_error(x0, P0, xi, Pi) < self.sensor_para_list[robot_id]["d0"]:
                # keep the one with shorter history
                print("deleted because of the similarity")
                if self.track_list[id1].record["time_interval"][0] >\
                        self.track_list[id2].record["time_interval"][0]:
                    self.track_list[id1].deletion(self.t)
                    # print("conflict raise and delete ", x0[0,0], x0[1, 0])
                    return True, id1
                else:
                    self.track_list[id2].deletion(self.t)
                    # print("conflict raise and delete ", xi[0,0], xi[1, 0])
                    return True, id2
        return False, -1

    def cal_beta(self, N: float) -> float:
        #  normalized volume for ellipsodial gate is 
        V = np.pi * self.gating_size  # equ. 6.28 b
        return N / V

    def track_update(self, 
        t: float,
        dt: float, 
        z_k: List[float], 
        xs: List[float],
        robot_id: int,
        est_msg: EstimationMsg(),
    ):
        
        '''
        tracking the target with RangeBearingKF object, based on range-bearing model

        t: time of receiving the observations
        dt: time between this time and last time of receiving observations
        z_k: a list of position data, expressed as [[x1, y1], [x2, y2], ...]
        xs: sensor's state, [x,y,theta]

        FoV analysis, to classifiy obs data into 2 parts, inside FoV or outside
        '''
        
        self.t = t
        # adaptive motion model here
        if not self.IsStatic:

            self.F, self.Q, self.P = motion_model(self.IsStatic, dt, self.sensor_para_list[robot_id]["sigma_a"])

        next_index_list: List[int] = list()
        
        track_output: List[track] = list()
        # bb_output_k = []
        
        # 1. extract all obs inside elliposidual gates of initialized and confirmed tracks


        obs_matrix = np.zeros((len(self.track_list_next_index), len(z_k)))


        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            kf = self.track_list[k].kf
            # S = np.dot(np.dot(kf.H, kf.P_k_k_min), kf.H.T) + kf.R
            kf.predict(self.F, self.Q)         # Check TODO
            pred_z = np.dot(kf.H, kf.x_k_k_min)
            
            S = np.dot(np.dot(kf.H, kf.P_k_k_min), kf.H.T) + kf.cal_R([pred_z[0,0], pred_z[1,0]], xs)
            #### usage for plotting ellipse
            self.track_list[k].S = S
            self.track_list[k].pred_z = pred_z

            for j in range(len(z_k)):
                z = z_k[j]
                z_til = pred_z - np.matrix(z).T
                # S = np.dot(np.dot(kf.H, kf.P_k_k_min), kf.H.T) + kf.cal_R(z, xs)
                obs_matrix[i][j] = self.elliposidualGating(z_til, S)

        # create a mitrix to check the relationship between obs and measurements


        obs_sum = obs_matrix.sum(axis=0)
        # matrix operation, exclude all observations whose sum is 0, which means its not in any gate
        index = np.where(obs_sum > 0)[0]
        obs_outside_index = np.where(obs_sum == 0)[0]


        # 2. deal with observations inside all gates
        # i. initialize each observation, find out how many gates each observation is in

        obs_class = []
        # now in obs_class, each element is a class with its .track includes all track numbers from 0 - m_k
        # then we need to analyse the gate. First for jth obs z_j in its ith gate we need to calculate
        # g_ij = N(z_j - z_ij|0, S_ij) here z_ij and S_ij is the updated KF given observation z_j


        for i in range(len(z_k)):

            if i in index:
                try:
                    a_mea = measurement(z_k[i], i)
                except:
                    print(i, z_k)
                for j in np.where(obs_matrix[:, i] != 0)[0]:
                    # track is the track id for obs i, in obs_matrix the column value is not 0
                    track_id = self.track_list_next_index[j]
                    a_mea.inside_track(track_id)
                    temp_kf = copy.deepcopy(self.track_list[track_id].kf)
                    temp_kf.update(z_k[i], xs, self.sensor_qual_dict[robot_id])
                    var = multivariate_normal(mean=np.squeeze(temp_kf.z_bar.reshape(2).tolist()[0]), cov=temp_kf.S_k)
                    a_mea.g_ij.append(var.pdf(z_k[i]))
                obs_class.append(a_mea)
            else:
                obs_class.append([])

        # ii. for each gate/track, analysis if there exists observations joint different tracks, find out the relation between different tracks


        track_class: List[track] = list()
        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            a_track = self.track_list[k]   # pointer to track_list class
            a_track.get_measurement(np.where(obs_matrix[i] != 0)[0])
            a_track.measurement.append(-1)
            track_class.append(a_track)

        for i in range(len(track_class)):
            
            # Case 1. if there is no obs inside track gate, 
            # make Kalman's update x_k_k= x_k_k_min, P_k_k = P_k_k_min
            if track_class[i].deleted:
                continue
            
            if len(track_class[i].measurement) == 1:
                # there is only false alarm in observation, then only do prediction
                kf = track_class[i].kf
                #  apparently this is wrong, we need to make observation in Kalman update 
                #  x_k_k_min, then update the covariance.
                
                kf.x_k_k = copy.deepcopy(kf.x_k_k_min)
                kf.P_k_k = copy.deepcopy(kf.P_k_k_min)
                track_class[i].update(t, kf, False)

                # only keep the confirmed ones

                if not (track_class[i].deleted or track_class[i].abandoned):
                    if np.trace(track_class[i].kf.P_k_k[0:2, 0:2]) < self.common_P:
                        next_index_list.append(self.track_list_next_index[i])
                        for j in self.track_list_next_index:
                            isSimilar, id_to_delete = self.checkSimilarity(j, self.track_list_next_index[i], robot_id)
                            if isSimilar:
                                try:
                                    next_index_list.remove(id_to_delete)
                                    # also remove the appended track_output
                                    if id_to_delete == j:
                                        for mmm in range(len(track_output)):
                                            if track_output[mmm].id == j:
                                                track_output.pop(mmm)
                                                # bb_output_k.pop(mmm)
                                except:
                                    pass
                    else:
                        print("deleted because of the trace, tr = %s, com_P = %s" % (np.trace(track_class[i].kf.P_k_k[0:2, 0:2]), self.common_P))
                        track_class[i].deletion(t)

                if track_class[i].confirmed and not track_class[i].deleted:
                    
                    track_output.append(track_class[i])
                    
                    # bb_output_k.append(track_class[i].bb_box_size)
                
                continue
            
            # Case 2. there is obs inside the track gate
            # calculate the beta for ith track
            # need the number of measurements inside the gate
            beta = self.cal_beta(len(track_class[i].measurement) - 1)

            
            table_key = [self.track_list_next_index[i]]

            
            # begin find all observations inside this gate that is related to other gates (joint)
            
            for obs_id in track_class[i].measurement:
                if obs_id != -1:
                    obs = obs_class[obs_id]
                    table_key += obs.track
                
            table_key = list(set(table_key))
            
            # for each track, figure out how many observations inside the track
            
            # inverse the table
            table_key_inv = []
            for j in table_key:
                table_key_inv.append(self.track_list_next_index.index(j))
            
            table_key_matrix = obs_matrix[table_key_inv]
            
            ######################
            
            #  there are overlaps of tracks, we only remain the one has oldest history
            # TODO if the overlaps slips, we can use tracking history to split them?
            if (table_key_matrix == table_key_matrix[0]).all() and len(table_key_matrix) > 2:
                # there are overlaps of tracks, we only remain the one has oldest history
                seed = min(table_key)
                for key in table_key:
                    if key != seed:
                        print("delete because of overlap")
                        self.track_list[key].deletion(t)
            
            ##### !!!!!!!!
            #         in order to avoid multiple tracks tracking same target, we need to analysis the
            #         obs_matrix
            
            obs_num_tracks = obs_matrix.sum(axis=1)[table_key_inv]

            # number of joint tracks
            N_T = len(table_key)
            # number of observations total
            total_obs = []
            for track_id in table_key:
                a_track = self.track_list[track_id]
                total_obs +=a_track.measurement

            total_obs = list(set(total_obs))
            N_o = len(total_obs) - 1

            common_factor = common_fact(beta, self.P_D, N_T, N_o)

            # iii. after merged all related tracks, we generat a hypothesis matrix/table based on the table 
            # generated by the table_key
            obs_num_tracks_ = obs_num_tracks + 1
            total_row = int(obs_num_tracks_.prod())

            # create title for the table


            hyp_matrix = {}
            for a_key in table_key:
                hyp_matrix[str(a_key)] = []
            hyp_matrix["p"] = []
            
            for row_num in range(total_row):
                key_num = len(table_key)
                col_num = 0
                # build one row of hypothesis
                while key_num > 0:
                    if col_num == len(table_key) - 1:
                        obs_id = int(row_num)
                        product = 1
                    else:
                        product = obs_num_tracks_[(col_num + 1):].prod()
                        obs_id = int(row_num // product)
                    
                    
                    value = self.track_list[table_key[col_num]].measurement[obs_id]
                    
                    key = str(table_key[col_num])
                    hyp_matrix[key].append(value)
                    row_num = row_num % product
                    col_num += 1
                    key_num -= 1
                
                # now we want to calculate the probability of this row's hypothesis
                hyp_list = []

                prob = common_factor
                for key in hyp_matrix.keys():
                    if key != 'p':
                        hyp_list.append(hyp_matrix[key][-1])
                
                # print('hyp_list, ', hyp_list)
                # calculate the prob of this hypothesis
                if checkIfDuplicates(hyp_list):
                    # this is not one vaild hypothesis.
                    prob = 0
                else:
                    # this is the valid hypothesis, we should calculate the prob
                    for key in hyp_matrix.keys():
                        if key != 'p':
                            track_id = int(key)
                            obs_id = hyp_matrix[key][-1]
                            # print('obs id ', obs_id)
                            if obs_id == -1:
                                prob *= (1 - self.P_D) * beta
                            else:
                                # print(obs_class[obs_id].table, print(obs_class[obs_id].id))
                                index = obs_class[obs_id].track.index(track_id)
                                prob *= self.P_D * obs_class[obs_id].g_ij[index]
                hyp_matrix['p'].append(prob)
                
            # iv. Then gather the prob in this track, and update the kF
            obs_in_i_track = track_class[i].measurement
            obs_in_i_track_prob = []
            hyp_in_i_track = np.array(hyp_matrix[str(self.track_list_next_index[i])])
            hyp_in_i_track_prob = np.array(hyp_matrix['p'])
            for obs in obs_in_i_track:
                index_ = np.where(hyp_in_i_track == obs)
                w_ij_list = hyp_in_i_track_prob[index_]
                obs_in_i_track_prob.append(w_ij_list.sum())
            
            # then normalize all w_ij s
            obs_in_i_track_prob_norm = normalize(obs_in_i_track_prob)

            # well, we then just need to update the KF of ith track
            kf = track_class[i].kf
            x_k = []
            P_k = []
            for obs in obs_in_i_track:
                if obs == -1:
                    x_k.append(kf.x_k_k)
                    P_k.append(kf.P_k_k)
                else:
                    
                    z = np.array(z_k[obs]).T

                    # update the kf
                    temp_kf = copy.deepcopy(kf)

                    temp_kf.update(z, xs, self.sensor_qual_dict[robot_id])
                    
                    x_k.append(temp_kf.x_k_k)
                    P_k.append(temp_kf.P_k_k)
            
            x_k_pda = 0 * temp_kf.x_k_k
            P_k_pda = 0 * temp_kf.P_k_k
            for j in range(len(obs_in_i_track_prob_norm)):
                x_k_pda += obs_in_i_track_prob_norm[j] * x_k[j]
            
            for j in range(len(obs_in_i_track_prob_norm)):
                P_k_pda += obs_in_i_track_prob_norm[j] * (P_k[j] + np.dot(x_k_pda - x_k[j], x_k_pda.T - x_k[j].T))
                
            # update this to kf and fused bb box
            kf.x_k_k = x_k_pda
            kf.P_k_k = P_k_pda
            
            
            if np.trace(kf.P_k_k[0:2, 0:2]) > self.common_P:
                # print("track get deleted here~~")
                track_class[i].update(t, kf, False)
                track_class[i].deleted = True
                
                continue
            
            track_class[i].update(t, kf, True)
            
            # save the activated ones for next recursion
            if not (track_class[i].deleted or track_class[i].abandoned):
                
                next_index_list.append(self.track_list_next_index[i])
                for j in self.track_list_next_index:
                    isSimilar, id_to_delete = self.checkSimilarity(j, self.track_list_next_index[i], robot_id)
                    if isSimilar:
                        try:
                            next_index_list.remove(id_to_delete)
                            # also remove the appended track_output
                            if id_to_delete == j:
                                for mmm in range(len(track_output)):
                                    if track_output[mmm].id == j:
                                        track_output.pop(mmm)
                                        # bb_output_k.pop(mmm)
                        except:
                            pass
                        
            # only keep the confirmed ones
            if track_class[i].confirmed and not track_class[i].deleted:
                
                track_output.append(track_class[i])
                # pick the bounding box which is the cloest to the associated x_k_k
                # bb_output_k.append(bb_k[self.find_cloest(x_k_pda, x_k[1:])])
                # track_class[i].bb_box_size = bb_output_k[-1]
                

        # 3. deal with observations outside all gates
            
        # now initialize all observations outside of the gate
        for i in obs_outside_index:
            z = z_k[i] + [0, 0]
            x0 = np.matrix(z).T
            kf = RangeBearingKF(self.F, self.H, x0, self.P, self.Q, copy.deepcopy(self.R_dict[robot_id]), \
                            self.sensor_para_list[robot_id]["r0"], quality=self.sensor_qual_dict[robot_id])
            id_ = len(self.track_list)
            new_track = track(t, id_, kf, self.DeletionThreshold, self.ConfirmationThreshold)
            # new_track.kf.predict(self.F, self.Q)
            self.track_list.append(new_track)
            next_index_list.append(id_)

        #     swap the track list to the new list.
        self.track_list_next_index = next_index_list

        for track_ in track_output:
            traj_ = PoseEstimation()
            traj_.pose.x = track_.kf.x_k_k[0, 0]
            traj_.pose.y = track_.kf.x_k_k[1, 0]
            vx = track_.kf.x_k_k[2, 0]
            vy = track_.kf.x_k_k[3, 0]
            traj_.pose.yaw = np.arctan2(vy, vx)
            # print(vx, vy)
            traj_.velocity.vx = vx
            traj_.velocity.vy = vy
            traj_.detection_label = self.label
            traj_.trajectory_id = track_.id
            traj_.covariance = track_.kf.P_k_k.flatten().tolist()[0]
            est_msg.pose_estimations.append(traj_)

        # return track_output
