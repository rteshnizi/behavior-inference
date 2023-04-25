import numpy as np
from numpy.linalg import inv
import copy
from typing import List, Tuple
from sa_trajectory_estimation.util.sensor_model import range_bearing_cov

class LinearKF:
    ''' a standard Linear KF'''
    def __init__(
        self, 
        _F: np.ndarray,
        _H: np.ndarray,
        _x: np.ndarray,
        _P: np.ndarray,
        _Q: np.ndarray,
        _R: np.ndarray, 
        q0: float = 1.0
    ):
        
        self.F=_F
        self.H=_H
        
        self.x_k_k_min=_x
        self.P_k_k_min=_P
        self.Q=_Q
        self.R=_R
        
        self.x_k_k=_x
        self.P_k_k=_P
        
        self.x_dim = _x.shape[0]
        self.z_dim = _H.shape[1]
        
        self.S_k = self.R

        # for Monte Carlo, it needs a feature 
        self.isUpdated = False
        self.Q0 = np.diag([q0, q0, q0, q0])
        self.init = True
    
    def getCurrentState(self) -> np.ndarray:
        return self.x_k_k_min
    
    def getEst(self) -> np.ndarray:
        return np.dot(self.H, self.x_k_k)
    
    def predict(self, F: np.ndarray, Q: np.ndarray):
        # KF prediction step 
        self.x_k_k_min = np.dot(F, self.x_k_k)
        if not self.init:
            self.P_k_k_min = np.dot(F, np.dot(self.P_k_k, F.T)) + Q
        else:
            self.P_k_k_min = np.dot(self.F, np.dot(self.P_k_k,self.F.T)) + self.Q0
            self.init = False
        
    def update(self, z: List[float], r: float =-1,):
        # posterior update in KF
        if r > 0:
            self.R = np.diag([r, r])
        z = np.matrix(z).T

        self.z_bar = np.dot(self.H, self.x_k_k_min)
        self.z_res = z - self.z_bar
        self.S_k = np.dot(np.dot(self.H, self.P_k_k_min), self.H.T) + self.R
        # Cholesky's method for inverse
        # c = np.linalg.inv(np.linalg.cholesky(self.S_k))
        # inv_S = np.dot(c.T,c)
        inv_S = inv(self.S_k)
        K_k = np.dot(self.P_k_k_min, self.H.T) * inv_S
        self.x_k_k = self.x_k_k_min + np.dot(K_k, self.z_res)
        self.P_k_k = np.dot(np.eye(self.x_dim) - np.dot(K_k, self.H), self.P_k_k_min)

        self.isUpdated = True

class RangeBearingKF(LinearKF):
    '''KF with range-bearing sensor model'''
    def __init__(
        self, 
        _F: np.ndarray,
        _H: np.ndarray,
        _x: np.ndarray,
        _P: np.ndarray,
        _Q: np.ndarray,
        _R: np.ndarray, 
        r0: float, 
        quality: float = 1.0
    ):
        LinearKF.__init__(self, _F,_H,_x,_P,_Q,_R)
        # this alpha factor is a scalar for R matrix for sepcific sensor
        # which can be regarded as sensing quality
        self.alpha = quality
        self.base_R = copy.deepcopy(self.R)
        self.r0 = r0

    def cal_R(
        self, 
        z: List[float], 
        xs: List[float],
        alpha: float = -1.0,
    ) -> np.ndarray:
        
        if alpha < 0:
            alpha = self.alpha
        
        return range_bearing_cov(z, xs, alpha, self.r0)

    def update(self, z: List[float], xs: List[float], alpha: float):
        '''
        Input
        =====
        z = [x, y]: observation
        xs = [x, y, theta]: sensor state
        alpha: factor
        '''

        self.alpha = alpha  # TODO adapt centralized and remove this step
        self.R = self.cal_R(z, xs)
        z = np.matrix(z).T
        self.z_bar = np.dot(self.H, self.x_k_k_min)
        self.z_res = z - self.z_bar
        self.S_k = np.dot(np.dot(self.H, self.P_k_k_min), self.H.T) + self.R
        # Cholesky's method for inverse
        # c = np.linalg.inv(np.linalg.cholesky(self.S_k))
        # inv_S = np.dot(c.T,c)
        inv_S = inv(self.S_k)
        K_k = np.dot(self.P_k_k_min, self.H.T) * inv_S
        self.x_k_k = self.x_k_k_min + np.dot(K_k, self.z_res)
        self.P_k_k = np.dot(np.eye(self.x_dim) - np.dot(K_k, self.H), self.P_k_k_min)

        self.isUpdated = True

class measurement:
    '''a class for jpda joint hypothesis table'''
    def __init__(self, z: List[float], id_: int):
        self.value = z
        self.track: List[int] = list()
        self.g_ij = []
        self.table = {"track": self.track, "g_ij": self.g_ij}
        self.id = id_
        
    def inside_track(self, track_id: int):
        self.track.append(track_id)

class track:
    '''track should have the function of initilization, confirmation, deletion, save all tracks
        initalize -> confirm -> delete
           |
           V
        abandon
    '''
    def __init__(self, 
        t0: float, 
        id_: int, 
        kf: LinearKF, 
        DeletionThreshold: List[int], 
        ConfirmationThreshold: List[int],
        memory_len: int = 10,
        isForeign: bool = False
    ):
        self.id = id_
        self.measurement = []
        self.confirmed = False
        self.deleted = False     # if deleted after confirmed, this deleted is true
        self.kf = kf
        self.t0 = t0
        self.history = [1]
        self.record = {"points": [], "time_interval": [t0]}
        self.abandoned = False   # if not get confirmed, this abandoned is true 
        self.S = self.kf.S_k
        self.pred_z = self.kf.x_k_k
        self.ConfirmationThreshold = ConfirmationThreshold
        self.DeletionThreshold = DeletionThreshold
        self.bb_box_size = [10, 10] # default size for bounding box
        self.agent_id = -1
        self.neighbor = []
        self.memory = {}
        self.memory_len = memory_len
        self.isForeign = isForeign
        self._check_confirm()
        
    def track_memory(self, agent_id: int, sub_id: int):
        # accumulate memory with fiex horizon
        # TODO
        # Q: if we need to save memory for non-matching siuation
        # A: I don't like that.... that's just trival memories
        try:
            if len(self.memory[agent_id]) == self.memory_len:
                self.memory[agent_id].pop(0)
                self.memory[agent_id].append(sub_id)
            else:
                self.memory[agent_id].append(sub_id)
        except:
            self.memory[agent_id] = [sub_id]

    def update(self, t: float, kf: LinearKF, isObs: bool):
        
        self.kf = kf
        # self.kf.predict()
        
        self.record["points"].append(self.kf.getEst())
        
        if self.confirmed:
            # check if there is observation inside the gate
            if isObs:
                self.history.append(1)
                
            else:
                self.history.append(0)
            
            
            if len(self.history) > self.DeletionThreshold[1]:
                self.history.pop(0)
                    
            if len(self.history) == self.DeletionThreshold[1] and sum(self.history) < self.DeletionThreshold[1] - self.DeletionThreshold[0]:
                # reaches the threshold to delete: self.DeletionThreshold[0] is number of missings allowed
                self.deletion(t)
        else:
            # check if there is observation inside the gate
            if isObs:
                self.history.append(1)
                
            else:
                self.history.append(0)
            
            self._check_confirm()
                
    
    def _check_confirm(self):
        # when enough observation is done, check the N/M
        if len(self.history) == self.ConfirmationThreshold[1]:
            if sum(self.history) < self.ConfirmationThreshold[0]:
                # reaches the threshold to abandon, ow, confirm
                self.abandoned = True
            else:
                self.confirmation()
                # print(self.record["points"], self.history, self.id)


    def confirmation(self):
        self.confirmed = True
        
    
    def deletion(self, t: float):
        self.deleted = True
        self.record["time_interval"] = [self.t0, t] 
    
    def get_measurement(self, a_list: np.ndarray):
        # here a_list is np.ndarray data type from np.where function
        self.measurement = a_list.tolist()
        

class vertex:

    def __init__(self, track_id: int):
        # self.agent_id = agent_id
        self.track_id = track_id
        self.neighbor = []
        self.memory = []

def motion_model(
    IsStatic: bool, 
    dt: float, 
    sigma_a: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if IsStatic:
        F = np.matrix(np.eye(4))
        Q = np.matrix(np.zeros((4,4)))
        P = np.matrix(np.diag([.25] * 4))
    else:
        # NCV dynamic model
        F = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        A = np.matrix([[0.5 * dt**2, 0],
                            [0, 0.5 * dt**2],
                            [dt, 0],
                            [0, dt]])
        # sigma_a = self.sensor_para_list[robot_id]["sigma_a"]
        Q = np.dot(np.dot(A, np.diag([sigma_a**2, sigma_a**2])), A.T)
        P = np.matrix(np.diag([0.1, 0.1, 0.01, 0.01]))
    return F, Q, P