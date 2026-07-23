import math
import EKF
import SLAMICP
import occupancy_grid
import AStarAlgorithm
KP = 0.0
KD = 0.0
global lastError
lastError = 0
global pose
pose = [0,0,0]
ekf = EKF()
rc = None
oc = occupancy_grid(20,20,1010101,123,123)
class Follower:
    def __init__(self, kP, kD, startPose, rc):
        KP = kP
        KD = kD
        pose = startPose
        self.rc = rc
    def follow_path(path, dt):
        global lastError
        min_dist = 0
        idx = 0
        tar_idx = 0
        for point in path:
            x1 = pose[0]
            y1 = pose[1]
            x2 = point[0]
            y2 = point[1]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if dist < min_dist:
                min_dist = dist
                tar_idx = idx
            idx += 1
        tnx_point = path[tar_idx]
        dx = pose[0] - tnx_point[0]
        dy = pose[1] - tnx_point[1]
        theta = math.atan2(dy, dx)
        error = pose[2] - theta
        #speed controller?
        speed = error * 0.6
        angle = KP * error + KD * ((error - lastError) / dt)
        lastError = error
        return speed, angle
    def update(goalPose):
        global pose
        global oc
        #calls ICP and EKF
        SLAMICP.update()
        ICPpose = SLAMICP.get_pose()
        #implement : pose w IMU
        IMUPose = [0,0,0]
        # how tf do i use the EKF sixian
        FusedPose = [0,0,0]
        pose = FusedPose
        dt = rc.get_delta_time()
        pc = rc.lidar.get_samples()
        oc.update_grid(pc, pose)
    def get_pose():
        global pose
        return pose    
        
        
        








