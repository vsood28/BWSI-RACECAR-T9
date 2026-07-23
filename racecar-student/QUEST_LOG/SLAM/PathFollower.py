import math
KP = 0.0
KD = 0.0
class Follower:
    #add more packaged functions
    def follow_path(path, pose, lastError, dt):
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
        return speed, angle





