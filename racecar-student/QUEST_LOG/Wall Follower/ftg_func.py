import math
def largest_gap(lidar):
    smp = lidar.get_samples()
    n = lidar.get_num_samples()

    pts = []

    for i in range(-n//4, n//4):
        if smp[i] != 0:
            ang = (i / n) * (2 * math.pi) #convert to angle
            pts.append((smp[i] * math.sin(ang), smp[i] * math.cos(ang))) #convert to caretsian

    pts = rolling_avg(pts)

    max_dist = 0
    max_i = (0, 0)

    def dist(p1, p2): #cart dist
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx*dx + dy*dy)

    for i in range(1, len(pts)):
        d = dist(pts[i], pts[i - 1]) 
        if d > max_dist:
            max_dist = d
            max_i = (i - 1, i)
    if len(pts) == 0:
        return 0,0
    else:
        return pts[max_i[0]], pts[max_i[1]]

def rolling_avg(pts, size=5):
    if not pts or size <= 0:
        return []

    result = []
    half = size // 2

    for i in range(len(pts)):
        start = max(0, i - half)
        end = min(len(pts), i + half + 1)

        window = pts[start:end]

        avg_x = sum(pt[0] for pt in window) / len(window)
        avg_y = sum(pt[1] for pt in window) / len(window)

        result.append((avg_x, avg_y))

    return result

def farthest_gap(lidar, min_gap_size = 20):
    smp = lidar.get_samples()
    n = lidar.get_num_samples()

    pts = []

    for i in range(-n//4, n//4):
        if smp[i] != 0:
            ang = (i / n) * (2 * math.pi) #convert to angle
            pts.append((smp[i] * math.sin(ang), smp[i] * math.cos(ang))) #convert to caretsian

    pts = rolling_avg(pts, 6)
    
    max_dist = 0
    max_i = (0, 0)

    def dist(p1, p2): #cart dist
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx*dx + dy*dy)

    for i in range(1, len(pts)):
        d = max(dist(pts[i], (0, 0)), dist(pts[i - 1], (0, 0)))
        if d > max_dist and dist(pts[i], pts[i - 1]) > min_gap_size:
            max_dist = d
            max_i = (i - 1, i)

    return pts[max_i[0]], pts[max_i[1]]

def magnitude(pt):
    return math.sqrt(pt[0]*pt[0] + pt[1]*pt[1])

def weight_function(d1, d2):
    return (1 - d1 / (d1 + d2)), (1 - d2 / (d1 + d2))

def add_pts(pt1, pt2):
    return (pt1[0] + pt2[0], pt1[1] + pt2[1])

def multiply_pt(pt, sc):
    return (pt[0] * sc, pt[1] * sc)

def weighted_point(pt1, pt2):
    d1 = magnitude(pt1)
    d2 = magnitude(pt2)

    d1, d2 = weight_function(d1, d2)

    return add_pts(multiply_pt(pt1, d1), multiply_pt(pt2, d2))

def tar_ang(smp, n, window, check_window=8, car_size=25):
    pt = weighted_point(window[0], window[1])

    ang = math.atan2(pt[0], pt[1])

    ang = int(ang * n / (math.pi * 2)) #convert to lidar smaples
    
    min_sample = 0
    if ang + check_window > n:
        rng = range(ang - check_window, n) + range(0, ang + check_window - n)
    else:
        rng = range(ang - check_window, ang + check_window)

    for i in rng:
        if smp[i] < smp[min_sample]:
            min_sample = i

    def magnitude(pt):
        return math.sqrt(pt[0]*pt[0] + pt[1]*pt[1])

    if min_sample != 0 and smp[min_sample] < magnitude(pt) and smp[min_sample] != 0:
        ang = min_sample * 2 * math.pi/n
        pt = (smp[min_sample] * math.sin(ang), smp[min_sample] * math.cos(ang))
        if pt[0] > 0: 
            pt = (pt[0] - car_size, pt[1])
        else:
            pt = (pt[0] + car_size, pt[1])

    #print(f"{pt} : {math.atan2(pt[0], pt[1])}")

    return math.atan2(pt[0], pt[1])
    
def angle_to(window):
    pt = ((window[0][0] + window[1][0])/2, (window[0][1] + window[1][1])/2)
    return math.atan2(pt[0], pt[1])
