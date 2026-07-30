import math

def rolling_avg(pts, size=6): #grap rolling avgs
    if not pts or size <= 0: #invid
        return []

    result = [] #output
    half = size // 2 #too either side

    for i in range(len(pts)): #for each point
        start = max(0, i - half) #begin ind
        end = min(len(pts), i + half + 1) #end ind

        window = pts[start:end] #wind

        avg_x = sum(pt[0] for pt in window) / len(window) #avg
        avg_y = sum(pt[1] for pt in window) / len(window) #avg

        result.append((avg_x, avg_y)) #avg point

    return result

def farthest_gap(lidar, min_gap_size = 20): #farthest
    smp = lidar.get_samples() #get
    n = lidar.get_num_samples() #get

    pts = [] #cartesian

    def to_angle(i):
        return (i / n) * (2 * math.pi)

    def to_cart(ray, ang):
        return (ray * math.sin(ang), ray * math.cos(ang))
    
    for i in range(-n//4, n//4): #for i in range
        if smp[i] != 0:
            ang = to_angle(i) #convert to angle
            pts.append(to_cart(smp[i], ang)) #convert to caretsian

    pts = rolling_avg(pts, 6) #rolling average to eliminate/reduce noise
    
    max_dist = 0 #initialize zero
    max_i = (0, 0) #window of zero

    def dist(p1, p2): #cart distance
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx*dx + dy*dy)

    for i in range(1, len(pts)): #for each point
        d = max(dist(pts[i], (0, 0)), dist(pts[i - 1], (0, 0))) #get d = distance
        if d > max_dist and dist(pts[i], pts[i - 1]) > min_gap_size: #if > distance and current window is valid
            max_dist = d #set
            max_i = (i - 1, i) #set

    return pts[max_i[0]], pts[max_i[1]] #return mid pts

def magnitude(pt): #magnitude of poitn aka dist
    return math.sqrt(pt[0]*pt[0] + pt[1]*pt[1])

def weight_function(d1, d2): #variable weihg tufnciton, currenlty weighted simply by prooprtion
    return (1 - d1 / (d1 + d2)), (1 - d2 / (d1 + d2))

def add_pts(pt1, pt2): #sum of two points
    return (pt1[0] + pt2[0], pt1[1] + pt2[1])

def multiply_pt(pt, sc): #scale point by scalar
    return (pt[0] * sc, pt[1] * sc)

def weighted_point(pt1, pt2): #get weighted poin tof two points
    d1 = magnitude(pt1)
    d2 = magnitude(pt2)

    d1, d2 = weight_function(d1, d2)

    return add_pts(multiply_pt(pt1, d1), multiply_pt(pt2, d2))

def tar_ang(smp, n, window, check_window=14, car_size=45):
    pt = weighted_point(window[0], window[1])

    ang = math.atan2(pt[0], pt[1])

    ang = int(ang * n / (math.pi * 2)) #convert to lidar smaples
    
    min_sample = 0
    if ang + check_window > n: #wraparound handle
        rng = range(ang - check_window, n) + range(0, ang + check_window - n) #for each in window
    else:
        rng = range(ang - check_window, ang + check_window)

    for i in rng:
        if smp[i] != 0 and smp[i] < smp[min_sample]: #if has a closer sample (i.e. wall nearby)
            min_sample = i

    def magnitude(pt):
        return math.sqrt(pt[0]*pt[0] + pt[1]*pt[1])

    if min_sample != 0 and smp[min_sample] < magnitude(pt) and smp[min_sample] != 0:
        ang = min_sample * 2 * math.pi/n
        pt = (smp[min_sample] * math.sin(ang), smp[min_sample] * math.cos(ang))
        if pt[0] > 0: 
            pt = (pt[0] - car_size, pt[1]) #set target to offset point to avoid crashing
        else:
            pt = (pt[0] + car_size, pt[1])

    #print(f"{pt} : {math.atan2(pt[0], pt[1])}")

    return math.atan2(pt[0], pt[1]) #angle, reversed because 0 is forwards rather than sideways and is cw instaed of ccw
    
def angle_to(window):
    pt = ((window[0][0] + window[1][0])/2, (window[0][1] + window[1][1])/2) #helper for angle to pt
    return math.atan2(pt[0], pt[1])
