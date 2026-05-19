import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

#primary drone settings
main_drone = {
    "speed": 3.0,
    "start_time": 0.0,
    "max_time": 100.0,
    "id": "Primary",
    "waypoints": [
        {"x": 0.0, "y": 0.0, "z": 0.0}, 
        {"x": 10.0, "y": 10.0, "z": 10.0},
        {"x": 20.0, "y": 20.0, "z": 5.0}
    ]
}

# Background Drones Settings

# conflict free
# Secondary_drone = [
#     {
#         "id": "Sim_1",
#         "speed": 1.0,
#         "start_time": 0.0,
#         "waypoints": [
#             {"x": 0.0, "y": 2.0, "z": 3.0},
#             {"x": 4.0, "y": 6.0, "z": 7.0}
#         ]
#     }

#     ,{
#         "id": "Sim_2",
#         "speed": 5.0,
#         "start_time": 2.0,
#         "waypoints": [
#             {"x": 3.0, "y": 9.0, "z": 3.0},
#             {"x": 10.0, "y": 0.0, "z": 0.0}
#         ]
#     }
# ]


#with conflict

Secondary_drone = [
    {
        "id": "Sim_1",
        "speed": 3.0,
        "start_time": 0.0,
        "waypoints": [
            {"x": 10.0, "y": 10.0, "z": 10.0},
            {"x": 0.0, "y": 0.0, "z": 0.0}
        ]
    }

    ,{
        "id": "Sim_2",
        "speed": 5.0,
        "start_time": 2.0,
        "waypoints": [
            {"x": 3.0, "y": 9.0, "z": 3.0},
            {"x": 10.0, "y": 0.0, "z": 0.0}
        ]
    }
]

#chnage this value to increase or decrease the safety distance between drones.
safety_distance = 1.0

#eucledean distance 
def dist(w1, w2):
    return math.sqrt(  (w1["x"] - w2["x"])**2 
                     + (w1["y"] - w2["y"])**2 
                     + (w1["z"] - w2["z"])**2)
# divide path in segments and calculate velocity for each segment in xyz

def make_segments(waypoints, speed, start_time):
    segments = []
    current_time = start_time
    for i in range(len(waypoints) - 1):
        w1 = waypoints[i]
        w2 = waypoints[i+1]
        d = dist(w1, w2)
        if d == 0:
            continue
            
        time_taken = d / speed
        end_time = current_time + time_taken
        
        vx = (w2["x"] - w1["x"]) / time_taken
        vy = (w2["y"] - w1["y"]) / time_taken
        vz = (w2["z"] - w1["z"]) / time_taken
        
        segments.append({
            "start_wp": w1,
            "end_wp": w2,
            "start_time": current_time,
            "end_time": end_time,
            "vx": vx,
            "vy": vy,
            "vz": vz
        })
        current_time = end_time
    return segments

#get position at time t for a segment

def get_pos(segment, t):
    dt = t - segment["start_time"]
    return {
        "x": segment["start_wp"]["x"] + segment["vx"] * dt,
        "y": segment["start_wp"]["y"] + segment["vy"] * dt,
        "z": segment["start_wp"]["z"] + segment["vz"] * dt
    }
#check overlap or not between two segments and return the time of conflict if any
def check_overlap(seg1, seg2, radius):
    start = max(seg1["start_time"], seg2["start_time"])
    end = min(seg1["end_time"], seg2["end_time"])
    
    if start > end:
        return []
        
    dvx = seg1["vx"] - seg2["vx"]                   
    dvy = seg1["vy"] - seg2["vy"]
    dvz = seg1["vz"] - seg2["vz"]
    
    p1 = get_pos(seg1, start)
    p2 = get_pos(seg2, start)
    dx = p1["x"] - p2["x"]
    dy = p1["y"] - p2["y"]
    dz = p1["z"] - p2["z"]
    
    A = dvx**2 + dvy**2 + dvz**2
    B = 2 * (dx * dvx + dy * dvy + dz * dvz)
    C = (dx**2 + dy**2 + dz**2) - radius**2
    
    conflicts = []
    
    if A == 0:
        if C <= 0:
            conflicts.append(start)
        return conflicts
        
    discriminant = B**2 - 4 * A * C
    if discriminant < 0:
        return conflicts
        
    t1 = (-B - discriminant**0.5) / (2 * A)
    t2 = (-B + discriminant**0.5) / (2 * A)
    
    t_abs1 = start + t1
    t_abs2 = start + t2
    
    c_start = max(start, t_abs1)
    c_end = min(end, t_abs2)
    
    if c_start <= c_end:
        conflicts.append(c_start)
        
    return conflicts
#literally runs checks XD and vizualise
def run_checks():
    print("Checking conflicts...\\n")
    p_segs = make_segments(main_drone["waypoints"], main_drone["speed"], main_drone["start_time"])
    
    if not p_segs:
        print("No waypoints found")
        return
        
    if p_segs[-1]["end_time"] > main_drone["max_time"]:
        print(" error: Time exceeded!")
        
    all_bad = []
    
    for drone in Secondary_drone:
        d_segs = make_segments(drone["waypoints"], drone["speed"], drone["start_time"])
        for p in p_segs:
            for d in d_segs:
                times = check_overlap(p, d, safety_distance)
                for t in times:
                    loc = get_pos(p, t)
                    all_bad.append({
                        "id": drone["id"],
                        "time": t,
                        "loc": loc
                    })
                    
    if all_bad:
        print("WARNING : CONFLICTS FOUND")
        for b in all_bad:
            print("Crash with", b["id"], "at time", round(b["time"], 2), "seconds.")
            print("Location - X:", round(b["loc"]["x"], 2), "Y:", round(b["loc"]["y"], 2), "Z:", round(b["loc"]["z"], 2))
    else:
        print("Path is clear")
        
    
    min_t = main_drone["start_time"]
    max_t = p_segs[-1]["end_time"]
    for drone in Secondary_drone:
        d_segs = make_segments(drone["waypoints"], drone["speed"], drone["start_time"])
        if d_segs:
            min_t = min(min_t, d_segs[0]["start_time"])
            max_t = max(max_t, d_segs[-1]["end_time"])
            
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Drone Path Visualization')
    
    px = [w["x"] for w in main_drone["waypoints"]]
    py = [w["y"] for w in main_drone["waypoints"]]
    pz = [w["z"] for w in main_drone["waypoints"]]
    ax.plot(px, py, pz, 'b--', alpha=0.5, label='Primary Drone')
    
    for drone in Secondary_drone:
        sx = [w["x"] for w in drone["waypoints"]]
        sy = [w["y"] for w in drone["waypoints"]]
        sz = [w["z"] for w in drone["waypoints"]]
        ax.plot(sx, sy, sz, 'r--', alpha=0.5)
        
    p_dot, = ax.plot([], [], [], 'bo', markersize=8)
    s_dots = {}
    for d in Secondary_drone:
        s_dots[d["id"]] = ax.plot([], [], [], 'ro', markersize=6)[0]
        
    c_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, color='red', fontsize=12)
    t_text = ax.text2D(0.05, 0.90, '', transform=ax.transAxes)
    
    fps = 30
    dur = max(max_t - min_t, 1.0)
    frames = int(dur * fps) + 30
    times = np.linspace(min_t, max_t, frames)
    
    all_d_segs = {}
    for drone in Secondary_drone:
        all_d_segs[drone["id"]] = make_segments(drone["waypoints"], drone["speed"], drone["start_time"])
        
    def find_pos(segs, t):
        if not segs: return None
        for s in segs:
            if s["start_time"] - 1e-5 <= t <= s["end_time"] + 1e-5:
                tc = max(s["start_time"], min(t, s["end_time"]))
                return get_pos(s, tc)
        return None
        
    def update(frame):
        t = times[frame] if frame < len(times) else max_t
        t_text.set_text('Time: ' + str(round(t, 2)) + ' s')
        
        pp = find_pos(p_segs, t)
        if pp:
            p_dot.set_data([pp["x"]], [pp["y"]])
            p_dot.set_3d_properties([pp["z"]])
        else:
            p_dot.set_data([], [])
            p_dot.set_3d_properties([])
            
        is_bad = False
        for d in Secondary_drone:
            dp = find_pos(all_d_segs[d["id"]], t)
            dot = s_dots[d["id"]]
            if dp:
                dot.set_data([dp["x"]], [dp["y"]])
                dot.set_3d_properties([dp["z"]])
                if pp:
                    distance = dist(pp, dp)
                    if distance <= safety_distance:
                        is_bad = True
                        dot.set_color('yellow')
                        dot.set_markersize(12)
                        p_dot.set_color('yellow')
                        p_dot.set_markersize(12)
                    else:
                        dot.set_color('red')
                        dot.set_markersize(6)
            else:
                dot.set_data([], [])
                dot.set_3d_properties([])
                
        if is_bad:
            c_text.set_text('CRASHING!')
        else:
            c_text.set_text('')
            p_dot.set_color('blue')
            p_dot.set_markersize(8)
            
        ans = [p_dot, c_text, t_text]
        for v in s_dots.values():
            ans.append(v)
        return ans

    ani = animation.FuncAnimation(fig, update, frames=frames, blit=False, interval=1000/fps, repeat=True)
    plt.show(block=True)
    plt.close(fig)

if __name__ == "__main__":
    run_checks()
