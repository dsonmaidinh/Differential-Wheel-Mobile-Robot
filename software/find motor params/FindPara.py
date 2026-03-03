import numpy as np

# Đường dẫn file của bạn
vel1_path = r"D:\HOC\V\VSCODE\Du an\Differential Mobile Robot\FindPara\Vel1.txt"
vel_path = r"D:\HOC\V\VSCODE\Du an\Differential Mobile Robot\FindPara\Vel.txt"

Umax = 1000
tauc = 0.03

def calculate_pid(tau, K_val):
    """Giữ nguyên công thức tính PID của bạn"""
    Kp_pos = ((2 * tauc + tau) / (tauc ** 2)) / K_val
    Ki_pos = Kp_pos / (2 * tauc + tau)
    Kd_pos = Kp_pos * ((2 * tauc * tau) / (2 * tauc + tau))
    Kb_pos = np.sqrt(Ki_pos / Kd_pos)

    Kp = tau / (tauc * K_val)
    Ki = Kp / tau
    Kb = Ki / Kp
    return Kp_pos, Ki_pos, Kd_pos, Kb_pos, Kp, Ki, Kb

def process_data(file_path):
    """Hàm đọc dữ liệu từ file và trả về mảng numpy [time, motorL, motorR]"""
    data = []
    with open(file_path, 'r') as fid:
        for line in fid:
            line = line.strip()
            if line.startswith('D:') and not any(x in line for x in ['Motor', 'time']):
                p = line.split('\t')
                if len(p) >= 4: # Format mới: D: time MotorL MotorR
                    try:
                        t = float(p[1])
                        vL = float(p[2])
                        vR = float(p[3])
                        data.append([t, vL, vR])
                    except ValueError:
                        continue
    return np.array(data)

# ====== B1: Tính wmax_avg từ Vel1.txt ======
data1 = process_data(vel1_path)
# Lấy 10% dữ liệu cuối để tính trung bình
N = int(len(data1) * 0.1)
wmax_avg_L = np.mean(data1[-N:, 1])
wmax_avg_R = np.mean(data1[-N:, 2])

print(f"--- Vel1.txt Analysis ---")
print(f"L_wmax_avg: {wmax_avg_L:.4f} | R_wmax_avg: {wmax_avg_R:.4f}\n")

# ====== B2 & B3: Tính Tau và PID từ Vel.txt ======
data = process_data(vel_path)
time = data[:, 0] / 1000 # Convert ms to s
motors = {'Left': (data[:, 1], wmax_avg_L), 'Right': (data[:, 2], wmax_avg_R)}

for side, (w_vals, wmax_avg) in motors.items():
    # Tìm t1 (bắt đầu tăng)
    idx1 = np.where(w_vals > 0)[0]
    t1 = time[idx1[0]] if len(idx1) > 0 else 0
    
    # Tìm t2 (đạt 63.2%)
    target = 0.632 * wmax_avg
    idx2 = np.where(w_vals >= target)[0]
    t2 = time[idx2[0]] if len(idx2) > 0 else time[-1]
    
    tau = t2 - t1
    K_val = wmax_avg / Umax
    
    # Tính toán PID
    Kp_pos, Ki_pos, Kd_pos, Kb_pos, Kp, Ki, Kb = calculate_pid(tau, K_val)
    
    print(f"--- Results for {side} Motor ---")
    print(f"tau: {tau:.4f}s, K: {K_val:.4f}")
    print(f"Position Loop: Kp={Kp_pos:.4f}, Ki={Ki_pos:.4f}, Kd={Kd_pos:.4f}, Kb={Kb_pos:.4f}")
    print(f"Velocity Loop: Kp={Kp:.4f}, Ki={Ki:.4f}, Kb={Kb:.4f}")
    print("-" * 30)