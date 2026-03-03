import numpy as np
import matplotlib.pyplot as plt
import math
from collections import deque

# ==========================================
# 1. THÔNG SỐ VẬT LÝ
# ==========================================
WHEEL_RADIUS    = 0.0344    # Bán kính bánh xe (m)
WHEEL_BASE      = 0.20      # Khoảng cách 2 bánh L (m)
KV              = 1.0       # Hệ số P vận tốc dài
KW              = 2.0       # Hệ số P vận tốc góc
MOTOR_MAX_OMEGA = 10.0      # Tốc độ motor tối đa (rad/s)
SAFETY_FACTOR   = 0.9

TRAJ_RADIUS = 2.0           # Bán kính quỹ đạo tròn (m)
TRAJ_OMEGA  = 0.1           # Tốc độ góc bám quỹ đạo (rad/s)

# ==========================================
# 2. THÔNG SỐ NHIỄU — THỰC TẾ
# ==========================================
ENCODER_NOISE_STD   = 0.15   # Nhiễu encoder (rad/s)
IMU_THETA_NOISE_STD = 0.05  # Nhiễu trắng theta IMU (rad)
IMU_BIAS_TRUE       = 0.05  # Bias offset cố định của IMU (rad)

# ==========================================
# 3. THÔNG SỐ EKF
# ==========================================
dt_nom = 0.01
Q_x     = 2 * (ENCODER_NOISE_STD * WHEEL_RADIUS / 2 * dt_nom) ** 2
Q_y     = Q_x
Q_theta = 2 * (ENCODER_NOISE_STD * WHEEL_RADIUS / WHEEL_BASE * dt_nom) ** 2
Q_bias  = 1e-8  
Q_MATRIX = np.diag([Q_x, Q_y, Q_theta, Q_bias])

R_COV = IMU_THETA_NOISE_STD ** 2
# R_COV = IMU_THETA_NOISE_STD ** 2

P0 = np.diag([0.5, 0.5, 0.5, 1.0])

# ==========================================
# 4. HÀM TIỆN ÍCH
# ==========================================
def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def gen_trajectory(t):
    angle = TRAJ_OMEGA * t
    return TRAJ_RADIUS * math.cos(angle), TRAJ_RADIUS * math.sin(angle)

def calc_setpoint(ep, et):
    v = KV * ep
    w = KW * et
    wR = (v + w * WHEEL_BASE / 2) / WHEEL_RADIUS
    wL = (v - w * WHEEL_BASE / 2) / WHEEL_RADIUS
    m  = max(abs(wR), abs(wL))
    ma = MOTOR_MAX_OMEGA * SAFETY_FACTOR
    if m > ma:
        sf = ma / m; v *= sf; w *= sf
    return max(v, 0.0), w

def inverse_kin(v, w):
    vl = v / WHEEL_RADIUS
    wa = w * WHEEL_BASE / (2 * WHEEL_RADIUS)
    return vl - wa, vl + wa

def forward_kin(Px, Py, th, wL, wR, dt):
    v = WHEEL_RADIUS * (wR + wL) * 0.5
    w = WHEEL_RADIUS * (wR - wL) / WHEEL_BASE
    if abs(w) < 1e-9:
        Px += v * math.cos(th) * dt
        Py += v * math.sin(th) * dt
        th += w * dt
    else:
        Rc  = v / w
        dth = w * dt
        thn = th + dth
        Px += Rc * (math.sin(thn) - math.sin(th))
        Py -= Rc * (math.cos(thn) - math.cos(th))
        th  = thn
    return Px, Py, normalize_angle(th), v, w
RK4 = True
# ==========================================
# 5. EKF — PREDICT và UPDATE
# ==========================================
def ekf_predict(x, P, wL_n, wR_n, dt):
    if RK4:
        X, Y, th, b = x
        v = WHEEL_RADIUS * (wR_n + wL_n) * 0.5
        w = WHEEL_RADIUS * (wR_n - wL_n) / WHEEL_BASE

        # Áp dụng Runge-Kutta bậc 2 (Midpoint) theo tài liệu
        # Dùng góc ở giữa chu kỳ để tích phân chính xác hơn khi xe đi vào đường cong
        th_mid = th + (w * dt) / 2.0

        x_p = np.array([
            X  + v * math.cos(th_mid) * dt,
            Y  + v * math.sin(th_mid) * dt,
            normalize_angle(th + w * dt),
            b
        ])

        # Ma trận Jacobian F cũng phải được tuyến tính hóa tại góc th_mid
        F = np.eye(4)
        F[0, 2] = -v * math.sin(th_mid) * dt
        F[1, 2] =  v * math.cos(th_mid) * dt

        P_p = F @ P @ F.T + Q_MATRIX
        return x_p, P_p
    else:
        X, Y, th, b = x
        v = WHEEL_RADIUS * (wR_n + wL_n) * 0.5
        w = WHEEL_RADIUS * (wR_n - wL_n) / WHEEL_BASE

        x_p = np.array([
            X  + v * math.cos(th) * dt,
            Y  + v * math.sin(th) * dt,
            normalize_angle(th + w * dt),
            b
        ])

        F = np.eye(4)
        F[0, 2] = -v * math.sin(th) * dt
        F[1, 2] =  v * math.cos(th) * dt

        P_p = F @ P @ F.T + Q_MATRIX
        return x_p, P_p
    
def ekf_update(x_p, P_p, z):
    X, Y, th, b = x_p
    h_x = th + b                          
    y   = normalize_angle(z - h_x)        

    H = np.array([[0.0, 0.0, 1.0, 1.0]]) 
    S = (H @ P_p @ H.T)[0, 0] + R_COV    
    K = (P_p @ H.T) / S                   

    x_u    = x_p.copy()
    x_u[0] += K[0, 0] * y
    x_u[1] += K[1, 0] * y
    x_u[2]  = normalize_angle(x_p[2] + K[2, 0] * y)
    x_u[3] += K[3, 0] * y

    # Joseph form
    IKH   = np.eye(4) - K @ H
    P_u   = IKH @ P_p @ IKH.T + (R_COV * K) @ K.T

    return x_u, P_u

# ==========================================
# 6. VÒNG LẶP MÔ PHỎNG
# ==========================================
# ==========================================
# 6. VÒNG LẶP MÔ PHỎNG (ĐÃ TỐI ƯU BỘ NHỚ BẰNG DEQUE)
# ==========================================
def run_simulation(T_total=200.0, dt=0.01, display_last_mins=1.0):
    # display_last_mins: Hiển thị N phút cuối. Ví dụ: 1.0 = 60 giây cuối.
    steps = int(T_total / dt)
    t_arr = np.linspace(0, T_total, steps)
    np.random.seed(None)

    x0, y0, th0 = TRAJ_RADIUS, 0.0, math.pi / 2.0

    Px, Py, Ptheta = x0, y0, th0
    x_noise, y_noise, theta_noise = x0, y0, th0

    # AEKF State: [X, Y, Theta, Bias]
    X_ekf = np.array([x0, y0, th0, 0.0])  
    P_ekf = P0.copy()

    # Tính số lượng phần tử cần giữ lại trong mảng
    maxlen_steps = int((display_last_mins * 60.0) / dt)

    # Khởi tạo log bằng deque với maxlen. Tự động xóa dữ liệu cũ!
    log = {k: deque(maxlen=maxlen_steps) for k in [
        'x_set', 'y_set', 'x_real', 'y_real', 'x_noise', 'y_noise', 'x_ekf', 'y_ekf',
        'theta_ekf', 'b_ekf', 'omega_l_real', 'omega_r_real', 'omega_l_noise', 'omega_r_noise',
        'theta_set', 'theta_real', 'theta_noise', 'err_x_noise', 'err_y_noise', 'err_x_ekf', 'err_y_ekf'
    ]}
    
    # Mảng thời gian cũng phải dùng deque để đồng bộ kích thước
    log_t = deque(maxlen=maxlen_steps)

    IMU_INTERVAL = 5 
    step_count = 0

    # Khai báo tỷ lệ không đồng bộ (Đặt trước vòng lặp for)
    # Ví dụ: Encoder = 10ms, IMU = 50ms -> IMU_INTERVAL = 5
    IMU_INTERVAL = 1 
    step_count = 0

    for t in t_arr:
        step_count += 1
        
        # Tạo quỹ đạo
        Pdx, Pdy    = gen_trajectory(t)
        e_pos       = math.hypot(Pdx - Px, Pdy - Py)
        Pdtheta     = normalize_angle(math.atan2(Pdy - Py, Pdx - Px))
        e_theta     = normalize_angle(Pdtheta - Ptheta)
        Pv, Pw      = calc_setpoint(e_pos, e_theta)
        PwL, PwR    = inverse_kin(Pv, Pw)

        # Nhiễu cảm biến Encoder (Luôn có ở mỗi 10ms)
        wL_n        = PwL + np.random.normal(0, ENCODER_NOISE_STD)
        wR_n        = PwR + np.random.normal(0, ENCODER_NOISE_STD)

        # Tính toán động học
        Px, Py, Ptheta, v_t, w_t            = forward_kin(Px, Py, Ptheta, PwL, PwR, dt)
        x_noise, y_noise, theta_noise, _, _ = forward_kin(x_noise, y_noise, theta_noise, wL_n, wR_n, dt)

        # =======================================================
        # CHẠY AEKF (MULTI-RATE)
        # =======================================================
        # 1. Dự đoán (Predict): Chạy liên tục mỗi 10ms vì Encoder luôn có data
        X_ekf, P_ekf = ekf_predict(X_ekf, P_ekf, wL_n, wR_n, dt)

        # 2. Cập nhật (Update): Chỉ chạy khi IMU trả về data (Mỗi 50ms)
        if step_count % IMU_INTERVAL == 0:
            # Đọc IMU và cộng nhiễu (Chỉ lấy mẫu khi đủ thời gian)
            z_imu = normalize_angle(Ptheta + IMU_BIAS_TRUE + np.random.normal(0, IMU_THETA_NOISE_STD))
            # Cập nhật EKF
            X_ekf, P_ekf = ekf_update(X_ekf, P_ekf, z_imu)
        # =======================================================

        # Ghi Log vào deque
        log_t.append(t)
        log['x_set'].append(Pdx); log['y_set'].append(Pdy)
        log['x_real'].append(Px); log['y_real'].append(Py)
        log['x_noise'].append(x_noise); log['y_noise'].append(y_noise)
        log['x_ekf'].append(X_ekf[0]); log['y_ekf'].append(X_ekf[1])
        log['theta_ekf'].append(X_ekf[2]); log['b_ekf'].append(X_ekf[3])
        log['omega_l_real'].append(PwL); log['omega_r_real'].append(PwR)
        log['omega_l_noise'].append(wL_n); log['omega_r_noise'].append(wR_n)
        log['theta_set'].append(Pdtheta); log['theta_real'].append(Ptheta)
        log['theta_noise'].append(theta_noise)
        
        log['err_x_noise'].append(x_noise - Px); log['err_y_noise'].append(y_noise - Py)
        log['err_x_ekf'].append(X_ekf[0] - Px); log['err_y_ekf'].append(X_ekf[1] - Py)

    # Chuyển deque về lại list/numpy array trước khi return để không làm hỏng code vẽ plt
    final_log = {k: list(v) for k, v in log.items()}
    return np.array(log_t), final_log

# ==========================================
# 7. CHẠY VÀ VẼ ĐỒ THỊ (TÁCH BIỆT RÕ RÀNG)
# ==========================================
if __name__ == '__main__':
    # Cho phép nhập tay phút cuối cùng muốn hiển thị (Mặc định 1 phút)
    # mins = float(input("Nhập số phút cuối cùng muốn hiển thị trên đồ thị (VD: 0.5, 1, 10): ") or 1.0)
    
    # print(f"Đang chạy mô phỏng... (Chỉ lưu data {mins} phút cuối)")
    # Giả sử chạy tổng cộng 1 tiếng (3600s), nhưng chỉ plot phần cuối
    t, h = run_simulation(T_total=800.0, dt=0.01, display_last_mins=60)

    # Tính toán sai số Euclidean tổng hợp để in ra Terminal
    err_pos_noise = np.sqrt(np.array(h['err_x_noise'])**2 + np.array(h['err_y_noise'])**2)
    err_pos_ekf   = np.sqrt(np.array(h['err_x_ekf'])**2 + np.array(h['err_y_ekf'])**2)
    bias_learned  = h['b_ekf'][-1]

    print(f"\n{'='*50}")
    print(f"  Odometry  — Sai số vị trí trung bình: {err_pos_noise.mean():.4f}m")
    print(f"  EKF       — Sai số vị trí trung bình: {err_pos_ekf.mean():.4f}m")
    impr = (1 - err_pos_ekf.mean() / err_pos_noise.mean()) * 100
    print(f"  Cải thiện: {impr:.1f}%")
    print(f"  IMU bias thực: {IMU_BIAS_TRUE:.4f} rad ({math.degrees(IMU_BIAS_TRUE):.2f}°)")
    print(f"  EKF học được:  {bias_learned:.4f} rad ({math.degrees(bias_learned):.2f}°)")
    print(f"{'='*50}\n")

    plt.style.use('bmh')

    # ---------------------------------------------------------
    # BIỂU ĐỒ 1: VỊ TRÍ (QUỸ ĐẠO X-Y)
    # ---------------------------------------------------------
    plt.figure(figsize=(9, 9))
    plt.plot(h['x_set'], h['y_set'], 'k--', lw=1.5, label='Set (Mục tiêu)')
    plt.plot(h['x_real'], h['y_real'], 'g-', lw=2.5, label='Real (Thực tế)')
    plt.plot(h['x_noise'], h['y_noise'], 'r-', lw=1.2, alpha=0.7, label='Noise (Odometry lỗi)')
    plt.plot(h['x_ekf'], h['y_ekf'], 'b-', lw=2.0, label='EKF (Ước lượng)')
    plt.title('BIỂU ĐỒ VỊ TRÍ: QUỸ ĐẠO ROBOT (X-Y)', fontsize=14, fontweight='bold')
    plt.xlabel('X (m)', fontsize=12); plt.ylabel('Y (m)', fontsize=12)
    plt.legend(fontsize=11, loc='upper right'); plt.axis('equal'); plt.grid(True)
    plt.tight_layout(); plt.show()

    # ---------------------------------------------------------
    # BIỂU ĐỒ 2: VẬN TỐC GÓC ĐỘNG CƠ (WHEEL VELOCITIES)
    # ---------------------------------------------------------
    plt.figure(figsize=(12, 5))
    plt.plot(t, h['omega_l_real'], 'g-', lw=2, label='Omega L (Real)')
    plt.plot(t, h['omega_r_real'], 'c-', lw=2, label='Omega R (Real)')
    plt.plot(t, h['omega_l_noise'], 'r-', lw=1, alpha=0.5, label='Omega L (Noise)')
    plt.plot(t, h['omega_r_noise'], 'm-', lw=1, alpha=0.5, label='Omega R (Noise)')
    plt.title('BIỂU ĐỒ VẬN TỐC GÓC: ĐỘNG CƠ TRÁI / PHẢI', fontsize=14, fontweight='bold')
    plt.xlabel('Thời gian (s)', fontsize=12); plt.ylabel('Vận tốc góc (rad/s)', fontsize=12)
    plt.legend(fontsize=10, loc='upper right'); plt.grid(True)
    plt.tight_layout(); plt.show()

    # ---------------------------------------------------------
    # BIỂU ĐỒ 3: SAI SỐ TRỤC X VÀ TRỤC Y
    # ---------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('BIỂU ĐỒ SAI SỐ VỊ TRÍ: TRỤC X & TRỤC Y', fontsize=14, fontweight='bold')
    
    # Sai số trục X
    ax1.plot(t, h['err_x_noise'], 'r-', alpha=0.6, label='Error X (Noise)')
    ax1.plot(t, h['err_x_ekf'], 'b-', lw=2, label='Error X (EKF)')
    ax1.set_ylabel('Sai số X (m)', fontsize=12); ax1.legend(); ax1.grid(True)
    
    # Sai số trục Y
    ax2.plot(t, h['err_y_noise'], 'r-', alpha=0.6, label='Error Y (Noise)')
    ax2.plot(t, h['err_y_ekf'], 'b-', lw=2, label='Error Y (EKF)')
    ax2.set_xlabel('Thời gian (s)', fontsize=12); ax2.set_ylabel('Sai số Y (m)', fontsize=12)
    ax2.legend(); ax2.grid(True)
    plt.tight_layout(); plt.show()

    # ---------------------------------------------------------
    # BIỂU ĐỒ 4: GÓC HƯỚNG (THETA)
    # ---------------------------------------------------------
    plt.figure(figsize=(12, 5))
    plt.plot(t, h['theta_set'], 'k--', lw=1.5, label='Theta Set')
    plt.plot(t, h['theta_real'], 'g-', lw=2.5, label='Theta Real')
    plt.plot(t, h['theta_noise'], 'r-', lw=1.2, alpha=0.7, label='Theta Noise (Odometry)')
    plt.plot(t, h['theta_ekf'], 'b-', lw=2.0, label='Theta EKF')
    plt.title('BIỂU ĐỒ GÓC: HƯỚNG ROBOT (THETA)', fontsize=14, fontweight='bold')
    plt.xlabel('Thời gian (s)', fontsize=12); plt.ylabel('Góc (rad)', fontsize=12)
    plt.legend(fontsize=11, loc='upper right'); plt.grid(True)
    plt.tight_layout(); plt.show()

    # ---------------------------------------------------------
    # BIỂU ĐỒ 5: TỰ HỌC BIAS CỦA IMU (EKF BIAS LEARNING)
    # ---------------------------------------------------------
    plt.figure(figsize=(12, 5))
    plt.plot(t, h['b_ekf'], 'b-', lw=2.5, label=f'EKF Ước lượng Bias (Hội tụ: {bias_learned:.4f} rad)')
    plt.axhline(y=IMU_BIAS_TRUE, color='r', lw=2, ls='--', label=f'Bias Thực Tế = {IMU_BIAS_TRUE:.4f} rad')
    plt.title('BIỂU ĐỒ AEKF: TỰ ĐỘNG HỌC ĐỘ TRÔI OFFSET CỦA IMU', fontsize=14, fontweight='bold')
    plt.xlabel('Thời gian (s)', fontsize=12); plt.ylabel('Bias (rad)', fontsize=12)
    plt.legend(fontsize=12, loc='lower right'); plt.grid(True)
    plt.tight_layout(); plt.show()