import numpy as np
import matplotlib.pyplot as plt

# Đọc dữ liệu từ file Vel.txt
times = []
velocities = []

with open('D:\HOC\V\VSCODE\PYTHON\DoAn\DoAnFinal\FindPID\Vel.txt', 'r') as file:
    lines = file.readlines()
    
    for line in lines:
        # Chỉ xử lý các dòng có dữ liệu số
        if line.startswith('D:') and '\t' in line:
            parts = line.strip().split('\t')
            if len(parts) >= 3:
                try:
                    # Lấy time (cột 1) và velocity (cột 2)
                    time_val = float(parts[1])
                    vel_val = float(parts[2])
                    times.append(time_val)
                    velocities.append(vel_val)
                except:
                    continue

# Chuyển sang numpy array
times = np.array(times) / 1000  # Chuyển từ ms sang giây
velocities = np.array(velocities)

# Tìm giá trị xác lập (giá trị lớn nhất hoặc giá trị ổn định cuối)
# Giả sử giá trị xác lập là giá trị trung bình của 10% cuối dữ liệu
stable_start = int(len(velocities) * 0.9)  # Lấy 10% cuối
K = np.mean(velocities[stable_start:])

# Tính 63.2% của giá trị xác lập
target_velocity = K * 0.632

# Thời điểm bắt đầu tăng (t1)
idx1 = np.where(velocities > 0)[0]
if len(idx1) > 0:
    idx1 = idx1[0]
    t1 = times[idx1]
else:
    t1 = 0

# Thời điểm đạt 63.2% wmax_avg (t2) - nội suy tuyến tính
idx2 = np.where(velocities >= target_velocity)[0]
if len(idx2) > 0:
    idx2 = idx2[0]
    # Nội suy tuyến tính để tìm thời điểm chính xác
    if idx2 > 0:
        v1, v2 = velocities[idx2-1], velocities[idx2]
        t1_interp, t2_interp = times[idx2-1], times[idx2]
        tau63_time = t1_interp + (target_velocity - v1) * (t2_interp - t1_interp) / (v2 - v1)
    else:
        tau63_time = times[idx2]
else:
    tau63_time = times[-1]

# Tau63 = tau63_time - t1 (thời gian từ lúc bắt đầu tăng đến khi đạt 63.2%)
tau63 = tau63_time - t1

# Giá trị tại tau63% luôn bằng target_velocity (theo định nghĩa)
tau63_velocity = target_velocity

# Tìm t3 khi velocity > 80
idx3 = np.where(velocities > 80)[0]
if len(idx3) > 0:
    idx3 = idx3[0]
    t3 = times[idx3]
    w3 = velocities[idx3]
else:
    t3 = times[-1]
    w3 = velocities[-1]

# Tính amax (đạo hàm tại điểm đầu tiên)
w3 = round(w3, 3)
t3 = round(t3, 3)

w3_rounded = round(w3, 3)      # 80.422
t3_rounded = round(t3, 3)      # 1.061  
t1_rounded = round(t1, 3)      # 0.925

amax = (w3_rounded - 0) / (t3_rounded - t1_rounded)

print('\n--- Tính toán amax ---')
print(f't1 = {t1:.3f} s')
print(f't3 = {t3:.3f} s')
print(f'w3 = {w3:.3f} rad/s')
print(f'amax = {amax:.3f} rad/s²')

# Vẽ đồ thị
plt.figure(figsize=(12, 6))

# Vẽ dữ liệu thực tế
plt.plot(times, velocities, 'b-', linewidth=2, label='ω', markersize=5)

# Vẽ đường ngang tại 63.2% của giá trị xác lập
plt.axhline(y=target_velocity, color='r', linestyle='--', alpha=0.7, 
           label=f'ω63% = {target_velocity:.4f} (ωmax={K:.4f})')

# Đánh dấu điểm tau63% - phải nằm TRÊN đường 63.2%
plt.plot(tau63_time, tau63_velocity, 'ro', markersize=10, 
         label=f'τ63% tại ({tau63_time:.4f}s, {tau63_velocity:.4f})')

# Vẽ đường đứng tại tau63%
plt.axvline(x=tau63_time, color='g', linestyle='--', alpha=0.5, label=f'τ63% = {tau63:.2f}s')

# Đánh dấu điểm bắt đầu tăng
plt.axvline(x=t1, color='orange', linestyle='--', alpha=0.5, label=f't1 = {t1:.2f}s')

# Thêm thông tin
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Vận tốc (rad/s)', fontsize=12)
plt.title('Đồ thị vận tốc', fontsize=14, fontweight='bold')
plt.legend(loc='best')

# Hiển thị giá trị
print("="*60)
print("KẾT QUẢ TÍNH τ63%")
print("="*60)
print(f"1. Giá trị xác lập K (ước tính): {K:.6f} rad/s")
print(f"2. 63.2% của K: {target_velocity:.6f} rad/s")
print(f"3. Thời điểm bắt đầu tăng (t1): {t1:.4f} giây")
print(f"4. Thời điểm đạt 63.2% (t2): {tau63_time:.4f} giây")
print(f"5. Tau63 (t2 - t1): {tau63:.4f} giây")
print(f"6. Giá trị tại τ63%: {tau63_velocity:.6f} rad/s")
print(f"7. Tỉ lệ thực tế: {tau63_velocity/K*100:.2f}% của K")
print("="*60)

# Vẽ đồ thị
plt.figure(figsize=(12, 6))

# Vẽ dữ liệu thực tế
plt.plot(times, velocities, 'b-', linewidth=2, label='ω', markersize=5)

# Vẽ đường đạo hàm amax từ t1 đến t3
t_line = np.array([t1, t3])
v_line = np.array([0, w3])
plt.plot(t_line, v_line, 'm--', linewidth=2, label=f'amax = {amax:.4f} rad/s²')

# Đánh dấu điểm t3
plt.plot(t3, w3, 'mo', markersize=8, label=f't3 tại ({t3:.2f}s, {w3:.4f})')

# Vẽ đường đứng tại t3
plt.axvline(x=t3, color='m', linestyle='--', alpha=0.5)

# Thêm chú thích τ giữa t1 và t3
mid_t = (t1 + t3) / 2
plt.annotate('', xy=(t3, -20), xytext=(t1, -20),
            arrowprops=dict(arrowstyle='<->', color='black', lw=1.5))
plt.text(mid_t, -30, 'τ', fontsize=14, ha='center', fontweight='bold')

# Đánh dấu điểm bắt đầu tăng
plt.axvline(x=t1, color='black', linestyle='--', alpha=0.5, label=f't1 = {t1:.3f}s')
plt.title('Đồ thị vận tốc', fontsize=14, fontweight='bold')
plt.xlabel('Thời gian (s)', fontsize=12)
plt.ylabel('Vận tốc (rad/s)', fontsize=12)
plt.legend(loc='best')


plt.tight_layout()
plt.show()