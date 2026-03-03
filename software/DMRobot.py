import sys
import os
import numpy as np
from PyQt5 import QtWidgets, uic, QtCore
from PyQt5.QtCore import QObject, pyqtSignal, QUrl, QByteArray
from PyQt5.QtWebSockets import QWebSocket

import pyqtgraph as pg  # Quan trọng: Phải import để PlotWidget hoạt động
from collections import deque
import json
from enum import IntEnum
import struct

from libs.map_widget import MapWidget
from libs.astar import astar_search

MAP_FILE_PATH = r"D:\HOC\Project\DMRobot\software\output\map.txt"
FRAME_FILE_PATH = r"D:\HOC\Project\DMRobot\software\output\frame.txt"
USE_V1 = False

# ==========================================
# ESP
# ==========================================
ESP_IP = "myrobot.local"
RECEIVED_JSON = False
# ==========================================
# STM32 PROTOCOL
# ==========================================
PROTOCOL_HEADER_SIZE = 2    # 2 bytes Header (AA 55)
PROTOCOL_FOOTER_SIZE = 1    # 1 byte Footer (EE)
FLOAT_SIZE = 4              # 1 biến float = 4 bytes
NUM_FLOATS_PER_SAMPLE = 9   # Pdx, Pdy, Px, Py, V, W, Ex, Ey, Etheta
SAMPLE_TIME = 0.005
MAX_SAMPLES = 1000;
PLOT_TIME = 30  

# --- TỰ ĐỘNG TÍNH TOÁN (KHÔNG SỬA TAY) ---
# Kích thước 1 mẫu: 9 * 4 = 36 bytes
SAMPLE_SIZE = NUM_FLOATS_PER_SAMPLE * FLOAT_SIZE  

# Số lượng mẫu trong 1 gói tin (Phải khớp với Code STM32/ESP32)
SAMPLES_PER_BATCH = 4   

# Tổng kích thước gói tin mong đợi: 2 + (4 * 36) + 1 = 147 bytes
BATCH_SIZE = PROTOCOL_HEADER_SIZE + (SAMPLES_PER_BATCH * SAMPLE_SIZE) + PROTOCOL_FOOTER_SIZE

class CommandID(IntEnum):
    CMD_STOP = 0
    CMD_RUN = 1
    CMD_RETURN_HOME = 2
    CMD_SET_TRAJECTORY = 3
    CMD_SET_PATH = 4

class Shape(IntEnum):
    CIRCLE = 0
    SQUARE = 1
    TRIANGLE = 2
    EIGHT = 3
    PATH = 4
    LINE = 5
# ==========================================
# GRAPH CONFIG
# ==========================================
GRAPH_X_RANGE = 50
GRAPH_OFFSET = 3
# ==========================================
# FLAG
# ==========================================
START_DEBUG = False
START_SIMULATION = False
# ==========================================
# MAP CONFIG
# ==========================================
MAP_ROWS = 40           # Số hàng
MAP_COLS = 40           # Số cột
MAP_WIDTH_M = 8.0       # Chiều rộng thực tế (m)
MAP_HEIGHT_M = 8.0      # Chiều dài thực tế (m)
# Tự động tính kích thước 1 ô (đơn vị m)
UNIT_W = MAP_WIDTH_M / MAP_COLS
UNIT_H = MAP_HEIGHT_M / MAP_ROWS

PROTOCOL_HEADER_1 = 0xAA
PROTOCOL_HEADER_2 = 0xBB
PROTOCOL_FOOTER   = 0xEE

# Tạo mảng toàn số 0
GLOBAL_MAP_DATA = np.zeros((MAP_ROWS, MAP_COLS), dtype=int)
def init_global_map():
    global GLOBAL_MAP_DATA
    
    # 1. Kiểm tra file có tồn tại không
    if os.path.exists(MAP_FILE_PATH):
        try:
            # Load file vào biến Global
            # delimiter=' ' vì lúc lưu ta dùng dấu cách để ngăn cách các số
            GLOBAL_MAP_DATA = np.loadtxt(MAP_FILE_PATH, dtype=int, delimiter=' ')
            print(f"--> Đã load Map từ file: {MAP_FILE_PATH}")
            return
        except Exception as e:
            print(f"Lỗi đọc file map: {e}. Sẽ tạo map mặc định.")

    # 2. Nếu không có file (hoặc lỗi), tạo map mặc định
    print("--> File map chưa tồn tại, tạo map mới.")
    GLOBAL_MAP_DATA[:, :] = 0
    # Vẽ tường bao quanh
    GLOBAL_MAP_DATA[0, :] = 1
    GLOBAL_MAP_DATA[-1, :] = 1
    GLOBAL_MAP_DATA[:, 0] = 1
    GLOBAL_MAP_DATA[:, -1] = 1
    # (Có thể vẽ thêm chướng ngại vật mẫu nếu thích)

# GỌI HÀM KHỞI TẠO NGAY KHI CHẠY CHƯƠNG TRÌNH
init_global_map()

def calculate_checksum(data_bytes):
    """Hàm tính Checksum (Tổng các byte % 256)"""
    return sum(data_bytes) % 256

def pack_navigation_frame(cmd, start_node, path_list, velocity=0.5, is_cycle=False):    
    """
    Đóng gói dữ liệu Binary (Đã sửa để khớp với logic STM32/ESP32)
    Logic Checksum: Tổng toàn bộ gói (bao gồm Header)
    Cấu trúc: [Header 1] [Header 2] [CMD] [Count] [StartPos] [PathData] [Checksum]
    """
    # 1. Tự động tính Scale (Giữ nguyên logic cũ)
    unit_w = MAP_WIDTH_M / MAP_COLS
    unit_h = MAP_HEIGHT_M / MAP_ROWS
    offset_x = unit_w / 2.0
    offset_y = unit_h / 2.0

    def grid_to_meter(r, c):
        x_m = (c * unit_w) + offset_x
        y_m = ((MAP_ROWS - r) * unit_h) - offset_y
        return x_m, y_m

    # 2. Chuyển đổi dữ liệu
    start_x, start_y = grid_to_meter(start_node[0], start_node[1])
    
    flat_path = []
    for r, c in path_list:
        px, py = grid_to_meter(r, c)
        flat_path.append(px)
        flat_path.append(py)
        
    data_count = len(path_list) 
    
    # --- BẮT ĐẦU ĐÓNG GÓI THEO CHUẨN C ---
    
    # A. Header (2 bytes)
    header_bytes = struct.pack('<BB', PROTOCOL_HEADER_1, PROTOCOL_HEADER_2)
    
    # B. Metadata Mới (8 bytes)
    # CMD (1B) + Count (2B) + Velocity (4B float) + IsCycle (1B)
    # Format string: < B H f B
    cycle_byte = 1 if is_cycle else 0
    meta_bytes = struct.pack('<BHfB', cmd, data_count, velocity, cycle_byte)
    
    # C. Payload (Start Node + Path Data)
    payload_bytes = struct.pack('<ff', start_x, start_y)
    path_format = f'<{len(flat_path)}f'
    payload_bytes += struct.pack(path_format, *flat_path)
    
    # D. Tính Checksum (Tổng tất cả các byte trước đó)
    raw_frame_without_checksum = header_bytes + meta_bytes + payload_bytes
    checksum = sum(raw_frame_without_checksum) % 256
    
    # E. Gói tin hoàn chỉnh
    full_frame = raw_frame_without_checksum + struct.pack('<B', checksum)
    
    return full_frame

class EditMode(IntEnum):
    DRAW_WALL = 0   # Vẽ tường (Mặc định)
    SET_START = 1   # Đặt điểm bắt đầu
    SET_GOAL = 2    # Đặt điểm đích

class WebSocketComms(QObject):
    signal_connected = pyqtSignal(bool) 
    signal_message = pyqtSignal(str)
    signal_binary = pyqtSignal(bytes)
    signal_debug = QtCore.pyqtSignal(str)
    
    def __init__(self, host=ESP_IP, port=80): # Port 80 cho AsyncWebServer
        super().__init__()
        # Đường dẫn chuẩn cho thư viện ESPAsyncWebServer là /ws
        self.url = f"ws://{host}:{port}/ws"
        self.client = QWebSocket()
        
        self.client.connected.connect(self._on_connected)
        self.client.disconnected.connect(self._on_disconnected)

        if RECEIVED_JSON:
            self.client.textMessageReceived.connect(self._on_text_received)
        else:
            self.client.binaryMessageReceived.connect(self._on_binary_received)

    def connect_to_robot(self, host=ESP_IP, port=80):
        self.url = f"ws://{host}:{port}/ws"
        if START_DEBUG:
            print(f"Checking URL: {self.url}")
        self.signal_debug.emit(f"Checking URL: {self.url}")
        self.client.open(QUrl(self.url))

    def send_json(self, data_dict):
        if self.client.isValid():
            message = json.dumps(data_dict)
            self.client.sendTextMessage(message)
            if START_DEBUG:
                print(f"[Gửi đi]: {message}")
            self.signal_debug.emit(f"[Gửi đi]: {message}")
        else:
            if START_DEBUG:
                print("Lỗi: Chưa kết nối tới Robot!")
            self.signal_debug.emit("Lỗi: Chưa kết nối tới Robot!")

    def _on_connected(self):
        if START_DEBUG:
            print("--> Đã kết nối thành công!")
        self.signal_connected.emit(True)
        self.signal_debug.emit("--> Đã kết nối thành công!")

    def _on_disconnected(self):
        if START_DEBUG:
            print("--> Mất kết nối!")
        self.signal_connected.emit(False)
        self.signal_debug.emit("--> Mất kết nối!")

    def _on_text_received(self, message):
        self.signal_message.emit(message)

    def _on_binary_received(self, message):
        # Message ở đây là kiểu QByteArray, cần convert sang bytes của Python
        self.signal_binary.emit(bytes(message))

    def send_binary(self, data_bytes):
        """Hàm gửi dữ liệu nhị phân (Binary)"""
        if self.client.isValid():
            # 1. Chuyển đổi sang QByteArray (Bắt buộc với PyQt5)
            q_data = QByteArray(data_bytes)
            
            # 2. Gửi đi
            sent_count = self.client.sendBinaryMessage(q_data)
            
            if START_DEBUG:
                print(f"[WebSocket] Gửi Binary: {len(data_bytes)} bytes -> Return: {sent_count}")
            self.signal_debug.emit(f"[WebSocket] Gửi Binary: {len(data_bytes)} bytes -> Return: {sent_count}")
            return sent_count
        else:
            if START_DEBUG:
                print("Lỗi: Socket chưa kết nối, không thể gửi Binary!")
            self.signal_debug.emit("Lỗi: Socket chưa kết nối, không thể gửi Binary!")
            return 0

class MyRobotApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(MyRobotApp, self).__init__()
        
        # Load file UI trực tiếp
        if USE_V1:
            ui_path = os.path.join(os.path.dirname(__file__), 'DMRobot_v1.ui')
            uic.loadUi(ui_path, self)
        else:
            ui_path = os.path.join(os.path.dirname(__file__), 'DMRobot.ui')
            uic.loadUi(ui_path, self)
        
        # ==================================================
        # CẤU HÌNH
        # ==================================================
        self.init_ui_states()

        # ==================================================
        # DATA
        # ==================================================
        self.init_data_buffers()    # Khởi tạo bộ đệm dữ liệu
        self.buffer_dirty = False   # Cờ đánh dấu có data mới để vẽ

        # ==================================================
        # TIMER CẬP NHẬT ĐỒ THỊ
        # ==================================================
        self.init_timers()

        # ==================================================
        # WEBSOCKET COMMUNICATIONS
        # ==================================================
        self.init_communications()

        # ==================================================
        # MAP WIDGET
        # ==================================================
        self.init_map_system()

        # ==================================================
        # KẾT NỐI CÁC SỰ KIỆN
        # ==================================================
        self.init_signals_connections()
        
        self.sample_count = 0

    # ==================================================
    # MAP
    # ==================================================
    def _set_edit_mode(self, mode):
        """Hàm cập nhật biến trạng thái khi ấn Radio Button"""
        self.current_mode = mode
        
        # In ra để debug xem đã chuyển đúng chưa
        if mode == EditMode.DRAW_WALL:
            if START_DEBUG:
                print("--> Mode: VẼ TƯỜNG (Click để 0 <-> 1)")
            self.lineEdit_debug.setText("--> Mode: VẼ TƯỜNG")
        elif mode == EditMode.SET_START:
            if START_DEBUG:
                print("--> Mode: ĐẶT START (Click để chọn điểm xuất phát)")
            self.lineEdit_debug.setText("--> Mode: ĐẶT START")
        elif mode == EditMode.SET_GOAL:
            if START_DEBUG:
                print("--> Mode: ĐẶT GOAL (Click để thêm điểm đến)")
            self.lineEdit_debug.setText("--> Mode: ĐẶT GOAL")

    def _on_map_edit(self):
        """Khi ấn nút SỬA"""
        # 1. Logic cũ: Bật chế độ sửa cho Widget
        # Lưu ý: Cần chỉnh map_widget để nó KHÔNG tự sửa data nữa (tớ sẽ nói ở Bước 3)
        self.map_widget.set_editable(True) 
        
        # 2. Logic Mới: Reset về chế độ Vẽ Tường
        self.current_mode = EditMode.DRAW_WALL
        self.radioButton_mapDraw.setChecked(True) # Tự động tick vào ô Draw trên UI
        
        # 3. Quản lý nút bấm (Giữ nguyên)
        self.pushButton_mapReset.setEnabled(True)
        self.pushButton_mapSave.setEnabled(True)
        
        # 4. Enable các Radio Button (Để người dùng chọn)
        self.groupBox_mapChooseMode.setEnabled(True) # Giả sử cậu gom 3 radio vào 1 GroupBox tên này
        # Hoặc enable từng cái:
        self.radioButton_mapDraw.setEnabled(True)
        self.radioButton_mapStart.setEnabled(True)
        self.radioButton_mapGoal.setEnabled(True)
        
        if START_DEBUG:
            print("--> BẮT ĐẦU SỬA MAP (Default: Draw Mode)")
        self.lineEdit_debug.setText("Bắt đầu sửa Map")

    def _on_map_reset(self):
        """Reset bản đồ và xóa toàn bộ điểm Start/Goal/Path"""
        
        # 1. Reset dữ liệu vật cản về gốc
        init_global_map() 

        # 2. XÓA CÁC BIẾN TRẠNG THÁI (Theo yêu cầu)
        self.start_pose = None       # Xóa điểm bắt đầu
        self.goal_list = []          # Xóa danh sách điểm đích
        self.final_path_grid = []    # Xóa đường đi đã tính toán (nếu có)
        
        # 3. Cập nhật lại giao diện (Quan trọng: Phải truyền None/[] để xóa hình vẽ)
        self.map_widget.load_map_data(
            GLOBAL_MAP_DATA, 
            MAP_WIDTH_M, 
            MAP_HEIGHT_M,
            start=None,     # Truyền None để xóa điểm vàng
            goals=[],       # Truyền list rỗng để xóa điểm xanh
            path=[]         # Truyền list rỗng để xóa đường tím
        )

        # 4. Reset các giao diện khác (nếu cần)
        self.checkBox_mapCycle.setChecked(False) # Bỏ check lặp
        
        # 5. Thông báo
        if START_DEBUG:
            print("--> Đã Reset Map: Xóa sạch Start, Goal và Path.")
        self.lineEdit_debug.setText("Đã Reset Map & Dữ liệu")

    def _on_map_save(self):
        global GLOBAL_MAP_DATA
        
        # 1. Lưu mảng map (0/1) ra file text
        try:
            # fmt='%d' để lưu số nguyên (0 1) cho đẹp, thay vì 0.0000
            np.savetxt(self.MAP_FILE_PATH, GLOBAL_MAP_DATA, fmt='%d', delimiter=' ')
            if START_DEBUG:
                print(f"--> Đã lưu file Map tại: {self.MAP_FILE_PATH}")
                QtWidgets.QMessageBox.information(self, "Thành công", f"Đã lưu Map vào:\n{self.MAP_FILE_PATH}")
            self.lineEdit_debug.setText(f"Đã lưu file Map tại: {self.MAP_FILE_PATH}")
        except Exception as e:
            if START_DEBUG:
                print(f"Lỗi lưu file: {e}")
                QtWidgets.QMessageBox.critical(self, "Lỗi", f"Không lưu được file:\n{e}")
            self.lineEdit_debug.setText(f"Lỗi lưu file: {e}")

        # 2. Khóa giao diện (Logic cũ)
        self.radioButton_mapDraw.setEnabled(False)
        self.radioButton_mapStart.setEnabled(False)
        self.radioButton_mapGoal.setEnabled(False)
        self.map_widget.set_editable(False)
        self.pushButton_mapEdit.setEnabled(True)
        self.pushButton_mapReset.setEnabled(False)
        self.pushButton_mapSave.setEnabled(False)
        
        if START_DEBUG:
            # [DEBUG] In ra tọa độ Start và List Goal để cậu kiểm tra
            print("--- SUMMARY ---")
            print(f"Start Pose: {self.start_pose}")
            print(f"Goal List: {self.goal_list}")
        self.lineEdit_debug.setText(f"Start Pose: {self.start_pose}\
                                      Goal List: {self.goal_list}")

    def _on_map_calculate(self):
        """Tính toán đường đi A* và Đóng gói dữ liệu kèm Cấu hình"""
        # 1. Kiểm tra đầu vào cơ bản
        if self.start_pose is None:
            QtWidgets.QMessageBox.warning(self, "Thiếu thông tin", "Chưa chọn điểm START!")
            return
        if not self.goal_list:
            QtWidgets.QMessageBox.warning(self, "Thiếu thông tin", "Chưa chọn điểm GOAL!")
            return

        full_path = [] # Mảng chứa toàn bộ đường đi nối các điểm
        current_start = self.start_pose 
        
        # ---------------------------------------------------------
        # 2. QUY TRÌNH TÌM ĐƯỜNG A* (Start -> G1 -> G2 ...)
        # ---------------------------------------------------------
        try:
            for i, target_goal in enumerate(self.goal_list):
                if START_DEBUG:
                    print(f"--> Đang tìm đường: {current_start} đến {target_goal}...")
                self.lineEdit_debug.setText(f"Đang tìm đường: {current_start} đến {target_goal}...")
                # Gọi hàm A* (Input: Map, Start, Goal)
                segment_path = astar_search(GLOBAL_MAP_DATA, current_start, target_goal)
                
                if segment_path is None:
                    if START_DEBUG:
                        print(f"Lỗi: Không tìm thấy đường đến Goal {i+1}!")
                    QtWidgets.QMessageBox.critical(self, "Lỗi", f"Không thể tìm đường đến Goal {i+1} (Có thể bị tường chặn).")
                    return
                
                # Logic nối điểm: Nếu không phải đoạn đầu tiên, bỏ điểm đầu của segment
                # (vì nó trùng với điểm cuối của đoạn trước đó)
                if i > 0:
                    segment_path = segment_path[1:]
                
                full_path.extend(segment_path)
                
                # Cập nhật điểm bắt đầu cho vòng lặp sau
                current_start = target_goal
            
            # Lưu kết quả đường đi vào biến class
            self.final_path_grid = full_path
            if START_DEBUG:
                print(f"--> TÍNH TOÁN XONG! Tổng số điểm: {len(full_path)}")
            self.lineEdit_debug.setText(f"TÍNH TOÁN XONG! Tổng số điểm: {len(full_path)}")
            
            # Cập nhật hiển thị trên Map Widget
            self.map_widget.load_map_data(
                GLOBAL_MAP_DATA, 
                MAP_WIDTH_M, 
                MAP_HEIGHT_M, 
                start=self.start_pose, 
                goals=self.goal_list,
                path=self.final_path_grid
            )

            # ---------------------------------------------------------
            # 3. ĐÓNG GÓI DỮ LIỆU (PACKING FRAME)
            # ---------------------------------------------------------
            
            # A. Lấy thông tin cấu hình từ UI (Vận tốc & Chu kỳ)
            try:
                # Xử lý Vận tốc (Thay dấu phẩy thành chấm để tránh lỗi float)
                vel_text = self.lineEdit_mapParamsVelConfig.text().replace(',', '.')
                velocity = float(vel_text)
            except ValueError:
                if START_DEBUG:
                    print("⚠️ Lỗi nhập liệu Vận tốc -> Dùng mặc định 0.5 m/s")
                self.lineEdit_debug.setText("⚠️ Lỗi nhập liệu Vận tốc -> Dùng mặc định 0.5 m/s")

                velocity = 0.5
                self.lineEdit_mapParamsVelConfig.setText("0.5")

            is_cycle = self.checkBox_mapCycle.isChecked()

            # B. Lấy mẫu đường đi (Down-sampling) nếu cần
            # [::1] là lấy tất cả, [::2] là lấy cách 1 điểm...
            path_to_send = self.final_path_grid[::1] 

            # C. Gọi hàm đóng gói (Hàm mới đã cập nhật Velocity & Cycle)
            # Lưu ý: Đảm bảo CMD_SET_PATH đã được import hoặc define (thường là 0x05)
            self.pending_binary_frame = pack_navigation_frame(
                cmd=CommandID.CMD_SET_PATH, 
                start_node=self.start_pose,
                path_list=path_to_send,
                velocity=velocity,  # <--- Mới
                is_cycle=is_cycle   # <--- Mới
            )
            
            # ---------------------------------------------------------
            # 4. LƯU FILE DEBUG & CẬP NHẬT UI
            # ---------------------------------------------------------
            try:
                with open(FRAME_FILE_PATH, "wb") as f:
                    f.write(self.pending_binary_frame)
                if START_DEBUG:
                    print(f"--> [TEST] Đã lưu Frame ({len(self.pending_binary_frame)} bytes) vào: {FRAME_FILE_PATH}")
                    print(f"--> Config: Vel={velocity}, Cycle={is_cycle}")
                
                QtWidgets.QMessageBox.information(self, "Thành công", 
                                                f"Đã tính toán và tạo gói tin!\n"
                                                f"Số điểm: {len(path_to_send)}\n"
                                                f"Vận tốc: {velocity} m/s\n"
                                                f"Chế độ lặp: {'BẬT' if is_cycle else 'TẮT'}")
                
                # Enable nút Gửi lệnh
                self.pushButton_mapSetConfig.setEnabled(True)
                
            except Exception as e:
                if START_DEBUG:
                    print(f"Lỗi lưu file Frame: {e}")
                self.lineEdit_debug.setText(f"Lỗi lưu file Frame: {e}")

                QtWidgets.QMessageBox.warning(self, "Cảnh báo", f"Lỗi lưu file debug: {e}")

        except Exception as e:
            if START_DEBUG:
                print(f"Lỗi thuật toán/Đóng gói: {e}")
            self.lineEdit_debug.setText(f"Lỗi thuật toán/Đóng gói: {e}")
            import traceback
            traceback.print_exc()
            self.pushButton_mapSetConfig.setEnabled(False)

    def _on_map_set_config(self):
        if self.pending_binary_frame:
            # 1. Ép kiểu từ Python bytes -> QByteArray
            q_bytes = QByteArray(self.pending_binary_frame)
            
            # 2. Gửi QByteArray
            # Lưu ý: Kiểm tra xem client của cậu là QWebSocket trực tiếp hay qua class wrapper
            val = self.comms.send_binary(self.pending_binary_frame)
            
            # 3. Kiểm tra kết quả trả về (hàm này trả về số byte gửi đi)
            if START_DEBUG:
                print(f"--> Bytes sent return value: {val}") 
                
                print("--> Đã gửi Binary Frame xuống Robot!")
                QtWidgets.QMessageBox.information(self, "Thông báo", "Đã gửi dữ liệu xuống Robot!")
        else:
            if START_DEBUG:
                print("Lỗi: Chưa có dữ liệu để gửi!")

    def change_mode_ui(self, index):
        """Chuyển đổi StackedWidget và Tối ưu hóa Timer"""
        # 1. Chuyển trang
        self.stackedWidget_Main.setCurrentIndex(index)
        
        # Nếu cậu có stackedWidget_mode (thanh bên trái) thì chuyển luôn
        if hasattr(self, 'stackedWidget_mode'):
            self.stackedWidget_mode.setCurrentIndex(index)

        # 2. XỬ LÝ TIMER (QUAN TRỌNG)
        if index == 0: 
            # ---> Đang ở Tab ĐỒ THỊ
            if not self.timer_plot.isActive():
                self.timer_plot.start()
                # Xóa queue cũ để đồ thị không bị vẽ giật cục 1 phát
                self.render_queue.clear() 
                print("--> Mode: GRAPH (Timer ON)")
                
        elif index == 1: 
            # ---> Đang ở Tab MAP
            if self.timer_plot.isActive():
                self.timer_plot.stop()
                if START_DEBUG:
                    print("--> Mode: MAP (Timer OFF - Save CPU)")
                self.lineEdit_debug.setText("Mode: MAP (Timer OFF - Save CPU)")

    def handle_map_click(self, row, col):
        """Xử lý khi user click vào ô (row, col)"""
        
        # MODE 1: VẼ TƯỜNG
        if self.current_mode == EditMode.DRAW_WALL:
            # Nếu lỡ click vào ô đang là Start hoặc Goal -> Xóa Start/Goal đó đi
            if self.start_pose == (row, col):
                self.start_pose = None
            if (row, col) in self.goal_list:
                self.goal_list.remove((row, col))
            
            # Đảo trạng thái tường
            current_val = GLOBAL_MAP_DATA[row, col]
            GLOBAL_MAP_DATA[row, col] = 1 - current_val
            if START_DEBUG:
                print(f"Click [{row},{col}]: Đảo tường -> {GLOBAL_MAP_DATA[row, col]}")
            self.lineEdit_debug.setText(f"Click [{row},{col}]: Đảo tường -> {GLOBAL_MAP_DATA[row, col]}")

        # MODE 2: ĐẶT ĐIỂM START
        elif self.current_mode == EditMode.SET_START:
            # Xóa Start cũ (chỉ cho phép 1 điểm Start)
            self.start_pose = (row, col)
            
            # Start không được là Tường -> Xóa tường nếu có
            GLOBAL_MAP_DATA[row, col] = 0 
            
            # Nếu trùng Goal -> Xóa Goal
            if (row, col) in self.goal_list:
                self.goal_list.remove((row, col))
            
            if START_DEBUG:
                print(f"Click [{row},{col}]: Set START")
            self.lineEdit_debug.setText(f"Click [{row},{col}]: Set START")

        # MODE 3: ĐẶT ĐIỂM GOAL
        elif self.current_mode == EditMode.SET_GOAL:
            # Goal không được là Tường -> Xóa tường nếu có
            GLOBAL_MAP_DATA[row, col] = 0
            
            # Nếu trùng Start -> Xóa Start (Hiếm khi xảy ra nhưng cứ check)
            if self.start_pose == (row, col):
                self.start_pose = None
                
            # Logic thêm/xóa Goal
            if (row, col) in self.goal_list:
                self.goal_list.remove((row, col)) # Click lại thì xóa
                if START_DEBUG:
                    print(f"Click [{row},{col}]: Xóa GOAL")
                self.lineEdit_debug.setText(f"Click [{row},{col}]: Xóa GOAL")
            else:
                self.goal_list.append((row, col)) # Chưa có thì thêm
                if START_DEBUG:
                    print(f"Click [{row},{col}]: Thêm GOAL (Tổng: {len(self.goal_list)})")
                self.lineEdit_debug.setText(f"Click [{row},{col}]: Thêm GOAL (Tổng: {len(self.goal_list)})")
        
        # CẬP NHẬT GIAO DIỆN (Truyền đủ Start/Goal vào để widget vẽ màu)
        self.map_widget.load_map_data(
            GLOBAL_MAP_DATA, 
            MAP_WIDTH_M, 
            MAP_HEIGHT_M, 
            start=self.start_pose, 
            goals=self.goal_list
        )

    # ==================================================
    # COMMS
    # ==================================================
    def _send_trajectory_config(self):
        """Hàm gửi cấu hình quỹ đạo xuống ESP32"""
        # 1. Mapping: Ánh xạ từ Tiếng Việt (GUI) sang Tiếng Anh (Code)
        # Sử dụng Dictionary để code gọn và dễ mở rộng sau này (ví dụ thêm Tam giác, Vô cực...)
        shape_map = {
            "Vuông": Shape.SQUARE,
            "Tròn": Shape.CIRCLE,
            "Tam giác": Shape.TRIANGLE,
            "Số 8": Shape.EIGHT,
            "Thẳng": Shape.LINE
        }
        
        # Lấy text hiện tại của ComboBox
        current_text = self.comboBox_chooseTraj.currentText().strip()
        
        # Lấy giá trị mapped, nếu không tìm thấy thì mặc định là square
        selected_shape = shape_map.get(current_text, Shape.SQUARE)

        try:
            if self.lineEdit_paramsTrajConfig.text().strip() == "" or \
               self.lineEdit_paramsVelConfig.text().strip() == "":
                QtWidgets.QMessageBox.critical(None, "Vui lòng nhập đầy đủ thông số!")

            # 2. Lấy dữ liệu và Xử lý số học (Input Validation)
            # .replace(',', '.') giúp người dùng nhập 1,5 hay 1.5 đều hiểu
            param_str = self.lineEdit_paramsTrajConfig.text().replace(',', '.')
            vel_str = self.lineEdit_paramsVelConfig.text().replace(',', '.')

            # Ép kiểu sang float
            para_val = float(param_str)
            vel_val = float(vel_str)

            # 3. Đóng gói JSON
            # Cần gửi kèm CMD_SET_TRAJECTORY (id=3) để ESP32 biết đây là lệnh cài đặt
            payload = {
                "cmd": int(CommandID.CMD_SET_TRAJECTORY), # = 3
                "Shape": selected_shape,
                "Para": para_val,
                "Vel": vel_val
            }

            # 4. Gửi đi
            self.comms.send_json(payload)
            if START_DEBUG:
                print(f"[CONFIG] Đã gửi cấu hình: {payload}")
            self.lineEdit_debug.setText(f"[CONFIG] Đã gửi cấu hình: {payload}")

        except ValueError:
            # Nếu người dùng nhập chữ linh tinh -> Báo lỗi chứ không để Crash App
            if START_DEBUG:
                print("[ERROR] Vui lòng nhập số hợp lệ vào ô tham số!")
            
            QtWidgets.QMessageBox.critical(None, "Vui lòng nhập số hợp lệ!")

    def _send_command(self, cmd_id):
        self.comms.send_json({"cmd": cmd_id})

    def process_binary_data(self, data_bytes):
        # 1. Kiểm tra kích thước gói tin dựa trên cấu hình global
        if len(data_bytes) != BATCH_SIZE:
            if START_DEBUG:
                print(f"[CẢNH BÁO] Lỗi size: Nhận {len(data_bytes)} != {BATCH_SIZE}")
            self.lineEdit_debug.setText(f"[CẢNH BÁO] Lỗi size: Nhận {len(data_bytes)} != {BATCH_SIZE}")
            return
        # 1. Check Header (AA 55) và Footer (EE) nếu cần thiết
        # header = data_bytes[0:2]
        # footer = data_bytes[-1]
        
        # 2. Giải mã Batch
        # Bỏ qua 2 byte đầu (Header), bắt đầu đọc data
        offset = PROTOCOL_HEADER_SIZE 
        
        try:
            # [FIX LỖI QUAN TRỌNG]: Vòng lặp chỉ chạy số lần bằng số mẫu (SAMPLES_PER_BATCH = 4)
            # Code cũ chạy range(BATCH_SIZE) = 147 lần -> Gây lỗi unpack
            for i in range(SAMPLES_PER_BATCH):
                # Cắt đúng kích thước 1 mẫu (SAMPLE_SIZE = 36 bytes)
                chunk = data_bytes[offset : offset + SAMPLE_SIZE]
                
                # Cập nhật offset cho vòng lặp sau
                offset += SAMPLE_SIZE

                # Unpack 9 số float (Little Endian '<')
                # Pdx, Pdy, Px, Py, V, W, Ex, Ey, Etheta
                values = struct.unpack('<9f', chunk)

                # --- [LOGIC MỚI] PHÂN LOẠI HIỂN THỊ ---
                # Kiểm tra xem đang ở Tab nào? (0: Đồ thị, 1: Map)
                current_tab = self.stackedWidget_Main.currentIndex()

                if current_tab == 0: 
                    # TRƯỜNG HỢP 1: Đang xem Đồ thị -> Đẩy vào hàng đợi vẽ đồ thị
                    self.render_queue.append(values)
                
                elif current_tab == 1:
                    # TRƯỜNG HỢP 2: Đang xem Map -> Lấy Px, Py để vẽ Robot ngay
                    # values[2] là Px, values[3] là Py
                    px_robot = values[2]
                    py_robot = values[3]

                    # Cập nhật lên Map Widget
                    # [LƯU Ý QUAN TRỌNG]: Nếu gốc toạ độ Robot (0,0) nằm giữa phòng
                    # mà gốc Map (0,0) nằm góc dưới trái, cậu cần cộng Offset tại đây.
                    # Ví dụ: px_robot += 4.0; py_robot += 4.0
                    self.map_widget.update_robot_pos(px_robot, py_robot)
                
        except struct.error as e:
            if START_DEBUG:
                # Lỗi này thường do chunk không đủ 36 bytes (đã được fix ở trên)
                print(f"Lỗi struct: {e} | Offset: {offset} | Len chunk: {len(chunk)}")
            self.lineEdit_debug.setText(f"Lỗi struct: {e} | Offset: {offset} | Len chunk: {len(chunk)}")
        except Exception as e:
            if START_DEBUG:
                print(f"Lỗi khác: {e}")
            self.lineEdit_debug.setText(f"Lỗi khác: {e}")

    def process_json_data(self, message):
        try:
            # Parse JSON
            data = json.loads(message)
            
            # Tính thời gian
            t = self.sample_count * self.dt
            
            # Cập nhật buffer - SỬ DỤNG ĐỦ DỮ LIỆU TỪ ESP32
            self._update_buffers(
                t=t, 
                setpoint=[data.get('Pdx', 0.0), data.get('Pdy', 0.0)],
                current=[data.get('Px', 0.0), data.get('Py', 0.0)],
                control=[data.get('V', 0.0), data.get('W', 0.0)],
                errors=[data.get('Ex', 0.0), data.get('Ey', 0.0), data.get('Etheta', 0.0)]
            )
            
            self.sample_count += 1
            
        except json.JSONDecodeError as e:
            if START_DEBUG:
                print(f"JSON Error: {e}")
        except Exception as e:
            if START_DEBUG:
                print(f"Data Error: {e}")

    def connect(self):
        # Lấy chuỗi IP từ giao diện và xóa khoảng trắng thừa (nếu có)
         # Lấy text từ giao diện
        host_input = self.lineEdit_IPAddress.text().strip()

        # Logic: Nếu có nhập thì dùng cái nhập, nếu TRỐNG thì dùng mặc định 'myrobot.local'
        target_host = host_input if host_input else ESP_IP
        
        # Cổng port: Dùng 80 như cậu đã xác nhận bên ESP
        target_port = 80

        # Kiểm tra xem người dùng có nhập gì không
        if target_host:
            # Nếu có nhập, truyền IP đó vào hàm kết nối
            self.comms.connect_to_robot(host=target_host, port=target_port)
        else:
            # Nếu để trống, gọi hàm không tham số (nó sẽ dùng IP mặc định cậu đã set)
            self.comms.connect_to_robot()

        if START_DEBUG:
            print(f"Đang kết nối tới Robot tại IP: {target_host if target_host else ESP_IP}...")
        self.lineEdit_debug.setText(f"Đang kết nối tới Robot tại IP: {target_host if target_host else ESP_IP}...")
        self.lineEdit_status.setText("Connecting...")
    
    def _check_connection_status(self):
        """Hàm kiểm tra trạng thái kết nối thủ công (khi nhấn nút Check)"""
        # 1. Kiểm tra trạng thái thực tế từ socket
        is_connected = self.comms.client.isValid()

        # 2. Cập nhật toàn bộ giao diện (ẩn/hiện nút) bằng hàm update có sẵn
        # Đoạn này thay thế cho hàng chục dòng code lặp lại của cậu
        self.update_connection_status(is_connected)

        # 3. Hiển thị thông báo (Feedback cho người dùng)
        if is_connected:
            QtWidgets.QMessageBox.information(self, "Trạng thái", "Kết nối vẫn ổn định! ✅")
        else:
            QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Đã mất kết nối tới Robot! ❌")
            
            # Chỉ sửa text nút Connect thành "Kết nối lại" khi check thủ công thấy lỗi
            self.pushButton_connect.setText("Kết nối lại")

    def update_connection_status(self, is_connected):
        if is_connected:
            self.lineEdit_status.setText("🟢 Connected")

            # Cấu hình label
            read_only_fields = [
                self.lineEdit_paramsTrajConfig,
                self.lineEdit_paramsVelConfig,
                # --- map
                self.lineEdit_mapParamsVelConfig
            ]
            for field in read_only_fields:
                field.setReadOnly(False)

            # Cấu hình nút bấm
            buttons = [
                self.pushButton_config,
                self.pushButton_run,
                self.pushButton_stop,
                self.pushButton_returnHome,
                self.pushButton_clearGraph,
                # --- map
                self.pushButton_mapEdit,
                self.pushButton_mapReset,
                self.pushButton_mapSave,
                self.pushButton_mapCal,
                self.pushButton_mapSetConfig,
            ]
            for button in buttons:
                button.setEnabled(True)
            self.pushButton_connect.setEnabled(False)
            
        else:
            self.lineEdit_status.setText("🔴 Disconnected")

            # Cấu hình label
            read_only_fields = [
                self.lineEdit_paramsTrajConfig,
                self.lineEdit_paramsVelConfig,
                # --- map
                self.lineEdit_mapParamsVelConfig
            ]
            for field in read_only_fields:
                field.setReadOnly(True)

            # Cấu hình nút bấm
                        # Cấu hình nút bấm
            buttons = [
                self.pushButton_config,
                self.pushButton_run,
                self.pushButton_stop,
                self.pushButton_returnHome,
                self.pushButton_clearGraph,
                # --- map
                self.pushButton_mapEdit,
                self.pushButton_mapReset,
                self.pushButton_mapSave,
                self.pushButton_mapCal,
                self.pushButton_mapSetConfig,
            ]
            for button in buttons:
                button.setEnabled(False)
            self.pushButton_connect.setEnabled(True)

    def _print_fps(self):
        if START_DEBUG:
            # print(f"📊 FPS: {self.fps_counter} | Buffer: {len(self.data_time)} samples")
            pass
        self.lineEdit_fps.setText(f"{self.fps_counter} FPS")
        self.fps_counter = 0

    # ==================================================
    # WINDOW CONFIG
    # ==================================================
    def init_ui_states(self):
        # 1. Cấu hình đồ thị
        self.curves = {}
        self.config_graphs()

        # 2. Trạng thái Read-Only cho các LineEdit
        read_only_fields = [
            self.lineEdit_status,
            self.lineEdit_paramsTrajConfig,
            self.lineEdit_paramsVelConfig,
            self.lineEdit_fps,
            self.lineEdit_debug,
            # --- map
            self.lineEdit_mapParamsVelConfig
        ]
        for field in read_only_fields:
            field.setReadOnly(True)

        # 3. Trạng thái Enable/Disable cho các nút bấm
        self.pushButton_config.setEnabled(False)
        self.pushButton_run.setEnabled(False)
        self.pushButton_stop.setEnabled(False)
        self.pushButton_returnHome.setEnabled(False)
        self.pushButton_clearGraph.setEnabled(False)
        
        # Nhóm nút bấm MAP
        self.pushButton_mapEdit.setEnabled(False)
        self.pushButton_mapReset.setEnabled(False)
        self.pushButton_mapSave.setEnabled(False)
        self.pushButton_mapCal.setEnabled(False)
        self.pushButton_mapSetConfig.setEnabled(False)

        # 4. Giá trị mặc định
        self.lineEdit_paramsTrajConfig.setText("1.0")
        self.lineEdit_paramsVelConfig.setText("0.6283")

        # --- map
        self.lineEdit_mapParamsVelConfig.setText("1.0")

    def init_timers(self):
        # --- Timer cập nhật đồ thị (UI Refresh) ---
        self.timer_plot = QtCore.QTimer()
        self.timer_plot.setInterval(PLOT_TIME) 
        self.timer_plot.timeout.connect(self._update_plot_curves)
        self.timer_plot.start()

        # --- Timer tính toán FPS (Performance Monitor) ---
        self.fps_counter = 0
        self.fps_timer = QtCore.QTimer()
        self.fps_timer.setInterval(1000) 
        self.fps_timer.timeout.connect(self._print_fps)
        self.fps_timer.start()

        # --- Timer giả lập (Debug Mode) ---
        if START_SIMULATION:
            self.sim_timer = QtCore.QTimer()
            self.sim_timer.timeout.connect(self.simulate_incoming_data)
            self.sim_timer.start(50) 
            self.sim_angle = 0.0

    def init_communications(self):
        # 1. Khởi tạo đối tượng giao tiếp (WebSocket)
        # Host và Port được lấy từ cấu hình mặc định
        self.comms = WebSocketComms(host=ESP_IP, port=80)
        
        # 2. Kết nối các tín hiệu (Signals) từ Module về App để xử lý dữ liệu
        self.comms.signal_connected.connect(self.update_connection_status)
        
        if RECEIVED_JSON:
            self.comms.signal_message.connect(self.process_json_data)
        
        self.comms.signal_binary.connect(self.process_binary_data)
        self.comms.signal_debug.connect(self.lineEdit_debug.setText)
        
        # 3. Cấu hình giao diện ban đầu cho phần kết nối
        self.lineEdit_IPAddress.setPlaceholderText(ESP_IP)
        
        # 4. Tự động kết nối ngay khi khởi động
        self.connect()

    def init_map_system(self):
        # 1. Khởi tạo và nạp dữ liệu cho Map Widget
        self.map_widget = MapWidget()
        self.map_widget.load_map_data(GLOBAL_MAP_DATA, MAP_WIDTH_M, MAP_HEIGHT_M)

        # 2. Thiết lập Layout container (Đưa widget vào giao diện)
        if self.widget_map_container.layout() is None:
            layout = QtWidgets.QVBoxLayout(self.widget_map_container)
            layout.setContentsMargins(0, 0, 0, 0)
            self.widget_map_container.setLayout(layout)
        self.widget_map_container.layout().addWidget(self.map_widget)

        # 3. Trạng thái hiển thị ban đầu
        self.stackedWidget_Main.setCurrentIndex(0)
        self.stackedWidget_mode.setCurrentIndex(0)
        self.current_mode = EditMode.DRAW_WALL
        self.radioButton_mapDraw.setChecked(True)

        # 4. Khởi tạo các biến lưu trữ dữ liệu điều hướng
        self.start_pose = None       # Điểm bắt đầu
        self.goal_list = []          # Danh sách điểm đích
        self.final_path_grid = []    # Đường đi cuối cùng
        self.MAP_FILE_PATH = MAP_FILE_PATH
        self.pending_binary_frame = None

    def init_signals_connections(self):
        # --- Điều khiển Robot (Control Commands) ---
        self.pushButton_run.clicked.connect(lambda: self._send_command(CommandID.CMD_RUN))
        self.pushButton_stop.clicked.connect(lambda: self._send_command(CommandID.CMD_STOP))
        self.pushButton_returnHome.clicked.connect(lambda: self._send_command(CommandID.CMD_RETURN_HOME))
        self.pushButton_config.clicked.connect(self._send_trajectory_config)

        # --- Kết nối & Đồ thị (Connection & Graph) ---
        self.pushButton_connect.clicked.connect(self.connect)
        self.pushButton_checkConnect.clicked.connect(self._check_connection_status)
        self.pushButton_clearGraph.clicked.connect(self._clear_plot)

        # --- Tương tác Bản đồ (Map Interactions) ---
        self.pushButton_mapEdit.clicked.connect(self._on_map_edit)
        self.pushButton_mapReset.clicked.connect(self._on_map_reset)
        self.pushButton_mapSave.clicked.connect(self._on_map_save)
        self.pushButton_mapCal.clicked.connect(self._on_map_calculate)
        self.pushButton_mapSetConfig.clicked.connect(self._on_map_set_config)
        
        # Chế độ vẽ bản đồ
        self.radioButton_mapDraw.clicked.connect(lambda: self._set_edit_mode(EditMode.DRAW_WALL))
        self.radioButton_mapStart.clicked.connect(lambda: self._set_edit_mode(EditMode.SET_START))
        self.radioButton_mapGoal.clicked.connect(lambda: self._set_edit_mode(EditMode.SET_GOAL))
        
        # Sự kiện click trực tiếp trên widget bản đồ
        self.map_widget.cellClicked.connect(self.handle_map_click)

        # --- Thay đổi cấu hình (UI Changes) ---
        self.comboBox_chooseTraj.currentTextChanged.connect(self._update_traj_label)
        self.comboBox_chooseMode.currentIndexChanged.connect(self.change_mode_ui)

    def config_graphs(self):
        if USE_V1:
            # --- THIẾT LẬP MÀU THEME BLUE-TEAL ---
            bg_color = "#ffffff"    # Xanh Navy đậm (GroupBox background)
            plot_bg = "#DEDFE0"     # Xanh Navy sâu hơn cho vùng vẽ
            text_color = "#040404"  # Chữ trắng
            grid_color = "#53a3cbe8"  # Lưới xanh chìm
            accent_blue = "#8bbccc" # Xanh lơ (Accent của app)

            label_style = {'color': text_color, 'font-size': '10pt'}
            all_graphs = [self.graph_pos, self.graph_vel, self.graph_error]
            titles = ["Vị trí", "Vận tốc", "Sai số"]

            for i, graph in enumerate(all_graphs):
                # 1. Đặt nền và Tiêu đề
                graph.setBackground(bg_color)
                graph.setTitle(titles[i], color=text_color, size='12pt')
                
                # 2. Xử lý các trục (Axes)
                for ax_name in ['left', 'bottom']:
                    axis = graph.getAxis(ax_name)
                    axis.setPen(pg.mkPen(text_color, width=1))
                    axis.setTextPen(text_color)
                    axis.labelStyle = label_style
                    axis._updateLabel()

                # 3. Khung viền và Lưới
                graph.getViewBox().setBorder(pg.mkPen(color=accent_blue, width=1.5))
                graph.showGrid(x=True, y=True, alpha=0.3)
                
                # 4. Tối ưu Render
                graph.setDownsampling(mode='peak') 
                graph.setClipToView(True)
                
                # 5. Legend (Chú thích)
                legend = graph.addLegend(offset=(10, 10), labelTextColor=text_color)
                if legend:
                    legend.setBrush(pg.mkBrush("#ffffffff")) # Nền legend trong suốt nhẹ

            # --- Đồ thị Vị trí (graph_pos) ---
            self.graph_pos.setAspectLocked(True)
            # Trục tọa độ tâm (0,0) màu xanh chìm
            self.graph_pos.addLine(x=0, pen=pg.mkPen(grid_color, width=1, style=QtCore.Qt.DotLine))
            self.graph_pos.addLine(y=0, pen=pg.mkPen(grid_color, width=1, style=QtCore.Qt.DotLine))
            
            self.curves['Pd'] = self.graph_pos.plot(pen=pg.mkPen(color='#50fa7b', width=2, style=QtCore.Qt.DashLine), name="Đặt")
            self.curves['P'] = self.graph_pos.plot(pen=pg.mkPen(color='#ff79c6', width=2.5), name="Thực")

            # --- Đồ thị Vận tốc (graph_vel) ---
            self.curves['V'] = self.graph_vel.plot(pen=pg.mkPen(color='#8be9fd', width=2), name="V")
            self.curves['W'] = self.graph_vel.plot(pen=pg.mkPen(color='#ffb86c', width=2), name="W")

            # --- Đồ thị Sai số (graph_error) ---
            self.curves['Ex'] = self.graph_error.plot(pen=pg.mkPen(color='#ff5555', width=2), name="Ex")
            self.curves['Ey'] = self.graph_error.plot(pen=pg.mkPen(color='#bd93f9', width=2), name="Ey")
            self.curves['Etheta'] = self.graph_error.plot(pen=pg.mkPen(color='#50fa7b', width=2), name="E_theta")

            # Liên kết trục X
            self.graph_vel.setXLink(self.graph_error)  
        else:
            # Cấu hình chung
            label_style = {'color': '#000000', 'font-size': '10pt'}
            all_graphs = [self.graph_pos, self.graph_vel, self.graph_error]
            
            # --- TỐI ƯU HÓA RENDER (QUAN TRỌNG) ---
        
            for graph in all_graphs:
                # 1. Xử lý số bị mờ: Chuyển màu chữ (Text) và trục (Axis) sang ĐEN hoàn toàn
                graph.getAxis('left').setTextPen('k')   # Số trục dọc màu đen
                graph.getAxis('left').setPen('k')       # Đường trục dọc màu đen
                graph.getAxis('bottom').setTextPen('k') # Số trục ngang màu đen
                graph.getAxis('bottom').setPen('k')     # Đường trục ngang màu đen
                
                # 2. Tạo khung viền (BOX) bao quanh biểu đồ
                # width=1.5 giúp viền rõ nét hơn
                graph.getViewBox().setBorder(pg.mkPen(color='k', width=1.5))
                graph.showGrid(x=True, y=True, alpha=0.1)
                # 1. Downsampling: Nếu dữ liệu quá dày đặc, tự động gộp bớt điểm ảnh
                graph.setDownsampling(mode='peak') 
                
                # 2. ClipToView: Chỉ vẽ những gì nằm trong khung nhìn (zoom in sẽ mượt hơn)
                graph.setClipToView(True)

            self.graph_pos.enableAutoRange(axis=pg.ViewBox.XYAxes, enable=False)

            self.graph_pos.setXRange(-1, 1) 
            self.graph_pos.setYRange(-1, 1)

            # self.graph_vel.enableAutoRange(axis=pg.ViewBox.XAxis, enable=True)
            # self.graph_vel.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
            
            # self.graph_error.enableAutoRange(axis=pg.ViewBox.XAxis, enable=True)
            # self.graph_error.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

            # # Đặt tầm nhìn ban đầu cho trục X (Ví dụ: 0 đến 20 giây)
            self.graph_vel.setXRange(0, GRAPH_X_RANGE)
            self.graph_error.setXRange(0, GRAPH_X_RANGE)

            # ==================================================
            # 1. ĐỒ THỊ VỊ TRÍ (graph_pos) - Pd, P
            # ==================================================
            self.graph_pos.setTitle("Vị trí", color='k', size='12pt')
            # Lưu ý: Nếu cậu vẽ quỹ đạo X-Y thì setLabel bottom là X, còn vẽ theo thời gian thì để Time
            self.graph_pos.setLabel('left', 'Position Y (m)', **label_style)
            self.graph_pos.setLabel('bottom', 'Position X', units='m', **label_style) 
            self.graph_pos.setBackground('w')
            
            # CỐ ĐỊNH LEGEND Ở GÓC TRÊN TRÁI
            self.graph_pos.addLegend(offset=(10, 10), labelTextColor='k') 

            # TẠO TÂM (0,0) VÀ GIỮ TỈ LỆ 1:1
            self.graph_pos.setAspectLocked(True) # Giữ tỉ lệ 1:1 để hình tròn/vuông không bị méo
            self.graph_pos.addLine(x=0, pen=pg.mkPen('k', width=1)) # Trục dọc qua tâm 0
            self.graph_pos.addLine(y=0, pen=pg.mkPen('k', width=1)) # Trục ngang qua tâm 0

            # Tạo Curves (Nét đậm hơn: width=2.5)
            self.curves['Pd'] = self.graph_pos.plot(pen=pg.mkPen(color='b', width=2.5, style=pg.QtCore.Qt.DashLine), name="Đặt")
            self.curves['P'] = self.graph_pos.plot(pen=pg.mkPen(color='r', width=2.5), name="Thực")

            # ==================================================
            # 2. ĐỒ THỊ VẬN TỐC (graph_vel) - V, W
            # ==================================================
            self.graph_vel.setTitle("Vận tốc", color='k', size='12pt')
            self.graph_vel.setLabel('left', 'Velocity', units='m/s | rad/s', **label_style)
            self.graph_vel.setLabel('bottom', 'Time', units='s', **label_style)
            self.graph_vel.setBackground('w')
            self.graph_vel.addLegend(offset=(10, 10), labelTextColor='k')

            # Tạo Curves (Dùng mã màu HEX để đậm hơn, tránh bị chói/mờ trên nền trắng)
            self.curves['V'] = self.graph_vel.plot(pen=pg.mkPen(color='#009900', width=2.5), name="V") # Xanh lá đậm
            self.curves['W'] = self.graph_vel.plot(pen=pg.mkPen(color='#FF5733', width=2.5), name="W") # Cam đậm

            # ==================================================
            # 3. ĐỒ THỊ SAI SỐ (graph_error) - Ex, Ey, Etheta
            # ==================================================
            self.graph_error.setTitle("Sai số", color='k', size='12pt')
            self.graph_error.setLabel('left', 'Error', units='m | rad', **label_style)
            self.graph_error.setLabel('bottom', 'Time', units='s', **label_style)
            self.graph_error.setBackground('w')
            self.graph_error.addLegend(offset=(10, 10), labelTextColor='k')

            # Tạo Curves (Màu đậm hơn)
            # Ex: Màu đỏ đậm thay vì đỏ tươi
            self.curves['Ex'] = self.graph_error.plot(pen=pg.mkPen(color='#CC0000', width=2), name="Ex")
            # Ey: Màu xanh dương đậm thay vì xanh lơ nhạt
            self.curves['Ey'] = self.graph_error.plot(pen=pg.mkPen(color='#0000CC', width=2), name="Ey")
            # Etheta: Màu xanh cổ vịt (Teal) thay vì Cyan nhạt
            self.curves['Etheta'] = self.graph_error.plot(pen=pg.mkPen(color='#008888', width=2), name="E_theta")

            # ==================================================
            # 4. LIÊN KẾT TRỤC HOÀNH (X-Link)
            # ==================================================
            self.graph_vel.setXLink(self.graph_error)    
    # ==================================================
    # DATA BUFFERS & PLOTTING
    # ==================================================
    def init_data_buffers(self):
        # Buffer cho dữ liệu vẽ đồ thị (Dữ liệu lịch sử)
        self.data_time = deque(maxlen=MAX_SAMPLES)
        self.data_pd_x = deque(maxlen=MAX_SAMPLES)
        self.data_pd_y = deque(maxlen=MAX_SAMPLES)
        self.data_p_x = deque(maxlen=MAX_SAMPLES)
        self.data_p_y = deque(maxlen=MAX_SAMPLES)
        self.data_v = deque(maxlen=MAX_SAMPLES)
        self.data_w = deque(maxlen=MAX_SAMPLES)
        self.data_ex = deque(maxlen=MAX_SAMPLES)
        self.data_ey = deque(maxlen=MAX_SAMPLES)
        self.data_etheta = deque(maxlen=MAX_SAMPLES)

        # --- MỚI: Hàng đợi chờ vẽ (Rendering Queue) ---
        # Nơi chứa các mẫu dữ liệu đã nhận từ ESP nhưng chưa kịp vẽ
        self.render_queue = deque(maxlen=MAX_SAMPLES) 
        
        # Cấu hình thời gian mẫu (theo code STM32 là 5ms)
        self.DT = SAMPLE_TIME 
        
        # Biến theo dõi thời gian nội bộ của Python để vẽ trục X liên tục
        self.current_plot_time = 0.0

    def _update_buffers(self, t, setpoint, current, control, errors):
        # Lưu dữ liệu
        self.data_time.append(t)
        self.data_pd_x.append(setpoint[0])
        self.data_pd_y.append(setpoint[1])
        self.data_p_x.append(current[0])
        self.data_p_y.append(current[1])
        self.data_v.append(control[0])
        self.data_w.append(control[1])
        self.data_ex.append(errors[0])
        self.data_ey.append(errors[1])
        self.data_etheta.append(errors[2])
        
        # ⭐ Chỉ đánh dấu là có data mới, KHÔNG vẽ ngay
        self.buffer_dirty = True

    def _update_plot_curves(self):
        # 1. Kiểm tra hàng đợi có dữ liệu không
        queue_len = len(self.render_queue)
        if queue_len == 0:
            return

        self.fps_counter += 1

        # 2. Tính toán số lượng điểm cần lấy ra để vẽ (Rate Limiting)
        # Mục tiêu: Lấy ra tương ứng với thời gian trôi qua của Timer (20ms)
        # Mỗi mẫu là 5ms -> Cần lấy 4 mẫu.
        
        points_to_pop = 4 

        # [Cơ chế chống trễ]: Nếu hàng đợi bị dồn ứ quá nhiều (do PC lag),
        # ta phải lấy nhiều hơn để đuổi kịp (Fast forward)
        if queue_len > 64: # Đang tồn đọng > 300ms dữ liệu
            points_to_pop = 16 # Lấy cả gói ra luôn
        elif queue_len > 32:
            points_to_pop = 8

        # Đừng lấy quá số lượng đang có
        points_to_pop = min(points_to_pop, queue_len)

        # 3. Lấy dữ liệu từ Queue đưa vào Main Buffer
        for _ in range(points_to_pop):
            # Lấy 1 mẫu từ hàng đợi: (Pdx, Pdy, Px, Py, V, W, Ex, Ey, Etheta)
            val = self.render_queue.popleft()
            
            # Cập nhật thời gian ảo
            self.current_plot_time += self.DT
            self.data_time.append(self.current_plot_time)

            # Append dữ liệu vào mảng vẽ
            self.data_pd_x.append(val[0])
            self.data_pd_y.append(val[1])
            self.data_p_x.append(val[2])
            self.data_p_y.append(val[3])
            self.data_v.append(val[4])
            self.data_w.append(val[5])
            self.data_ex.append(val[6])
            self.data_ey.append(val[7])
            self.data_etheta.append(val[8])

        # 4. Vẽ đồ thị (Logic cũ giữ nguyên nhưng tối ưu convert)
        # Mẹo: Chỉ convert phần mới hoặc dùng slice nếu data quá lớn, 
        # nhưng với PyQtGraph hiện đại, việc setData toàn bộ mảng numpy vẫn rất nhanh.
        
        t_arr = np.array(self.data_time, dtype=np.float32)
        
        # Vị trí
        self.curves['Pd'].setData(self.data_pd_x, self.data_pd_y)
        self.curves['P'].setData(self.data_p_x, self.data_p_y)
        
        # Vận tốc
        self.curves['V'].setData(t_arr, self.data_v)
        self.curves['W'].setData(t_arr, self.data_w)
        
        # Sai số
        self.curves['Ex'].setData(t_arr, self.data_ex)
        self.curves['Ey'].setData(t_arr, self.data_ey)
        self.curves['Etheta'].setData(t_arr, self.data_etheta)

        # Auto Scroll trục X
        current_page = int(self.current_plot_time / GRAPH_X_RANGE)
        
        # Tính giới hạn Min/Max của trục X dựa trên trang hiện tại
        min_x = current_page * GRAPH_X_RANGE
        max_x = min_x + GRAPH_X_RANGE

        # Cập nhật range (padding=0 để trục X không bị nhảy lung tung)
        # PyQtGraph xử lý việc này khá nhẹ, nhưng nếu muốn tối ưu hơn nữa, 
        # cậu có thể thêm if check xem range cũ có khác range mới không rồi mới set.
        self.graph_vel.setXRange(min_x, max_x, padding=0)
        self.graph_error.setXRange(min_x, max_x, padding=0)

        if len(self.data_time) > 0:
            
            # --- 1. Đồ thị Vận tốc (Chứa đường V và W) ---
            # Tìm min/max của cả 2 đường V và W
            min_v = min(self.data_v)
            max_v = max(self.data_v)
            min_w = min(self.data_w)
            max_w = max(self.data_w)
            
            # Lấy cực trị chung
            y_min_vel = min(min_v, min_w)
            y_max_vel = max(max_v, max_w)

            # Set range với Offset
            self.graph_vel.setYRange(y_min_vel - GRAPH_OFFSET, y_max_vel + GRAPH_OFFSET, padding=0)

            # --- 2. Đồ thị Sai số (Chứa Ex, Ey, Etheta) ---
            # Tìm min/max của 3 đường
            min_ex, max_ex = min(self.data_ex), max(self.data_ex)
            min_ey, max_ey = min(self.data_ey), max(self.data_ey)
            min_eth, max_eth = min(self.data_etheta), max(self.data_etheta)

            # Lấy cực trị chung
            y_min_err = min(min_ex, min_ey, min_eth)
            y_max_err = max(max_ex, max_ey, max_eth)

            # Set range với Offset
            self.graph_error.setYRange(y_min_err - GRAPH_OFFSET, y_max_err + GRAPH_OFFSET, padding=0)

    def _clear_plot(self):
        self.sample_count = 0

        # 1. Tạm dừng Timer vẽ
        self.timer_plot.stop()

        # 2. QUAN TRỌNG: Xóa sạch hàng đợi trung gian (Render Queue)
        # Nếu không xóa cái này, vừa bấm Clear xong nó lại vẽ nốt mấy mẫu cũ còn sót -> Nhìn rất dị
        if hasattr(self, 'render_queue'):
            self.render_queue.clear()
        
        # 3. QUAN TRỌNG: Reset thời gian vẽ về 0
        self.current_plot_time = 0.0

        # 4. Xóa các mảng dữ liệu chính
        self.data_time.clear()

        # Vị trí
        self.data_pd_x.clear()
        self.data_pd_y.clear()
        self.data_p_x.clear()
        self.data_p_y.clear()

        # Vận tốc
        self.data_v.clear()
        self.data_w.clear()

        # Sai số
        self.data_ex.clear()
        self.data_ey.clear()
        self.data_etheta.clear()

        # 5. Xóa trên giao diện đồ thị
        for curve in self.curves.values():
            curve.clear()
        
        # Reset trục X về mặc định (tuỳ chọn)
        self.graph_vel.setXRange(0, GRAPH_X_RANGE)
        self.graph_error.setXRange(0, GRAPH_X_RANGE)

        # 6. Cho chạy lại
        self.timer_plot.start()

    def _update_traj_label(self, text):
        # Dùng strip() để xóa khoảng trắng thừa cho chắc ăn
        mode = text.strip() 

        if mode == "Vuông":
            self.label_paramsTrajConfig.setText("Cạnh (m):")
        elif mode == "Tròn":
            self.label_paramsTrajConfig.setText("Bán kính (m):")
        elif mode == "Tam giác": # Ví dụ thêm
            self.label_paramsTrajConfig.setText("Chiều cao (m):")
        else:
            # Trường hợp mặc định
            self.label_paramsTrajConfig.setText("Tham số:")

    # ==================================================
    # TEST
    # ==================================================
    def simulate_incoming_data(self):
        """Hàm này giả vờ như có gói tin gửi đến từ ESP32"""
        import struct, math
        
        # 1. Tạo toạ độ giả (Chạy hình tròn bán kính 2m quanh tâm 4,4)
        self.sim_angle += 0.05
        fake_x = 4.0 + 2.0 * math.cos(self.sim_angle)
        fake_y = 4.0 + 2.0 * math.sin(self.sim_angle)
        
        # 2. Đóng gói giả thành 36 bytes (9 floats)
        # Pdx, Pdy, Px, Py, V, W, Ex, Ey, Etheta
        # Ta chỉ quan tâm Px (index 2) và Py (index 3)
        dummy_data = [0,0, fake_x, fake_y, 0,0,0,0,0]
        chunk = struct.pack('<9f', *dummy_data)
        
        # 3. Giả lập gói tin hoàn chỉnh (Header + Data + Footer)
        # BATCH_SIZE = 147 bytes. Ta cần điền cho đủ độ dài để hàm process không báo lỗi size
        # Header (2) + Data (36) + Footer (1) = 39 bytes (nếu SAMPLES_PER_BATCH=1)
        # Cậu cần padding thêm nếu BATCH_SIZE của cậu lớn hơn.
        # Ở đây ta gọi thẳng hàm xử lý logic cho gọn:
        
        # Gọi trực tiếp logic vẽ (Bỏ qua bước check size/header của process_binary_data)
        if self.stackedWidget_Main.currentIndex() == 1:
            self.map_widget.update_robot_pos(fake_x, fake_y)
  
if __name__ == "__main__":
    pg.setConfigOptions(antialias=False) # Tắt khử răng cưa (vẽ xấu xíu nhưng siêu nhanh)
    pg.setConfigOption('useOpenGL', True) # Bật OpenGL (QUAN TRỌNG NHẤT)

    app = QtWidgets.QApplication(sys.argv)
    
    # Khởi tạo class
    window = MyRobotApp()
    window.show()
    
    sys.exit(app.exec_())