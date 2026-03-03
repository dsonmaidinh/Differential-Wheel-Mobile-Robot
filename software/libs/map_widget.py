import numpy as np
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QMouseEvent
from PyQt5.QtCore import Qt, pyqtSignal

class MapWidget(QWidget):
    mapChanged = pyqtSignal()
    cellClicked = pyqtSignal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        # Thông số cơ bản
        self.rows = 40
        self.cols = 40
        self.map_w_m = 4.0
        self.map_h_m = 4.0
        
        # Dữ liệu hiển thị
        self.grid_data = None 
        self.start_node = None  # Toạ độ (r, c) điểm bắt đầu
        self.goal_nodes = []    # Danh sách các điểm đích [(r,c), (r,c)...]

        #  Path
        self.path_nodes = []

        self.editable = False
        self.robot_visible = False
        self.robot_x = 0.0
        self.robot_y = 0.0

    def load_map_data(self, map_array, map_w, map_h, start=None, goals=None, path=None):
        """
        Nhận toàn bộ dữ liệu từ File chính để vẽ.
        - start: Tuple (row, col) hoặc None
        - goals: List [(row, col), ...] hoặc None
        """
        self.grid_data = map_array.copy() # Chỉ copy để vẽ
        self.rows, self.cols = self.grid_data.shape
        self.map_w_m = map_w
        self.map_h_m = map_h
        
        # Cập nhật Start/Goal để paintEvent biết đường vẽ
        self.start_node = start
        self.goal_nodes = goals if goals is not None else []
        self.path_nodes = path if path is not None else []

        self.update() # Vẽ lại ngay

    def get_map_data(self):
        """Trả về dữ liệu hiện tại (đã chỉnh sửa)"""
        if self.grid_data is not None:
            return self.grid_data.copy()
        return None

    def set_editable(self, allow: bool):
        self.editable = allow
        self.update()

    def update_robot_pos(self, x_meter, y_meter):
        self.robot_x = x_meter
        self.robot_y = y_meter
        self.robot_visible = True
        self.update()

    def paintEvent(self, event):
        if self.grid_data is None: return

        painter = QPainter(self)
        w = self.width()
        h = self.height()
        cell_w = w / self.cols
        cell_h = h / self.rows

        # --- VẼ LƯỚI & CÁC Ô ---
        for r in range(self.rows):
            for c in range(self.cols):
                x = c * cell_w
                y = r * cell_h
                
                # [QUAN TRỌNG] THỨ TỰ ƯU TIÊN MÀU SẮC
                # 1. Điểm Start (Vàng)
                if self.start_node == (r, c):
                    brush = QBrush(QColor("yellow"))
                
                # 2. Điểm Goal (Xanh Lá)
                elif (r, c) in self.goal_nodes:
                    brush = QBrush(QColor("green"))
                
                # 3. Tường (Đen) - Nếu không phải Start/Goal mà data=1
                elif self.grid_data[r, c] == 1:
                    brush = QBrush(Qt.black)
                
                # 4. Đường đi (Trắng)
                else:
                    brush = QBrush(Qt.white)
                
                painter.setBrush(brush)
                
                # Viền ô
                if self.editable:
                    painter.setPen(QPen(QColor("#0078d7"), 1))
                else:
                    painter.setPen(QPen(QColor(220, 220, 220), 1))
                
                painter.drawRect(int(x), int(y), int(cell_w)+1, int(cell_h)+1)

                # [OPTION] Vẽ số thứ tự cho Goal (để biết điểm nào đến trước)
                if (r, c) in self.goal_nodes:
                    idx = self.goal_nodes.index((r, c)) + 1
                    painter.setPen(Qt.black)
                    painter.drawText(int(x), int(y), int(cell_w), int(cell_h), 
                                     Qt.AlignCenter, str(idx))

        # --- VẼ ĐƯỜNG ĐI (PATH) ---
        if len(self.path_nodes) > 1:
            painter.setPen(QPen(QColor("purple"), 2, Qt.DashLine)) # Màu tím, nét đứt
            
            # Nối các điểm lại với nhau
            for i in range(len(self.path_nodes) - 1):
                r1, c1 = self.path_nodes[i]
                r2, c2 = self.path_nodes[i+1]
                
                # Đổi sang pixel (lấy tâm ô vuông)
                p1_x = c1 * cell_w + cell_w/2
                p1_y = r1 * cell_h + cell_h/2
                p2_x = c2 * cell_w + cell_w/2
                p2_y = r2 * cell_h + cell_h/2
                
                painter.drawLine(int(p1_x), int(p1_y), int(p2_x), int(p2_y))

        # 2. VẼ ROBOT
        if self.robot_visible:
            scale_x = w / self.map_w_m
            scale_y = h / self.map_h_m
            px = self.robot_x * scale_x
            py = h - (self.robot_y * scale_y) 

            painter.setBrush(QBrush(Qt.red))
            painter.setPen(QPen(Qt.white, 2))
            painter.drawEllipse(int(px - 6), int(py - 6), 12, 12)

    def mousePressEvent(self, event: QMouseEvent):
        if not self.editable: return

        w = self.width()
        h = self.height()
        cell_w = w / self.cols
        cell_h = h / self.rows

        col_clicked = int(event.x() // cell_w)
        row_clicked = int(event.y() // cell_h)

        if 0 <= row_clicked < self.rows and 0 <= col_clicked < self.cols:
            # [THAY ĐỔI QUAN TRỌNG]: 
            # Không tự ý sửa self.grid_data tại đây nữa!
            # Chỉ bắn tín hiệu về Main App để Main App quyết định làm gì
            self.cellClicked.emit(row_clicked, col_clicked)