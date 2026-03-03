import numpy as np
import heapq

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        
        self.g = 0 # Chi phí từ điểm xuất phát đến node hiện tại
        self.h = 0 # Chi phí ước lượng từ node hiện tại đến đích (Heuristic)
        self.f = 0 # Tổng chi phí (f = g + h)

    def __eq__(self, other):
        return self.position == other.position
    
    # Hàm so sánh để Priority Queue (heapq) biết node nào nhỏ hơn
    def __lt__(self, other):
        return self.f < other.f

def astar_search(maze, start, end):
    """
    Hàm tìm đường A* cơ bản
    - maze: Mảng 2D numpy (0: đường, 1: tường)
    - start: Tuple (row, col)
    - end: Tuple (row, col)
    - Return: List các tuple [(r,c), (r,c)...] hoặc None nếu không tìm thấy
    """
    
    # 1. Kiểm tra điều kiện biên
    rows, cols = maze.shape
    if not (0 <= start[0] < rows and 0 <= start[1] < cols): return None
    if not (0 <= end[0] < rows and 0 <= end[1] < cols): return None
    if maze[start[0]][start[1]] == 1 or maze[end[0]][end[1]] == 1:
        print("Lỗi: Điểm Start hoặc End nằm trong tường!")
        return None

    # Tạo Node Start và End
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Khởi tạo 2 danh sách
    open_list = []      # Các node cần xem xét
    closed_set = set()  # Các node đã xem xét xong (dùng Set để tra cứu nhanh)

    # Thêm start node vào open_list
    heapq.heappush(open_list, start_node)

    # Vòng lặp chính
    iteration = 0
    max_iterations = (rows * cols) * 2 # Tránh treo máy nếu map quá phức tạp

    while len(open_list) > 0:
        iteration += 1
        if iteration > max_iterations:
            print("Quá thời gian tìm kiếm!")
            return None

        # Lấy node có F thấp nhất
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        # KIỂM TRA ĐÍCH
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Đảo ngược danh sách (Start -> End)

        # QUÉT CÁC Ô LÂN CẬN (8 hướng: Trên, Dưới, Trái, Phải, 4 Chéo)
        # (Delta Row, Delta Col)
        neighbors = [
            (0, -1), (0, 1), (-1, 0), (1, 0), # Ngang dọc (Chi phí 1)
            (-1, -1), (-1, 1), (1, -1), (1, 1) # Chéo (Chi phí 1.414)
        ]

        for next_position in neighbors:
            # Tọa độ hàng xóm
            node_position = (current_node.position[0] + next_position[0], 
                             current_node.position[1] + next_position[1])

            # 1. Kiểm tra trong map
            if not (0 <= node_position[0] < rows and 0 <= node_position[1] < cols):
                continue

            # 2. Kiểm tra vật cản (Tường = 1)
            if maze[node_position[0]][node_position[1]] == 1:
                continue
            
            # [NÂNG CAO] Kiểm tra cắt góc (nếu đi chéo)
            # Nếu 2 bên cạnh là tường thì không được đi xuyên khe hẹp chéo
            if abs(next_position[0]) == 1 and abs(next_position[1]) == 1: # Đang đi chéo
                # Kiểm tra 2 ô kề
                if maze[current_node.position[0] + next_position[0]][current_node.position[1]] == 1 or \
                   maze[current_node.position[0]][current_node.position[1] + next_position[1]] == 1:
                    continue

            # 3. Kiểm tra đã đi qua chưa
            if node_position in closed_set:
                continue

            # TẠO NODE MỚI
            new_node = Node(current_node, node_position)

            # Tính toán chi phí
            # Đi thẳng = 1, Đi chéo = 1.414 (căn 2)
            move_cost = 1.414 if abs(next_position[0]) == 1 and abs(next_position[1]) == 1 else 1.0
            
            new_node.g = current_node.g + move_cost
            
            # Heuristic (Euclidean Distance bình phương để đỡ tính căn)
            new_node.h = ((new_node.position[0] - end_node.position[0]) ** 2) + \
                         ((new_node.position[1] - end_node.position[1]) ** 2)
            new_node.f = new_node.g + new_node.h

            # Kiểm tra xem node này có trong open_list với chi phí thấp hơn chưa
            # (Phần này tối ưu tốc độ, có thể bỏ qua nếu map nhỏ, nhưng nên giữ)
            add_to_list = True
            for open_node in open_list:
                if new_node == open_node and new_node.g >= open_node.g:
                    add_to_list = False
                    break
            
            if add_to_list:
                heapq.heappush(open_list, new_node)

    return None # Không tìm thấy đường