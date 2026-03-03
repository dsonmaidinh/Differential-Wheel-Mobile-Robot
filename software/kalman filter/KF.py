class KalmanFilter(object):
     
    # x0 - initial guess of the state vector 
    # P0 - initial guess of the covariance matrix of the state estimation error
    # A,B,C - system matrices describing the system model
    # Q - covariance matrix of the process noise 
    # R - covariance matrix of the measurement noise
     
    def __init__(self,x0,P0,A,B,C,Q,R):
         
        # initialize vectors and matrices
        self.x0=x0
        self.P0=P0
        self.A=A
        self.B=B
        self.C=C
        self.Q=Q
        self.R=R
         
        # this variable is used to track the current time step k of the estimator 
        # after every measurement arrives, this variables is incremented for +1 
        self.currentTimeStep=0
         
        # this list is used to store the a posteriori estimates xk^{+} starting from the initial estimate 
        # note: list starts from x0^{+}=x0 - where x0 is an initial guess of the estimate
        #
        # Ký hiệu:  x̂₀⁺ = x0  (điều kiện ban đầu - giá trị đoán ban đầu)
        self.estimates_aposteriori=[]
        self.estimates_aposteriori.append(x0)
         
        # this list is used to store the a apriori estimates xk^{-} starting from x1^{-}
        # note: x0^{-} does not exist, that is, the list starts from x1^{-}
        #
        # Ký hiệu:  x̂ₖ⁻  (ước lượng TRƯỚC khi có phép đo tại bước k)
        # List bắt đầu từ x̂₁⁻, vì x̂₀⁻ không tồn tại (bước 0 chỉ có x̂₀⁺ = x0)
        self.estimates_apriori=[]
         
        # this list is used to store the a posteriori estimation error covariance matrices Pk^{+}
        # note: list starts from P0^{+}=P0, where P0 is the initial guess of the covariance
        #
        # Ký hiệu:  P₀⁺ = P0  (điều kiện ban đầu - mức độ "mông lung" ban đầu)
        # P càng lớn → hệ thống càng không tự tin về ước lượng hiện tại
        self.estimationErrorCovarianceMatricesAposteriori=[]
        self.estimationErrorCovarianceMatricesAposteriori.append(P0)
         
        # this list is used to store the a priori estimation error covariance matrices Pk^{-}
        # note: list starts from P1^{-}, that is, P0^{-} does not exist
        #
        # Ký hiệu:  Pₖ⁻  (độ bất định TRƯỚC khi có phép đo tại bước k)
        # List bắt đầu từ P₁⁻, vì P₀⁻ không tồn tại
        self.estimationErrorCovarianceMatricesApriori=[]
         
        # this list is used to store the gain matrices Kk
        #
        # Ký hiệu:  Kₖ  (Kalman Gain - trọng số quyết định tin mô hình hay tin cảm biến)
        # Kₖ lớn → tin cảm biến nhiều | Kₖ nhỏ → tin mô hình nhiều
        self.gainMatrices=[]
          
        # this list is used to store prediction errors error_k=y_k-C*xk^{-}
        #
        # Ký hiệu:  ỹₖ = yₖ - C·x̂ₖ⁻   (Innovation - sai lệch đo lường)
        # Ý nghĩa: chênh lệch giữa đo thực tế và đo dự đoán từ mô hình
        self.errors=[]
         
    # =========================================================================
    # BƯỚC 1: PREDICT (Dự đoán) — "Nhắm mắt đi tiếp, chưa nhìn cảm biến"
    # =========================================================================
    # Lan truyền x̂ₖ₋₁⁺ qua mô hình để tính x̂ₖ⁻
    # Lan truyền Pₖ₋₁⁺ qua phương trình hiệp phương sai để tính Pₖ⁻
    # Cuối hàm: tăng currentTimeStep lên +1
    def propagateDynamics(self,inputValue):
         
        # --- Dự đoán trạng thái (State Prediction) ---
        # Công thức:  x̂ₖ⁻ = A·x̂ₖ₋₁⁺ + B·uₖ
        #
        #   A·x̂ₖ₋₁⁺  → trạng thái cũ được "đẩy" qua ma trận động học
        #   B·uₖ      → đóng góp của đầu vào điều khiển (v, ω từ Encoder)
        #   w̄ₖ = 0    → nhiễu quá trình bị bỏ qua (mean = 0)
        xk_minus=self.A*self.estimates_aposteriori[self.currentTimeStep]+self.B*inputValue

        # --- Lan truyền hiệp phương sai (Covariance Propagation) ---
        # Công thức:  Pₖ⁻ = A·Pₖ₋₁⁺·Aᵀ + Q
        #
        #   A·Pₖ₋₁⁺·Aᵀ → độ bất định cũ được "biến đổi" qua ma trận A
        #   + Q          → cộng thêm nhiễu quá trình (trượt bánh, sai số cơ khí)
        #   → Pₖ⁻ > Pₖ₋₁⁺ : độ bất định TĂNG lên sau mỗi bước predict (hệ thống "mông lung" hơn)
        Pk_minus=self.A*self.estimationErrorCovarianceMatricesAposteriori[self.currentTimeStep]*(self.A.T)+self.Q
         
        self.estimates_apriori.append(xk_minus)
        self.estimationErrorCovarianceMatricesApriori.append(Pk_minus)
         
        self.currentTimeStep=self.currentTimeStep+1
     
    # =========================================================================
    # BƯỚC 2: UPDATE (Cập nhật) — "Mở mắt nhìn cảm biến, sửa sai"
    # =========================================================================
    # Phải gọi SAU propagateDynamics() vì cần x̂ₖ⁻ và Pₖ⁻ đã được tính
    def computeAposterioriEstimate(self,currentMeasurement):
        import numpy as np

        # --- Tính Kalman Gain ---
        # Công thức:  Kₖ = Pₖ⁻·Cᵀ·(C·Pₖ⁻·Cᵀ + R)⁻¹
        #
        #   Pₖ⁻·Cᵀ        → độ bất định của dự đoán chiếu lên không gian đo lường
        #   C·Pₖ⁻·Cᵀ + R  → tổng nhiễu: nhiễu dự đoán + nhiễu cảm biến (= Sₖ)
        #   (Sₖ)⁻¹         → nghịch đảo → "chia tỷ lệ"
        #
        #   Nếu R nhỏ (cảm biến tốt)  → Kₖ lớn → tin cảm biến
        #   Nếu R lớn (cảm biến kém)  → Kₖ nhỏ → tin mô hình
        Kk=self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1]*(self.C.T)*np.linalg.inv(self.R+self.C*self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1]*(self.C.T))
         
        # --- Tính sai lệch đo lường (Innovation) ---
        # Công thức:  ỹₖ = yₖ - C·x̂ₖ⁻
        #
        #   yₖ      → giá trị cảm biến đo được thực tế (VD: θ_IMU)
        #   C·x̂ₖ⁻  → giá trị cảm biến dự đoán từ mô hình (VD: θ_predict)
        #   ỹₖ      → sai lệch giữa thực tế và dự đoán
        error_k=currentMeasurement-self.C*self.estimates_apriori[self.currentTimeStep-1]

        # --- Cập nhật ước lượng hậu nghiệm (A Posteriori State Update) ---
        # Công thức:  x̂ₖ⁺ = x̂ₖ⁻ + Kₖ·ỹₖ
        #
        #   x̂ₖ⁻    → vị trí dự đoán từ mô hình
        #   Kₖ·ỹₖ  → phần bù sai lệch (Kalman Gain × Innovation)
        #   x̂ₖ⁺    → ước lượng cuối cùng sau khi đã "nhìn" cảm biến
        xk_plus=self.estimates_apriori[self.currentTimeStep-1]+Kk*error_k
         
        # --- Cập nhật hiệp phương sai hậu nghiệm (A Posteriori Covariance Update) ---
        # Công thức (Joseph form - ổn định số hơn):
        #   Pₖ⁺ = (I - Kₖ·C)·Pₖ⁻·(I - Kₖ·C)ᵀ + Kₖ·R·Kₖᵀ
        #
        #   (I - Kₖ·C)       → ma trận "co lại" độ bất định sau khi dùng cảm biến
        #   Kₖ·R·Kₖᵀ        → thêm lại phần nhiễu cảm biến vào (để tránh underestimate)
        #   → Pₖ⁺ < Pₖ⁻     : độ bất định GIẢM xuống sau khi có phép đo (hệ thống "tự tin" hơn)
        #
        # Lưu ý: Dạng đơn giản hơn là  Pₖ⁺ = (I - Kₖ·C)·Pₖ⁻
        #        nhưng dạng Joseph form ở đây bền vững hơn về mặt số học (numerically stable)
        IminusKkC=np.matrix(np.eye(self.x0.shape[0]))-Kk*self.C
        Pk_plus=IminusKkC*self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1]*(IminusKkC.T)+Kk*(self.R)*(Kk.T)
         
        # update the lists that store the vectors and matrices
        self.gainMatrices.append(Kk)
        self.errors.append(error_k)
        self.estimates_aposteriori.append(xk_plus)
        self.estimationErrorCovarianceMatricesAposteriori.append(Pk_plus)