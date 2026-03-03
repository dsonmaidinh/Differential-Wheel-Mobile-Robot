#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// =================================================================================
// PHẦN CẤU HÌNH HỆ THỐNG (USER CONFIGURATION)
// =================================================================================
// CẤU HÌNH PHẦN CỨNG (HARDWARE)
#define CPU_FREQ_MHZ            240         // Ép xung CPU (MHz)
#define STM32Serial             Serial2     // Cổng Serial kết nối STM32
#define UART_RX_PIN             40          // Chân RX
#define UART_TX_PIN             39          // Chân TX
#define BATTERY_PIN             3
#define STM32_BAUDRATE          225000      // Tốc độ giao tiếp với STM32 (High Speed)
#define STM32_SEND_INTERVAL_MS  40          // Chu kỳ gửi lệnh xuống STM32 (ms)
#define DEBUG_BAUDRATE          115200      // Tốc độ Serial Debug (USB)

// CẤU HÌNH WIFI & SERVER (NETWORK)
#define ROBOT_WIFI_SSID               "DMRobot_Net" // Tên WiFi Robot (chế độ AP)
#define ROBOT_WIFI_PASSWORD           "12345678"    // Mật khẩu WiFi Robot (chế độ AP)
#define MY_WIFI_SSID                  "DONG SON"    // Tên WiFi nhà (chế độ STA)
#define MY_WIFI_PASSWORD              "05041901"    // Mật khẩu WiFi nhà (chế độ STA)
#define MY_HOSTNAME                   "myrobot"     // Tên mDNS (chế độ STA)
#define WS_PORT                       80            // Cổng WebSocket
#define WS_PATH                       "/ws"         // Đường dẫn WebSocket
#define WS_SEND_INTERVAL_MS           80          

// CẤU HÌNH GIAO THỨC (PROTOCOL DEFINITIONS)
#define PROTOCOL_BATCH_HEADER_1       0xAA          // Header gói Batch từ STM32
#define PROTOCOL_BATCH_HEADER_2       0x55          // Header gói Batch từ STM32
#define PROTOCOL_BATCH_FOOTER         0xEE          // Footer gói Batch từ STM32
#define PROTOCOL_CMD_HEADER_1         0xAA          // Header lệnh gửi xuống STM32
#define PROTOCOL_CMD_HEADER_2         0xBB          // Header lệnh gửi xuống STM32
#define PROTOCOL_CHECKSUM             0xCC          // Byte checksum lệnh gửi xuống 

#define PROTOCOL_MAP_HEADER_1         0xAA          // Header lệnh gửi xuống STM32
#define PROTOCOL_MAP_HEADER_2         0xBB          // Header lệnh gửi xuống STM32

// CẤU HÌNH KÍCH THƯỚC DỮ LIỆU (DATA SIZING)
#define SAMPLES_PER_BATCH       4           // Số mẫu trong 1 gói Batch (Quan trọng)
#define TX_BUFFER_SIZE          32          // Kích thước buffer gửi lệnh xuống STM32
#define UART_RX_BUFFER_SIZE     8192        // Kích thước Buffer nhận phần cứng UART (ESP32 - có thể để lớn)
// 
#define DebugCode false                      // [TRUE]: Bật in Debug, [FALSE]: Tắt toàn bộ để tối ưu
#define MY_WIFI   true                      // [TRUE]: Kết nối WiFi nhà (STA), [FALSE]: Tự phát WiFi (AP)

// ==========================================
// ĐỊNH NGHĨA STRUCT & BATCH SIZE
// ==========================================
#pragma pack(push, 1)

// Struct dữ liệu 1 mẫu (9 float * 1 float (4 bytes) = 36 bytes)
typedef struct { 
    float Pdx, Pdy, Px, Py, V, W, Ex, Ey, Etheta; 
} DMAData_t;

// Struct Batch nhận từ STM32 (Header + n mẫu + Footer)
typedef struct { 
    uint8_t batch_header[2]; 
    DMAData_t samples[SAMPLES_PER_BATCH]; 
    uint8_t batch_footer; 
} RxBatchData_t;

// Struct lệnh ngắn (4 bytes)
typedef struct {
    uint8_t header[2]; 
    uint8_t cmd;       
    uint8_t checksum;  
} TxCMDData_t;

// Struct cấu hình (13 bytes)
typedef struct {
    uint8_t header[2]; 
    uint8_t cmd;       
    uint8_t shape;
    float param;
    float vel;
    uint8_t checksum;  
} TxConfigData_t;

typedef struct {
    uint8_t header[2];
    uint8_t cmd;
    uint8_t wifi_status;
    uint8_t battery_status;
    uint8_t checksum;
} TxStatusData_t;

typedef union {
    uint8_t raw[TX_BUFFER_SIZE];// Đệm đủ cho Queue
    TxCMDData_t cmdFrame;       // Gói lệnh ngắn (4 byte)
    TxConfigData_t configFrame; // Gói cấu hình (13 byte)
    TxStatusData_t statusFrame;
} TxPacket_t;

#pragma pack(pop)

// ==========================================
// BIẾN TOÀN CỤC & QUEUE
// ==========================================
AsyncWebServer server(WS_PORT);
AsyncWebSocket ws(WS_PATH);

// --- QUEUES & SYNC ---
QueueHandle_t rxQueue;          // UART → WebSocket
QueueHandle_t txQueue;          // WebSocket → UART (MỚI!)
SemaphoreHandle_t wifiMutex;    // Khóa truy cập WiFi

TxCMDData_t txCMDBuffer;
TxConfigData_t txConfigBuffer;

#if DebugCode
    // --- STATS ---
    volatile uint32_t uart_rx_count = 0;
    volatile uint32_t uart_tx_count = 0;
    volatile uint32_t ws_sent_count = 0;
    volatile uint32_t queue_drops = 0;
    volatile uint32_t ws_blocked = 0;
#endif
volatile bool isRun = true;      // Chờ app gửi lệnh mới bắt đầu đọc UART
size_t rxDataSize = sizeof(RxBatchData_t);

// --- ENUMS ---
typedef enum { CMD_STOP, CMD_RUN, CMD_RETURN_HOME, CMD_SET_TRAJECTORY, CMD_SET_PATH, CMD_STATUS } RobotCommand_t;
typedef enum { CIRCLE, SQUARE, TRIANGLE } RobotShape_t;
// Thêm vào sau các Enum có sẵn
typedef enum {
    WIFI_STATUS_CONNECTING = 0x00,
    WIFI_STATUS_CONNECTED  = 0x01,
} WifiStatus_t;

typedef enum {
    BATTERY_STATUS_100 = 0x00,
    BATTERY_STATUS_60  = 0x01,
    BATTERY_STATUS_50  = 0x02,
} BatteryStatus_t;
typedef enum {
    BAT_TYPE_2S = 0,
    BAT_TYPE_3S = 1
} BatteryType_t;
#define MY_BATTERY_TYPE    BAT_TYPE_3S
// Biến lưu trạng thái hiện tại để gửi
volatile uint8_t current_wifi_stat = WIFI_STATUS_CONNECTING;
volatile uint8_t current_bat_stat  = BATTERY_STATUS_100;

// Macro debug cho gọn
#if DebugCode
    #define DBG(...) Serial.printf(__VA_ARGS__)
    #define DBG_LN(...) Serial.println(__VA_ARGS__)
#endif

void SendStatusToSTM32();
void WiFiEvent(WiFiEvent_t event);

// ================================================================
// WEBSOCKET HANDLER - GỬI VÀO QUEUE THAY VÌ GỬI TRỰC TIẾP
// ================================================================
static inline uint8_t CalculateChecksum(uint8_t *pData, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += pData[i];
    }
    return sum;
}
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg; 

    // [SỬA ĐỔI 1] Bỏ điều kiện info->opcode == WS_TEXT ở đây
    // Để chấp nhận cả WS_TEXT và WS_BINARY
    if (!(info->final && info->index == 0 && info->len == len)) return;

    // ============================================================
    // TRƯỜNG HỢP 1: DỮ LIỆU VĂN BẢN (JSON COMMANDS)
    // ============================================================
    if (info->opcode == WS_TEXT) {
        data[len] = 0; 
        JsonDocument doc;
        if (deserializeJson(doc, (char*)data) != DeserializationError::Ok) return;

        int cmd = doc["cmd"];
        
        TxPacket_t txPacket; 
        memset(txPacket.raw, 0, sizeof(txPacket.raw));

        switch(cmd) {
            case CMD_STOP:
                txPacket.cmdFrame.header[0] = PROTOCOL_CMD_HEADER_1;
                txPacket.cmdFrame.header[1] = PROTOCOL_CMD_HEADER_2;
                txPacket.cmdFrame.cmd = (uint8_t)cmd;
                txPacket.cmdFrame.checksum = CalculateChecksum(txPacket.raw, offsetof(TxCMDData_t, checksum));  

                xQueueSend(txQueue, &txPacket, 0); 
                isRun = false;                    
                break;

            case CMD_RUN:
            case CMD_RETURN_HOME:
                txPacket.cmdFrame.header[0] = PROTOCOL_CMD_HEADER_1;
                txPacket.cmdFrame.header[1] = PROTOCOL_CMD_HEADER_2;
                txPacket.cmdFrame.cmd = (uint8_t)cmd;
                txPacket.cmdFrame.checksum = CalculateChecksum(txPacket.raw, offsetof(TxCMDData_t, checksum));

                xQueueSend(txQueue, &txPacket, 0);
                isRun = true;
                break;
                
            case CMD_SET_TRAJECTORY:
                txPacket.configFrame.header[0] = PROTOCOL_CMD_HEADER_1;
                txPacket.configFrame.header[1] = PROTOCOL_CMD_HEADER_2;
                txPacket.configFrame.cmd = CMD_SET_TRAJECTORY;
                txPacket.configFrame.shape = doc["Shape"].is<int>() ? (uint8_t)doc["Shape"] : SQUARE;
                txPacket.configFrame.param = doc["Para"].is<float>() ? doc["Para"] : 0.0f;
                txPacket.configFrame.vel = doc["Vel"].is<float>() ? doc["Vel"] : 0.0f;
                txPacket.configFrame.checksum = CalculateChecksum(txPacket.raw, offsetof(TxConfigData_t, checksum));

                xQueueSend(txQueue, &txPacket, 0);
                
                #if DebugCode
                    DBG_LN("📲 App Config Trajectory -> Queue");
                #endif
                break;
        }
    }

    // ============================================================
    // TRƯỜNG HỢP 2: DỮ LIỆU NHỊ PHÂN (PATH DATA - CMD_SET_PATH)
    // ============================================================
    else if (info->opcode == WS_BINARY) {
        // Kiểm tra sơ bộ Header để chắc chắn là gói tin của mình (AA 55)
        // Header 1 (Byte 0) và Header 2 (Byte 1)
        if (len > 2 && data[0] == PROTOCOL_MAP_HEADER_1 && data[1] == PROTOCOL_MAP_HEADER_2) {
            
            // [PHƯƠNG ÁN PASS-THROUGH]
            // Gửi thẳng xuống STM32 qua UART ngay lập tức
            // Không dùng Queue vì gói tin này rất lớn (> 100 byte), Queue TxPacket_t không chứa nổi
            Serial2.write(data, len);

            #if DebugCode
                DBG("📦 Received Binary Path! Size: ");
                DBG_LN(len);
                DBG_LN("--> Forwarded directly to STM32 via UART2");
            #endif
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            #if DebugCode
                DBG("✅ Client #%u connected\n", client->id());
            #endif
            if (client->client()) {
                client->client()->setNoDelay(true);
            }
            break;
        case WS_EVT_DISCONNECT:
            #if DebugCode
                DBG("❌ Client #%u disconnected\n", client->id());
            #endif
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        default:
            break;
    }
}

// ================================================================
// TASK 1: ĐỌC UART (CORE 1) - THÊM THROTTLING
// ================================================================
void TaskUARTReceiver(void *pvParameters) {
    RxBatchData_t batchBuffer;
    #if DebugCode
        static uint32_t rxDataCount = 0;
        static uint32_t last_receive_time = 0;
        static uint32_t sync_count = 0;
        static uint32_t check_uart_task_count = 0;
        static uint32_t last_check_uart_task_time = 0;
        DBG_LN("📡 UART RX Task started (Core 1)");
    #endif

    uint8_t head[2];
    
    STM32Serial.setTimeout(20); // Giảm timeout xuống thấp hơn nữa để phản ứng nhanh

    while (1) {
        // --- PHẦN 1: KEEP ALIVE LOG ---
        #if DebugCode
            if(millis() - last_check_uart_task_time >= 2000){
                DBG("🧐 UART RX Task is alive - Count: %lu\n", ++check_uart_task_count);
                last_check_uart_task_time = millis();
            }
        #endif
        
        if (!isRun) {
            // [FIX LỖI] Xả sạch buffer để khi chạy lại không dính dữ liệu cũ
            while(STM32Serial.available()) {
                STM32Serial.read();
            }
            vTaskDelay(100 / portTICK_PERIOD_MS); // Ngủ sâu để tiết kiệm CPU
            continue; // Quay lại đầu vòng lặp
        }
        // --- PHẦN 2: LOGIC ĐỌC DỮ LIỆU ---
        // [FIX 1] Bỏ điều kiện kiểm tra thời gian (millis - last >= INTERVAL).
        // Lý do: Có dữ liệu là phải múc ngay, không chờ đủ giờ mới múc -> gây đầy buffer.
        if (STM32Serial.available() >= sizeof(RxBatchData_t)) {
            
            // Peek: Xem trộm byte đầu tiên xem có phải AA không
            if (STM32Serial.peek() == PROTOCOL_BATCH_HEADER_1) {
                // Đọc thử 2 byte Header
                STM32Serial.readBytes(head, 2);

                if (head[1] == PROTOCOL_BATCH_HEADER_2) {
                    // Header chuẩn (AA 55) -> Đọc phần thân
                    uint8_t* ptr = (uint8_t*)&batchBuffer;
                    
                    // Gán lại header vào struct để tí nữa tính checksum hoặc gửi đi
                    batchBuffer.batch_header[0] = PROTOCOL_BATCH_HEADER_1;
                    batchBuffer.batch_header[1] = PROTOCOL_BATCH_HEADER_2;

                    size_t bytesToRead = sizeof(RxBatchData_t) - 2;
                    size_t count = STM32Serial.readBytes(ptr + 2, bytesToRead);

                    // [FIX 2] Kiểm tra Footer 
                    if (count == bytesToRead && batchBuffer.batch_footer == PROTOCOL_BATCH_FOOTER) {
                        
                        // [FIX 3 - QUAN TRỌNG] Thêm Checksum để chống nhiễu khi nạp code
                        // (Giả sử byte cuối cùng của samples là checksum hoặc footer đóng vai trò check)
                        // Nếu cậu không có checksum thì Footer 0xEE là chốt chặn cuối cùng.
                        #if DebugCode
                            uart_rx_count++;
                        #endif
                        
                        #if DebugCode 
                            // Chỉ log khi nhận thành công
                            DBG("Đã nhận: %d Batch | dt: %lu ms\n", ++rxDataCount, millis() - last_receive_time);
                            last_receive_time = millis();
                        #endif

                        // Đẩy vào Queue
                        if (uxQueueSpacesAvailable(rxQueue) == 0) { // nếu đầy thì bỏ qua
                            RxBatchData_t dummy;
                            xQueueReceive(rxQueue, &dummy, 0); 
                            #if DebugCode
                                queue_drops++;
                            #endif
                        }
                        xQueueSend(rxQueue, &batchBuffer, 0);
                        
                    } else {                        
                        #if DebugCode 
                            DBG("⚠️ Lỗi Footer/Size (Count: %d, Exp: %d, Footer: %02X) -> Skip packet\n", 
                                count, bytesToRead, batchBuffer.batch_footer);
                            sync_count++;
                        #endif
                        
                        // Tùy chọn: Xả nhẹ 1 byte để tránh lặp vô hạn nếu kẹt đúng byte AA giả
                        // Nhưng thường logic 'else' ở dưới cùng sẽ lo việc này.
                    }
                } else {
                    // Header 1 là AA, nhưng Header 2 không phải 55
                    // Đã đọc 2 byte ra rồi -> Coi như bỏ qua gói rác này.
                    #if DebugCode 
                        DBG("⚠️ Header 2 sai: %02X -> Skip\n", head[1]);
                    #endif
                }
            } else {
                // Byte đầu tiên không phải AA -> Đọc bỏ 1 byte để dịch cửa sổ tìm kiếm
                STM32Serial.read(); 
                #if DebugCode 
                   DBG("Rác: Bỏ qua 1 byte\n"); // Comment lại cho đỡ spam log
                #endif
            }
            continue;
        }
        else {
            vTaskDelay(1); // Yield cực ngắn
        }
    }
}

// ================================================================
// TASK 2: GỬI WEBSOCKET (CORE 0) - KHÔNG BLOCK
// ================================================================
// CODE TEST STRESS WIFI (Không dùng Queue từ UART)
void TaskWebSocketSender(void *pvParameters) {
    RxBatchData_t dataToSend;
    #if DebugCode
        DBG_LN("📤 WebSocket Sender: Real Mode (Queue -> WiFi)");
    #endif

    while (1) {
        // Chờ dữ liệu từ Queue (Block tối đa 50ms)
        // Nếu STM32 gửi chuẩn, Queue sẽ có dữ liệu đều đặn mỗi 50ms (do UART gom 25 mẫu * 2ms)
        if (xQueueReceive(rxQueue, &dataToSend, pdMS_TO_TICKS(STM32_SEND_INTERVAL_MS)) == pdTRUE) {
            
            if (ws.count() > 0) {
                // Kiểm tra xem socket có đang bận không
                if (ws.availableForWriteAll()) {
                    ws.binaryAll((uint8_t*)&dataToSend, sizeof(RxBatchData_t));
                    #if DebugCode
                        ws_sent_count++;
                    #endif
                } else {
                    #if DebugCode
                        ws_blocked++;
                    #endif
                }
            }
        }
    }
}

// ================================================================
// TASK 3: GỬI LỆNH XUỐNG STM32 (CORE 1) - FIXED!
// ================================================================
void TaskUARTTransmitter(void *pvParameters) {
    TxPacket_t txOutPacket; // Buffer hứng dữ liệu từ Queue
    #if DebugCode
        DBG_LN("📤 UART TX Task started (Core 1)");
    #endif
    
    while (1) {
        // NHẬN SỬA ĐỔI: Nhận trực tiếp vào struct txOutPacket
        if (xQueueReceive(txQueue, &txOutPacket, pdMS_TO_TICKS(500)) == pdTRUE) {
            
            size_t sendSize = 0;
            uint8_t cmd = txOutPacket.cmdFrame.cmd; // Lấy cmd để xác định độ dài
            
            if (cmd == CMD_SET_TRAJECTORY) {
                sendSize = sizeof(TxConfigData_t); // 13 bytes
            } 
            else if(cmd == CMD_STATUS){
                sendSize = sizeof(TxStatusData_t);    // 5 bytes
            }
            else {
                sendSize = sizeof(TxCMDData_t);    // 4 bytes
            }
            
            // THỰC HIỆN GỬI: Sử dụng con trỏ tới vùng nhớ raw của Union
            size_t sent = STM32Serial.write(txOutPacket.raw, sendSize);
            
            // Ép UART đẩy dữ liệu đi ngay (tránh nằm trong buffer của ESP32)
            STM32Serial.flush(); 
            
            #if DebugCode
                uart_tx_count++;
                DBG("📤 Sent to STM32: Cmd=%d, Size=%d, Sent=%d\n", cmd, sendSize, sent);
                // In mã Hex để cậu debug xem có đúng AA BB không
                DBG("   HEX: ");
                for(int i=0; i<sendSize; i++) DBG("%02X ", txOutPacket.raw[i]);
                DBG("\n");
            #endif
        }
    }
}

// ================================================================
// TASK 4: BẢO TRÌ WIFI (CORE 0)
// ================================================================
void TaskWiFiMaintenance(void *pvParameters) {
    #if DebugCode
        DBG_LN("🔧 WiFi Maintenance Task started (Core 0)");
    #endif
    
    TickType_t lastClean = 0;
    TickType_t lastStats = 0;
    TickType_t lastBatCheck = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();
        
        // 1. Đọc Pin mỗi 10 giây
        if ((now - lastBatCheck) >= pdMS_TO_TICKS(10000)) {
            lastBatCheck = now;
            
            // Đọc nhiều lần lấy trung bình cho ổn định (tránh nhiễu tức thời)
            long sumAdc = 0;
            for(int i=0; i<10; i++) sumAdc += analogRead(BATTERY_PIN);
            float adcVal = sumAdc / 10.0f;

            // CÔNG THỨC: Vpin = (ADC * 3.3 / 4095) * ((R1 + R2) / R2)
            // Với R1=47k, R2=10k -> Hệ số nhân là 5.7
            float voltage = (adcVal * 3.3f / 4095.0f) * 5.7f;

            // 2. PHÂN LOẠI NGƯỠNG THEO LOẠI PIN
            if (MY_BATTERY_TYPE == BAT_TYPE_3S) {
                // Ngưỡng cho 3S (Full 12.6V)
                if (voltage > 10.8f)      current_bat_stat = BATTERY_STATUS_100; // > 3.8V/cell
                else if (voltage > 8.0f) current_bat_stat = BATTERY_STATUS_60;  // > 3.7V/cell
                else                      current_bat_stat = BATTERY_STATUS_50;  // < 3.7V/cell
            } 
            else {
                // Ngưỡng cho 2S (Full 8.4V)
                if (voltage > 7.6f)       current_bat_stat = BATTERY_STATUS_100; // > 3.8V/cell
                else if (voltage > 7.4f)  current_bat_stat = BATTERY_STATUS_60;  // > 3.7V/cell
                else                      current_bat_stat = BATTERY_STATUS_50;  // < 3.7V/cell
            }

            SendStatusToSTM32();
            
            #if DebugCode
                DBG("🔋 [%s] Volt: %.2fV | ADC: %.0f -> Stat: %d\n", 
                    (MY_BATTERY_TYPE == BAT_TYPE_3S ? "3S" : "2S"), 
                    voltage, adcVal, current_bat_stat);
            #endif
        }

        // Cleanup mỗi 3 giây
        if ((now - lastClean) >= pdMS_TO_TICKS(3000)) {
            lastClean = now;
            
            if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ws.cleanupClients();
                xSemaphoreGive(wifiMutex);
            }
        }
        
        // Stats mỗi 5 giây
        if ((now - lastStats) >= pdMS_TO_TICKS(5000)) {
            lastStats = now;
            #if DebugCode
                DBG("📊 RX:%lu | TX:%lu | WS:%lu | Drop:%lu | Block:%lu | Q:%d/%d\n",
                    uart_rx_count, uart_tx_count, ws_sent_count, queue_drops, ws_blocked,
                    uxQueueMessagesWaiting(rxQueue), uxQueueSpacesAvailable(rxQueue) + uxQueueMessagesWaiting(rxQueue));
                uart_rx_count = 0;
                uart_tx_count = 0;
                ws_sent_count = 0;
                queue_drops = 0;
                ws_blocked = 0;
            #endif
            
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    setCpuFrequencyMhz(CPU_FREQ_MHZ);

    #if DebugCode
        DBG_LN("\n=== ESP32 Optimized (Fixed WiFi & UART) ===");
        Serial.begin(DEBUG_BAUDRATE);
        delay(1000);
    #endif
    // 4. Queues & Mutex
    rxQueue = xQueueCreate(10, sizeof(RxBatchData_t));
    txQueue = xQueueCreate(10, sizeof(TxPacket_t));
    wifiMutex = xSemaphoreCreateMutex();

    // 1. UART
    STM32Serial.setRxBufferSize(UART_RX_BUFFER_SIZE);
    STM32Serial.begin(STM32_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    #if DebugCode
        DBG_LN("✅ UART Ready (625000 bps)");
    #endif

    analogReadResolution(12); // Đảm bảo độ phân giải 12 bit
    WiFi.onEvent(WiFiEvent);  // Đăng ký nhận sự kiện WiFi

// 2. WiFi (SỬA LẠI: TỰ ĐỘNG CHỌN CHẾ ĐỘ THEO BIẾN MY_WIFI)
    if (MY_WIFI) {
        // --- CHẾ ĐỘ STATION (Kết nối vào mạng wifi nhà/công ty) ---
        WiFi.mode(WIFI_STA);
        WiFi.begin(MY_WIFI_SSID, MY_WIFI_PASSWORD); // Biến ssid, password cậu đã khai báo ở dòng 83, 84
        current_wifi_stat = WIFI_STATUS_CONNECTING;
        SendStatusToSTM32();
        #if DebugCode
            DBG("⏳ Đang kết nối tới: %s ", MY_WIFI_SSID);
        #endif

        // Chờ kết nối (thử 20 lần ~ 10 giây)
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500);
            #if DebugCode
                DBG(".");
            #endif
            retry++;
        }
        
        #if DebugCode
            if (WiFi.status() == WL_CONNECTED) {
                DBG_LN("\n✅ Đã kết nối WiFi!");
                DBG("   IP Address: ");
                DBG_LN(WiFi.localIP());
            } else {
                DBG_LN("\n❌ Kết nối thất bại! Vui lòng kiểm tra SSID/Pass.");
            }
        #endif

        if (WiFi.status() == WL_CONNECTED) {
            if (!MDNS.begin(MY_HOSTNAME)) { 
                #if DebugCode
                    DBG_LN("Error setting up MDNS responder!");
                #endif
            } else {
                #if DebugCode
                    DBG_LN("mDNS responder started. Address: myrobot.local");
                #endif
            }
        }

    } else {
        // --- CHẾ ĐỘ AP (Tự phát wifi như cũ) ---
        WiFi.mode(WIFI_AP);
        // Kênh 6, Ẩn SSID = 0, Max Client = 2
        WiFi.softAP(ROBOT_WIFI_SSID, ROBOT_WIFI_PASSWORD, 6, 0, 2);
        
        #if DebugCode
            DBG("\n✅ WiFi AP Created. Connect to: ");
            DBG_LN(ROBOT_WIFI_SSID);
            DBG("   IP Address: ");
            DBG_LN(WiFi.softAPIP());
        #endif
    }

    // Cấu hình chung cho cả 2 chế độ để tối ưu tốc độ (Quan trọng)
    WiFi.setSleep(false);
    WiFi.setSleep(WIFI_PS_NONE); // Tắt tiết kiệm pin để WiFi ổn định tối đa    // 3. WebSocket
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    
    // QUAN TRỌNG: Thêm Default Headers để tránh lỗi CORS khi App chạy trên nền Web
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    server.begin();
    #if DebugCode
        DBG_LN("✅ Web Server started");
    #endif

   
    
    // 5. Tasks
    xTaskCreatePinnedToCore(TaskUARTReceiver, "UartRx", 4096, NULL, 9, NULL, 1);
    xTaskCreatePinnedToCore(TaskUARTTransmitter, "UartTx", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(TaskWebSocketSender, "WsSender", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(TaskWiFiMaintenance, "WifiMaint", 4096, NULL, 3, NULL, 0);
    
    #if DebugCode
        DBG_LN("✅ All tasks created!");
    #endif
    
    // Init buffers (Giữ nguyên)
    txCMDBuffer.header[0] = PROTOCOL_CMD_HEADER_1;
    txCMDBuffer.header[1] = PROTOCOL_CMD_HEADER_2;
    txCMDBuffer.checksum  = PROTOCOL_CHECKSUM;

    txConfigBuffer.header[0] = PROTOCOL_CMD_HEADER_1;
    txConfigBuffer.header[1] = PROTOCOL_CMD_HEADER_2;
    txConfigBuffer.checksum  = PROTOCOL_CHECKSUM;
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void SendStatusToSTM32() {
    TxPacket_t packet;
    TxStatusData_t *pStatus = (TxStatusData_t*)packet.raw;

    pStatus->header[0]     = PROTOCOL_CMD_HEADER_1;
    pStatus->header[1]     = PROTOCOL_CMD_HEADER_2;
    pStatus->cmd           = CMD_STATUS;
    pStatus->wifi_status   = current_wifi_stat;
    pStatus->battery_status = current_bat_stat;
    pStatus->checksum      = CalculateChecksum(packet.raw, 5); // Checksum của 5 byte đầu

    xQueueSend(txQueue, &packet, 0);
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            current_wifi_stat = WIFI_STATUS_CONNECTED;
            SendStatusToSTM32(); // Gửi ngay lệnh sáng đứng LED Vàng
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            current_wifi_stat = WIFI_STATUS_CONNECTING;
            SendStatusToSTM32(); // Gửi ngay lệnh nháy LED Vàng
            break;
        default: break;
    }
}