// ============================================================
// ESP32-C3 固件: SPI 从机 + WiFi 云端双向通信 + CSV 分块导入
// 依据文档重写:
//   通讯接口(ESP与主控).md v3.1
//   CSV导入技术方案.md v1.1
//   双向通信方案.md
// ============================================================

#include <WiFi.h>
#include <vector>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------- 引脚 ----------------
#define GPIO_MOSI   3
#define GPIO_MISO   10
#define GPIO_SCLK   2
#define GPIO_CS     7
#define GPIO_IRQ    13          // 空闲高, 有新帧待读拉低

// ---------------- SPI 从机 ----------------
#define PACKET_LENGTH   128
#define QUEUE_SIZE      8
#define SPI_SLAVE_HOST  SPI2_HOST
#define DMA_CHANNEL     SPI_DMA_CH_AUTO
#define RING_BUF_SIZE   8

// 帧字段偏移
#define OFF_CMD         0
#define OFF_SUBCMD      1
#define OFF_LEN         2
#define OFF_PAYLOAD     3
#define OFF_SEQ         126
#define OFF_RESERVED    127
#define MAX_PAYLOAD     123

// ---------------- 诊断开关 ----------------
// SPI_CS_PROBE = 1: 不初始化 SPI 从机, GPIO7 作普通输入统计 CS 边沿 (隔离测试)
// SPI_CS_PROBE = 0: 正常 SPI 从机
#define SPI_CS_PROBE 0

// ---------------- 命令定义 ----------------
// 主命令
#define CMD_HEARTBEAT     0x00
#define CMD_DATA_UPDATE   0x10
#define CMD_SYS_CTRL      0x20
#define CMD_STATUS_QUERY  0x30
#define CMD_PROCESS_CTRL  0x40
#define CMD_LOG_DATA      0x50
#define CMD_HEATER_CTRL   0x60
#define CMD_CSV_UPLOAD    0x70
#define CMD_FILE_CTRL     0x71

// 0x10 数据更新
#define SUB_PROGRESS      0x01
#define SUB_STATUS        0x02
#define SUB_HEATER_ON     0x03
#define SUB_HEATER_TEMP   0x04

// 0x20 系统控制
#define CTRL_WIFI_ON      0x01
#define CTRL_WIFI_OFF     0x02
#define CTRL_WIFI_CONNECT 0x03

// 0x30 状态查询
#define QUERY_FAULT       0x01
#define QUERY_WIFI        0x02
#define QUERY_ALL         0x03

// 0x40 贴片流程控制
#define PROC_START        0x01
#define PROC_PAUSE        0x02
#define PROC_RESUME       0x03
#define PROC_STOP         0x04
#define PROC_ESTOP        0x05

// 0x50 日志
#define SUB_LOG_TEXT      0x01

// 0x60 加热台控制
#define HEAT_START        0x10
#define HEAT_STOP         0x11

// 0x70 CSV 上传
#define SUB_CSV_START     0x01
#define SUB_CSV_DATA      0x02
#define SUB_CSV_END       0x03
#define SUB_CSV_CANCEL    0x04

// 0x71 文件回执
#define SUB_FILE_NEXT         0x01
#define SUB_FILE_RESULT       0x02
#define SUB_FILE_CANCEL_ACK   0x03

// ESP 响应类型
#define RSP_IDLE          0x00
#define RSP_FAULT         0xF1
#define RSP_WIFI_STAT     0xF2
#define RSP_COMPOSITE     0xFF

// ---------------- 时序/重试参数 ----------------
#define SPI_TIMEOUT_MS       3000
#define WEB_CMD_DEDUP_MS     3000
#define WIFI_RETRY_BASE      1000
#define WIFI_RETRY_MAX       60000
#define CLOUD_REPORT_INTERVAL 500
#define FILE_REPORT_INTERVAL  5000
#define FILE_CHUNK_SIZE       121
#define FILE_MAX_SIZE         102400
#define FILE_RETRY_LIMIT      3
#define FILE_NET_FAIL_LIMIT   5
#define FILE_NET_RETRY_MS     30000
#define HTTP_TIMEOUT_MS       10000
#define MAX_LOG_BUF           20

// ---------------- 全局缓冲 ----------------
WORD_ALIGNED_ATTR uint8_t tx_pool[QUEUE_SIZE][PACKET_LENGTH] = {{0}};
WORD_ALIGNED_ATTR uint8_t rx_pool[QUEUE_SIZE][PACKET_LENGTH] = {{0}};
spi_slave_transaction_t trans_pool[QUEUE_SIZE];
// 逻辑发送缓冲; 发送前镜像到全部事务, 保证主控下一笔读到的都是最新帧
WORD_ALIGNED_ATTR uint8_t tx_buffer[PACKET_LENGTH] = {0};

static uint8_t ring_buffer[RING_BUF_SIZE][PACKET_LENGTH];
volatile uint8_t ring_head = 0;
volatile uint8_t ring_tail = 0;
volatile uint32_t ring_bits[RING_BUF_SIZE] = {0};

volatile uint32_t transaction_count = 0;
volatile uint32_t ring_drop_count = 0;
volatile uint32_t last_spi_activity = 0;
volatile bool irq_low = false;
volatile bool response_pending = false;   // tx_buffer 内还有未读取的响应
bool spi_connected = false;
volatile uint32_t tx_generation = 0;        // tx_pool 内容版本, 每次发布 +1
volatile uint32_t trans_start_generation = 0; // 当前事务开始时的内容版本
volatile bool trans_started_with_irq_low = false;       // 事务开始时 IRQ 是否处于低电平
volatile bool trans_started_with_response_pending = false; // 事务开始时是否有待读响应

// CS 诊断
volatile uint64_t cs_active_start_us = 0;
volatile uint32_t last_cs_active_us = 0;
volatile uint32_t cs_probe_edge_count = 0;
volatile uint64_t cs_probe_last_edge_us = 0;

// ---------------- 设备状态 ----------------
struct DeviceState {
    String   progress      = "--/--";
    String   status        = "Waiting";
    bool     heater_on     = false;
    float    heater_temp   = 0.0f;
    uint8_t  fault_code    = 0x00;
    unsigned long last_update = 0;
    bool     progress_valid = false;
    bool     status_valid   = false;
    bool     heater_on_valid = false;
    bool     heater_temp_valid = false;
};
DeviceState state;

// ---------------- 日志 ----------------
std::vector<String> logBuffer;

// ---------------- WiFi ----------------
char ssid[33]     = "3284659";
char password[64] = "09180918";
bool wifi_enabled = false;                 // v3.1: 上电不自动连接, 等 WiFi ON
unsigned long wifi_retry_interval = WIFI_RETRY_BASE;
unsigned long lastWifiCheck = 0;
unsigned long wifi_connect_start = 0;

// ---------------- 云端 ----------------
const char* CLOUD_HOST = "http://124.222.148.162:8080";
unsigned long lastCloudReport = 0;
unsigned long cloudReportInterval = CLOUD_REPORT_INTERVAL;

// ---------------- CSV 文件传输 ----------------
enum FilePhase {
    FILE_IDLE = 0,
    FILE_WAIT_START_RESULT = 1,
    FILE_SENDING_DATA = 2,
    FILE_WAIT_END_RESULT = 3,
    FILE_WAIT_NET = 4,
    FILE_SCANNING = 5
};

// CSV 编码模式
#define CSV_ENC_NONE      0
#define CSV_ENC_UTF8_BOM  1
#define CSV_ENC_UTF16LE   2
#define CSV_ENC_UTF16BE   3

// UTF-16 -> UTF-8 转换缓冲（121 字节 UTF-16LE 最多输出约 183 字节）
#define CSV_CONVERT_BUF   (FILE_CHUNK_SIZE * 2)
#define CSV_OUT_QUEUE     (FILE_CHUNK_SIZE * 3)

struct FileTransferState {
    bool     active = false;
    uint32_t version = 0;
    uint32_t offset = 0;
    uint32_t total = 0;
    uint32_t crc32 = 0;
    uint16_t next_frame = 0;
    uint16_t frames = 0;
    uint8_t  phase = FILE_IDLE;
    uint8_t  retry_count = 0;
    uint8_t  net_fail_count = 0;
    uint8_t  pending_frame[PACKET_LENGTH] = {0};
    unsigned long last_spi_attempt = 0;
    unsigned long next_retry_at = 0;

    // UTF-16LE/BE 转码会话状态
    uint8_t  csv_encoding = CSV_ENC_NONE;   // 检测到的编码
    bool     csv_scanning = false;          // 第一遍扫描进行中
    bool     csv_scan_done = false;         // 第一遍扫描完成
    bool     csv_pass2 = false;             // 第二遍转码发送已开始
    uint32_t csv_raw_total = 0;             // 服务器原始字节数（进度用）
    uint32_t csv_send_total = 0;            // 转码后发给主控的字节数
    uint16_t csv_send_frames = 0;           // 转码后帧数
    uint32_t csv_send_crc32 = 0;            // 转码后 CRC32
    uint32_t csv_scan_total = 0;            // 扫描累计转码字节
    uint32_t csv_scan_crc32 = 0;            // 扫描累计 CRC32
};
FileTransferState fileState;
String fileStatus = "idle";
int fileProgress = 0;

// 会话期间的缓存
bool fileQueryPending = false;
uint8_t fileQuerySub = 0;
uint8_t fileQuerySeq = 0;
bool fileCmdPending = false;
uint8_t pendingWebCmd = 0;
uint8_t pendingWebSub = 0;
String pendingWebPayload = "";
bool fileStartPending = false;
uint32_t pendingFileStartVersion = 0;
bool fileCancelPending = false;

// NVS
Preferences prefs;
uint32_t last_csv_version = 0;
bool last_csv_done = false;

// 前置声明
void parsePacketLocal(uint8_t *buf);
void processWebCommand(uint8_t cmd, uint8_t subcmd, const char* payload);
void handleStatusQuery(uint8_t subcmd, uint8_t seq);
void startFileImport(uint32_t version);
void cancelFileImport();
void abortFileTransfer();
void fetchAndSendNextDataFrame();
void fileTransferTick();
void fileFlushPending();
void fileSendPending();
void fileSetProgress();
void publishTxBuffer();
void spiRecycleTask(void *pvParameters);

// ============================================================
// 帧构建辅助
// ============================================================
String extractPayload(uint8_t *buf, uint8_t len) {
    String s = "";
    for (int i = 0; i < len && i < MAX_PAYLOAD; i++) {
        uint8_t b = buf[OFF_PAYLOAD + i];
        if (b == 0x00) break;
        s += (char)b;
    }
    return s;
}

void buildResponse(uint8_t rsp_type, uint8_t seq, uint8_t payload_len, const char* payload) {
    if (irq_low) return;   // 通道忙, 不覆盖待读帧
    memset(tx_buffer, 0x00, PACKET_LENGTH);
    tx_buffer[OFF_CMD]      = CMD_HEARTBEAT;
    tx_buffer[OFF_SUBCMD]   = rsp_type;
    tx_buffer[OFF_LEN]      = payload_len;
    tx_buffer[OFF_SEQ]      = seq;
    tx_buffer[OFF_RESERVED] = 0x00;
    if (payload && payload_len > 0) {
        memcpy(&tx_buffer[OFF_PAYLOAD], payload, min(payload_len, (uint8_t)MAX_PAYLOAD));
    }
    response_pending = true;
    publishTxBuffer();
}

void buildSPICommand(uint8_t cmd, uint8_t subcmd, const char* payload) {
    memset(tx_buffer, 0x00, PACKET_LENGTH);
    tx_buffer[OFF_CMD]    = cmd;
    tx_buffer[OFF_SUBCMD] = subcmd;
    tx_buffer[OFF_SEQ]    = 0;
    if (payload) {
        uint8_t len = strlen(payload);
        if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
        tx_buffer[OFF_LEN] = len;
        memcpy(&tx_buffer[OFF_PAYLOAD], payload, len);
    } else {
        tx_buffer[OFF_LEN] = 0;
    }
}

// 把逻辑 tx_buffer 镜像到事务池, 避免主控连续事务时读到旧帧
void publishTxBuffer() {
    for (int i = 0; i < QUEUE_SIZE; i++) {
        memcpy(tx_pool[i], tx_buffer, PACKET_LENGTH);
    }
    tx_generation++;
}

bool sendCommandToSTM32(uint8_t cmd, uint8_t subcmd, const char* payload) {
    if (irq_low || response_pending) {
        Serial.printf("[CMD] 命令 0x%02X/0x%02X 通道忙, 等待重试\n", cmd, subcmd);
        return false;
    }
    buildSPICommand(cmd, subcmd, payload);
    publishTxBuffer();
    digitalWrite(GPIO_IRQ, LOW);
    irq_low = true;
    Serial.printf("[CMD] 发送命令 0x%02X/0x%02X 到 STM32 (IRQ->LOW)\n", cmd, subcmd);
    return true;
}

// ============================================================
// 云端指令解析
// ============================================================
void parseCloudCommands(String response) {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, response);
    if (err) return;

    JsonArray arr = doc["commands"].as<JsonArray>();
    if (arr.isNull() || arr.size() == 0) return;

    for (JsonVariant item : arr) {
        uint8_t cmd    = item["cmd"]    | 0;
        uint8_t subcmd = item["subcmd"] | 0;
        const char* payload = item["payload"] | "";

        if (cmd == 0) continue;

        const char* display = payload;
        if (cmd == CMD_SYS_CTRL && subcmd == CTRL_WIFI_CONNECT) display = "<hidden>";
        Serial.printf("[CLOUD_CMD] 0x%02X/0x%02X payload=%s\n", cmd, subcmd, display);
        processWebCommand(cmd, subcmd, payload);
    }
}

// ============================================================
// 网页指令处理
// ============================================================
void processWebCommand(uint8_t cmd, uint8_t subcmd, const char* payload) {
    if (cmd == CMD_CSV_UPLOAD) {
        if (subcmd == SUB_CSV_START) {
            uint32_t v = 0;
            if (payload && sscanf(payload, "v=%u", &v) == 1) {
                if (fileState.active) {
                    cancelFileImport();
                    pendingFileStartVersion = v;
                    fileStartPending = true;
                    Serial.printf("[WEB] 会话忙, 先取消旧会话再导入 version=%u\n", (unsigned)v);
                } else if (irq_low || response_pending) {
                    pendingFileStartVersion = v;
                    fileStartPending = true;
                    Serial.printf("[WEB] 通道忙, 开始导入 version=%u 已排队\n", (unsigned)v);
                } else {
                    Serial.printf("[WEB] 开始 CSV 导入 version=%u\n", (unsigned)v);
                    startFileImport(v);
                }
            }
        } else if (subcmd == SUB_CSV_CANCEL) {
            Serial.println("[WEB] 取消 CSV 导入");
            cancelFileImport();
        }
        return;
    }

    if (cmd != CMD_PROCESS_CTRL && cmd != CMD_HEATER_CTRL) {
        Serial.printf("[WEB] 未知命令 0x%02X/0x%02X\n", cmd, subcmd);
        return;
    }

    static uint8_t lastWebCmd = 0;
    static uint8_t lastWebSub = 0;
    static unsigned long lastWebCmdMs = 0;
    if (cmd == lastWebCmd && subcmd == lastWebSub &&
        (unsigned long)(millis() - lastWebCmdMs) < WEB_CMD_DEDUP_MS) {
        Serial.printf("[WEB] 忽略重复命令 0x%02X/0x%02X (去重窗口 %lu ms)\n",
                      cmd, subcmd, (unsigned long)WEB_CMD_DEDUP_MS);
        return;
    }
    lastWebCmd = cmd;
    lastWebSub = subcmd;
    lastWebCmdMs = millis();

    if (fileState.active || irq_low || response_pending) {
        if (!fileCmdPending) {
            Serial.printf("[WEB] cmd cached: 0x%02X/0x%02X\n", cmd, subcmd);
        } else {
            Serial.printf("[WEB] pending overwritten: 0x%02X/0x%02X\n", cmd, subcmd);
        }
        fileCmdPending = true;
        pendingWebCmd = cmd;
        pendingWebSub = subcmd;
        pendingWebPayload = payload ? payload : "";
        return;
    }

    switch (cmd) {
        case CMD_PROCESS_CTRL:
            switch (subcmd) {
                case PROC_START:  Serial.println("[WEB] 开始贴片"); break;
                case PROC_PAUSE:  Serial.println("[WEB] 暂停"); break;
                case PROC_RESUME: Serial.println("[WEB] 继续"); break;
                case PROC_STOP:   Serial.println("[WEB] 结束任务"); break;
                case PROC_ESTOP:  Serial.println("[WEB] 急停!"); break;
                default: return;
            }
            sendCommandToSTM32(cmd, subcmd, payload);
            break;

        case CMD_HEATER_CTRL:
            switch (subcmd) {
                case HEAT_START:
                    Serial.println("[WEB] 开启加热"); break;
                case HEAT_STOP:
                    Serial.println("[WEB] 暂停加热"); break;
                default: return;
            }
            if (sendCommandToSTM32(cmd, subcmd, payload)) {
                if (subcmd == HEAT_START) {
                    state.heater_on = true;
                    state.heater_on_valid = true;
                } else {
                    state.heater_on = false;
                    state.heater_on_valid = true;
                }
            }
            break;
    }
}

// ============================================================
// 状态查询响应
// ============================================================
void handleStatusQuery(uint8_t subcmd, uint8_t seq) {
    switch (subcmd) {
        case QUERY_FAULT: {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X", state.fault_code);
            buildResponse(RSP_FAULT, seq, 2, buf);
            break;
        }
        case QUERY_WIFI: {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d,%d",
                     WiFi.status() == WL_CONNECTED ? 1 : 0, WiFi.RSSI());
            buildResponse(RSP_WIFI_STAT, seq, strlen(buf), buf);
            break;
        }
        case QUERY_ALL: {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s|%s|%d|%.1f|%02X",
                     state.progress.c_str(), state.status.c_str(),
                     state.heater_on ? 1 : 0, state.heater_temp, state.fault_code);
            buildResponse(RSP_COMPOSITE, seq, strlen(buf), buf);
            break;
        }
        default:
            break;
    }
}

// ============================================================
// WiFi 控制
// ============================================================
void applyWifiConnect(uint8_t *buf, uint8_t len) {
    int sep = -1;
    for (int i = 0; i < len; i++) {
        if (buf[OFF_PAYLOAD + i] == 0x00) { sep = i; break; }
    }
    if (sep <= 0) {
        Serial.println("[CTRL] WiFi CONNECT 格式错误: 缺少分隔符");
        return;
    }

    String newSsid = "";
    String newPass = "";
    for (int i = 0; i < sep; i++) newSsid += (char)buf[OFF_PAYLOAD + i];
    for (int i = sep + 1; i < len && buf[OFF_PAYLOAD + i] != 0x00; i++) newPass += (char)buf[OFF_PAYLOAD + i];

    if (newSsid.length() < 1 || newSsid.length() > 32) {
        Serial.printf("[CTRL] SSID 长度无效: %u (要求 1~32)\n", newSsid.length());
        return;
    }
    if (newPass.length() < 8 || newPass.length() > 63) {
        Serial.printf("[CTRL] 密码长度无效: %u (要求 8~63)\n", newPass.length());
        return;
    }

    newSsid.toCharArray(ssid, sizeof(ssid));
    newPass.toCharArray(password, sizeof(password));
    wifi_enabled = true;
    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(ssid, password);
    wifi_retry_interval = WIFI_RETRY_BASE;
    lastWifiCheck = millis();
    wifi_connect_start = millis();
    Serial.printf("[CTRL] WiFi CONNECT: SSID=\"%s\" (密码 %u 字符)\n", ssid, newPass.length());
}

// ============================================================
// SPI 数据包解析
// ============================================================
void parsePacketLocal(uint8_t *buf) {
    uint8_t cmd    = buf[OFF_CMD];
    uint8_t subcmd = buf[OFF_SUBCMD];
    uint8_t len    = buf[OFF_LEN];
    uint8_t seq    = buf[OFF_SEQ];
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;

    switch (cmd) {
        case CMD_DATA_UPDATE: {
            String payload = extractPayload(buf, len);
            switch (subcmd) {
                case SUB_PROGRESS:
                    state.progress = payload;
                    state.progress_valid = true;
                    state.last_update = millis();
                    break;
                case SUB_STATUS:
                    state.status = payload;
                    state.status_valid = true;
                    state.last_update = millis();
                    break;
                case SUB_HEATER_ON:
                    state.heater_on = (payload == "1");
                    state.heater_on_valid = true;
                    state.last_update = millis();
                    break;
                case SUB_HEATER_TEMP:
                    state.heater_temp = payload.toFloat();
                    state.heater_temp_valid = true;
                    state.last_update = millis();
                    break;
                default: break;
            }
            break;
        }

        case CMD_SYS_CTRL:
            switch (subcmd) {
                case CTRL_WIFI_ON:
                    Serial.println("[CTRL] WiFi ON");
                    if (!wifi_enabled) {
                        wifi_enabled = true;
                        WiFi.begin(ssid, password);
                        wifi_retry_interval = WIFI_RETRY_BASE;
                        lastWifiCheck = millis();
                        wifi_connect_start = millis();
                    }
                    break;
                case CTRL_WIFI_OFF:
                    Serial.println("[CTRL] WiFi OFF");
                    wifi_enabled = false;
                    WiFi.disconnect(true);
                    break;
                case CTRL_WIFI_CONNECT:
                    applyWifiConnect(buf, len);
                    break;
                default: break;
            }
            break;

        case CMD_STATUS_QUERY:
            if (fileState.active) {
                fileQueryPending = true;
                fileQuerySub = subcmd;
                fileQuerySeq = seq;
                Serial.println("[QUERY] 文件会话中, 状态查询已缓存");
            } else {
                Serial.printf("[RX] 状态查询 SUB=0x%02X\n", subcmd);
                handleStatusQuery(subcmd, seq);
            }
            break;

        case CMD_LOG_DATA: {
            String logText = extractPayload(buf, len);
            if (logText.length() > 0) {
                Serial.printf("[LOG] %s\n", logText.c_str());
                if (logBuffer.size() < MAX_LOG_BUF) logBuffer.push_back(logText);
                state.last_update = millis();
            }
            break;
        }

        case CMD_FILE_CTRL: {
            if (!fileState.active) break;
            switch (subcmd) {
                case SUB_FILE_NEXT: {
                    uint16_t expected = buf[OFF_PAYLOAD] | ((uint16_t)buf[OFF_PAYLOAD + 1] << 8);
                    if (expected == (uint16_t)(fileState.next_frame + 1)) {
                        fileState.retry_count = 0;
                        fileState.next_frame++;
                        fileSetProgress();
                        Serial.printf("[CSV] NEXT(%u) 确认\n", (unsigned)expected);
                        fetchAndSendNextDataFrame();
                    } else if (expected <= fileState.next_frame) {
                        // 主控可能重复发送同一条回执; 过期 NEXT 不再触发重发, 避免误发旧帧
                        Serial.printf("[CSV] 忽略重复/过期 NEXT(%u) (当前帧 %u)\n",
                            (unsigned)expected, (unsigned)fileState.next_frame);
                    } else {
                        Serial.printf("[CSV] NEXT(%u) 与期望 %u 不符, 中止\n",
                            (unsigned)expected, (unsigned)(fileState.next_frame + 1));
                        fileStatus = "failed";
                        abortFileTransfer();
                    }
                    break;
                }
                case SUB_FILE_RESULT: {
                    String res = extractPayload(buf, len);
                    if (fileState.phase == FILE_WAIT_START_RESULT) {
                        if (res == "ok") {
                            Serial.println("[CSV] START 已确认");
                            fileState.retry_count = 0;
                            fileState.phase = FILE_SENDING_DATA;
                            fetchAndSendNextDataFrame();
                        } else {
                            Serial.printf("[CSV] START 被拒绝: %s\n", res.c_str());
                            fileStatus = "failed";
                            abortFileTransfer();
                        }
                    } else if (fileState.phase == FILE_WAIT_END_RESULT) {
                        if (res == "ok") {
                            Serial.println("[CSV] END 校验通过, 导入成功");
                            fileStatus = "success";
                            fileProgress = 100;
                            last_csv_version = fileState.version;
                            last_csv_done = true;
                            prefs.putUInt("version", last_csv_version);
                            prefs.putBool("done", true);
                            abortFileTransfer();
                        } else {
                            Serial.printf("[CSV] END 校验失败: %s\n", res.c_str());
                            fileStatus = "failed";
                            abortFileTransfer();
                        }
                    }
                    break;
                }
                case SUB_FILE_CANCEL_ACK:
                    Serial.println("[CSV] CANCEL 已确认");
                    break;
                default: break;
            }
            break;
        }

        case CMD_HEARTBEAT:
            break;

        default:
            Serial.printf("[UNKNOWN] CMD=0x%02X SUB=0x%02X LEN=%u SEQ=0x%02X\n", cmd, subcmd, len, seq);
            break;
    }
}

// ============================================================
// CSV 文件传输状态机
// ============================================================
void fileSetProgress() {
    if (fileState.total > 0) {
        fileProgress = (int)((uint64_t)fileState.offset * 100 / fileState.total);
        if (fileProgress > 100) fileProgress = 100;
    } else {
        fileProgress = 0;
    }
}

void fileSendPending() {
    if (irq_low || response_pending) {
        Serial.println("[CSV] 发送帧失败: 通道忙");
        return;
    }
    memcpy(tx_buffer, fileState.pending_frame, PACKET_LENGTH);
    publishTxBuffer();
    digitalWrite(GPIO_IRQ, LOW);
    irq_low = true;
    fileState.last_spi_attempt = millis();
    Serial.printf("[CSV] 发送帧 SUB=0x%02X LEN=%u SEQ=0x%02X\n",
        fileState.pending_frame[OFF_SUBCMD], fileState.pending_frame[OFF_LEN],
        fileState.pending_frame[OFF_SEQ]);
}

void fileSendStart() {
    if (irq_low || response_pending) {
        Serial.println("[CSV] START 等待通道空闲");
        fileState.phase = FILE_WAIT_START_RESULT;
        return;
    }

    char payload[64];
    snprintf(payload, sizeof(payload), "len=%u,frames=%u,crc32=%08lx",
        (unsigned)fileState.csv_send_total, (unsigned)fileState.csv_send_frames,
        (unsigned long)fileState.csv_send_crc32);

    memset(tx_buffer, 0x00, PACKET_LENGTH);
    tx_buffer[OFF_CMD]    = CMD_CSV_UPLOAD;
    tx_buffer[OFF_SUBCMD] = SUB_CSV_START;
    tx_buffer[OFF_LEN]    = strlen(payload);
    memcpy(&tx_buffer[OFF_PAYLOAD], payload, tx_buffer[OFF_LEN]);
    tx_buffer[OFF_SEQ]    = 0;

    publishTxBuffer();
    digitalWrite(GPIO_IRQ, LOW);
    irq_low = true;
    fileState.last_spi_attempt = millis();
    Serial.printf("[CSV] START len=%u frames=%u crc32=%08lx\n",
        (unsigned)fileState.csv_send_total, (unsigned)fileState.csv_send_frames,
        (unsigned long)fileState.csv_send_crc32);
}

void fileBuildDataFrame(uint16_t frame_no, const uint8_t* data, uint8_t data_len) {
    memset(fileState.pending_frame, 0x00, PACKET_LENGTH);
    fileState.pending_frame[OFF_CMD]    = CMD_CSV_UPLOAD;
    fileState.pending_frame[OFF_SUBCMD] = SUB_CSV_DATA;
    fileState.pending_frame[OFF_LEN]    = 2 + data_len;
    fileState.pending_frame[OFF_PAYLOAD] = frame_no & 0xFF;
    fileState.pending_frame[OFF_PAYLOAD + 1] = (frame_no >> 8) & 0xFF;
    memcpy(&fileState.pending_frame[OFF_PAYLOAD + 2], data, data_len);
    fileState.pending_frame[OFF_SEQ] = frame_no & 0xFF;
}

void fileBuildEndFrame() {
    char payload[32];
    snprintf(payload, sizeof(payload), "crc32=%08lx", (unsigned long)fileState.csv_send_crc32);
    memset(fileState.pending_frame, 0x00, PACKET_LENGTH);
    fileState.pending_frame[OFF_CMD]    = CMD_CSV_UPLOAD;
    fileState.pending_frame[OFF_SUBCMD] = SUB_CSV_END;
    fileState.pending_frame[OFF_LEN]    = strlen(payload);
    memcpy(&fileState.pending_frame[OFF_PAYLOAD], payload, fileState.pending_frame[OFF_LEN]);
    fileState.pending_frame[OFF_SEQ] = fileState.next_frame & 0xFF;
}

void fileSendCancel() {
    if (irq_low || response_pending) {
        Serial.println("[CSV] CANCEL 等待通道空闲");
        return;
    }
    memset(tx_buffer, 0x00, PACKET_LENGTH);
    tx_buffer[OFF_CMD]    = CMD_CSV_UPLOAD;
    tx_buffer[OFF_SUBCMD] = SUB_CSV_CANCEL;
    tx_buffer[OFF_LEN]    = 0;
    publishTxBuffer();
    digitalWrite(GPIO_IRQ, LOW);
    irq_low = true;
    fileState.last_spi_attempt = millis();
    Serial.println("[CSV] CANCEL 已发送");
}

// 从服务器拉取一块原始字节, 返回 HTTP 状态码 (网络失败返回负值)
int fetchFileBlock(uint32_t version, uint32_t offset, uint8_t* out_buf, uint8_t& out_len,
                   uint32_t& next_offset, uint32_t& total, uint32_t& crc32, bool& done) {
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.setConnectTimeout(HTTP_TIMEOUT_MS);
    String url = String(CLOUD_HOST) + "/file?version=" + String(version) + "&offset=" + String(offset);
    http.begin(url);

    // ESP32 HTTPClient 只返回 collectHeaders 注册过的响应头
    const char* headerKeys[] = {"X-Size", "X-Next-Offset", "X-Crc32", "X-Done"};
    http.collectHeaders(headerKeys, 4);

    // HTTP 是阻塞调用, 喂一次狗避免 loop 内多段网络超时累计触发看门狗
    esp_task_wdt_reset();
    int code = http.GET();
    if (code == 200) {
        total = strtoul(http.header("X-Size").c_str(), NULL, 10);
        next_offset = strtoul(http.header("X-Next-Offset").c_str(), NULL, 10);
        crc32 = strtoul(http.header("X-Crc32").c_str(), NULL, 16);
        done = (http.header("X-Done") == "1");

        out_len = 0;
        Stream& stream = http.getStream();
        out_len = (uint8_t)stream.readBytes(out_buf, FILE_CHUNK_SIZE);  // 原始字节, 禁止 getString
        Serial.printf("[CSV] 拉取块 offset=%u len=%u next=%u total=%u crc32=%08lx done=%d\n",
            (unsigned)offset, (unsigned)out_len, (unsigned)next_offset,
            (unsigned)total, (unsigned long)crc32, done ? 1 : 0);
        fileState.net_fail_count = 0;
    }
    http.end();
    esp_task_wdt_reset();
    return code;
}

void fileHandleNetError() {
    fileState.net_fail_count++;
    fileState.phase = FILE_WAIT_NET;
    if (fileState.net_fail_count >= FILE_NET_FAIL_LIMIT) {
        fileStatus = "network_error";
        fileState.next_retry_at = millis() + FILE_NET_RETRY_MS;
        Serial.println("[CSV] 网络连续失败, 进入 30s 自动重试");
    } else {
        fileState.next_retry_at = millis() + 1000;
        Serial.printf("[CSV] 网络拉块失败 (%u/%u)\n",
            (unsigned)fileState.net_fail_count, (unsigned)FILE_NET_FAIL_LIMIT);
    }
}

// ============================================================
// UTF-16LE/BE -> UTF-8 转码 + CRC32 重算
// ============================================================
static uint32_t csvCrcTable[256];
static bool csvCrcReady = false;

static void csvCrcInit() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++) {
            c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
        }
        csvCrcTable[i] = c;
    }
    csvCrcReady = true;
}

// 标准 zlib CRC-32，与 relay_server.py / 主控端校验一致
static uint32_t csvCrc32Update(uint32_t crc, const uint8_t *data, uint32_t len) {
    if (!csvCrcReady) csvCrcInit();
    crc ^= 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ csvCrcTable[(crc ^ data[i]) & 0xFFUL];
    }
    return crc ^ 0xFFFFFFFFUL;
}

struct CsvConvertState {
    bool     first = true;
    bool     pending = false;
    uint8_t  pending_lo = 0;
    uint8_t  pending_hi = 0;
    uint32_t high_surrogate = 0;
    uint8_t  encoding = CSV_ENC_NONE;
};
static CsvConvertState csvConvert;

// 转码后字节队列，容量足够容纳一次转换输出 + 未满一帧的残留
static uint8_t csvOutQueue[CSV_OUT_QUEUE];
static uint16_t csvOutHead = 0;
static uint16_t csvOutLen = 0;

static void csvWriteUtf8(uint8_t *out, uint16_t &out_len, uint16_t cap, uint32_t cp) {
    if (cp <= 0x7F) {
        if (out_len < cap) out[out_len++] = (uint8_t)cp;
    } else if (cp <= 0x7FF) {
        if (out_len + 1 < cap) {
            out[out_len++] = (uint8_t)(0xC0 | (cp >> 6));
            out[out_len++] = (uint8_t)(0x80 | (cp & 0x3F));
        }
    } else if (cp <= 0xFFFF) {
        if (out_len + 2 < cap) {
            out[out_len++] = (uint8_t)(0xE0 | (cp >> 12));
            out[out_len++] = (uint8_t)(0x80 | ((cp >> 6) & 0x3F));
            out[out_len++] = (uint8_t)(0x80 | (cp & 0x3F));
        }
    } else {
        if (out_len + 3 < cap) {
            out[out_len++] = (uint8_t)(0xF0 | (cp >> 18));
            out[out_len++] = (uint8_t)(0x80 | ((cp >> 12) & 0x3F));
            out[out_len++] = (uint8_t)(0x80 | ((cp >> 6) & 0x3F));
            out[out_len++] = (uint8_t)(0x80 | (cp & 0x3F));
        }
    }
}

static uint8_t csvDetectEncoding(const uint8_t *data, uint16_t len) {
    if (len >= 3 && data[0] == 0xEF && data[1] == 0xBB && data[2] == 0xBF) {
        return CSV_ENC_UTF8_BOM;
    }
    if (len >= 2 && data[0] == 0xFF && data[1] == 0xFE) {
        return CSV_ENC_UTF16LE;
    }
    if (len >= 2 && data[0] == 0xFE && data[1] == 0xFF) {
        return CSV_ENC_UTF16BE;
    }
    return CSV_ENC_NONE;
}

static void csvConvertReset() {
    csvConvert.first = true;
    csvConvert.pending = false;
    csvConvert.pending_lo = 0;
    csvConvert.pending_hi = 0;
    csvConvert.high_surrogate = 0;
    csvConvert.encoding = CSV_ENC_NONE;
}

static uint16_t csvConvertBlock(const uint8_t *in, uint16_t in_len,
                                uint8_t *out, uint16_t out_cap) {
    uint16_t out_len = 0;

    if (csvConvert.first) {
        csvConvert.first = false;
        csvConvert.encoding = csvDetectEncoding(in, in_len);
        if (csvConvert.encoding == CSV_ENC_UTF8_BOM) {
            in += 3;
            in_len -= 3;
        } else if (csvConvert.encoding == CSV_ENC_UTF16LE ||
                   csvConvert.encoding == CSV_ENC_UTF16BE) {
            in += 2;
            in_len -= 2;
        } else {
            csvConvert.encoding = CSV_ENC_NONE;
        }
    }

    if (csvConvert.encoding == CSV_ENC_NONE ||
        csvConvert.encoding == CSV_ENC_UTF8_BOM) {
        if (in_len > 0) {
            uint16_t n = in_len;
            if (n > out_cap) n = out_cap;
            memcpy(out, in, n);
            out_len = n;
        }
        return out_len;
    }

    uint16_t i = 0;
    while (i < in_len) {
        uint32_t code;
        if (csvConvert.pending) {
            if (csvConvert.encoding == CSV_ENC_UTF16LE) {
                code = csvConvert.pending_lo | ((uint32_t)in[i] << 8);
            } else {
                code = ((uint32_t)csvConvert.pending_hi << 8) | in[i];
            }
            csvConvert.pending = false;
            i++;
        } else {
            if (in_len - i == 1) {
                if (csvConvert.encoding == CSV_ENC_UTF16LE) {
                    csvConvert.pending_lo = in[i];
                } else {
                    csvConvert.pending_hi = in[i];
                }
                csvConvert.pending = true;
                i++;
                break;
            }
            if (csvConvert.encoding == CSV_ENC_UTF16LE) {
                code = in[i] | ((uint32_t)in[i + 1] << 8);
            } else {
                code = ((uint32_t)in[i] << 8) | in[i + 1];
            }
            i += 2;
        }

        if (code >= 0xD800 && code <= 0xDBFF) {
            if (csvConvert.high_surrogate != 0) {
                csvWriteUtf8(out, out_len, out_cap, 0xFFFD);
            }
            csvConvert.high_surrogate = code;
            continue;
        }
        if (code >= 0xDC00 && code <= 0xDFFF) {
            if (csvConvert.high_surrogate != 0) {
                uint32_t cp = 0x10000UL +
                    ((csvConvert.high_surrogate - 0xD800UL) << 10) +
                    (code - 0xDC00UL);
                csvConvert.high_surrogate = 0;
                csvWriteUtf8(out, out_len, out_cap, cp);
            } else {
                csvWriteUtf8(out, out_len, out_cap, 0xFFFD);
            }
            continue;
        }
        if (csvConvert.high_surrogate != 0) {
            csvWriteUtf8(out, out_len, out_cap, 0xFFFD);
            csvConvert.high_surrogate = 0;
        }
        csvWriteUtf8(out, out_len, out_cap, code);
    }
    return out_len;
}

static uint16_t csvConvertFlush(uint8_t *out, uint16_t out_cap) {
    uint16_t out_len = 0;
    if (csvConvert.high_surrogate != 0) {
        csvWriteUtf8(out, out_len, out_cap, 0xFFFD);
        csvConvert.high_surrogate = 0;
    }
    if (csvConvert.pending) {
        csvWriteUtf8(out, out_len, out_cap, 0xFFFD);
        csvConvert.pending = false;
    }
    return out_len;
}

static void csvQueueAppend(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len && csvOutLen < CSV_OUT_QUEUE; i++) {
        uint16_t pos = (csvOutHead + csvOutLen) % CSV_OUT_QUEUE;
        csvOutQueue[pos] = data[i];
        csvOutLen++;
    }
}

static void csvQueueTake(uint8_t *out, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        out[i] = csvOutQueue[csvOutHead];
        csvOutHead = (csvOutHead + 1) % CSV_OUT_QUEUE;
    }
    csvOutLen -= len;
}

static void csvFinishScan() {
    if (fileState.csv_scan_total == 0 || fileState.csv_scan_total > FILE_MAX_SIZE) {
        Serial.println("[CSV] 转码后文件为空或超过 100KB, 中止导入");
        fileStatus = "failed";
        abortFileTransfer();
        return;
    }
    fileState.csv_send_total = fileState.csv_scan_total;
    fileState.csv_send_frames =
        (uint16_t)((fileState.csv_scan_total + FILE_CHUNK_SIZE - 1) / FILE_CHUNK_SIZE);
    fileState.csv_send_crc32 = fileState.csv_scan_crc32;
    fileState.frames = fileState.csv_send_frames;
    fileState.csv_scan_done = true;
    fileState.csv_scanning = false;
    fileState.phase = FILE_WAIT_START_RESULT;
    Serial.printf("[CSV] 扫描完成: 转码后 len=%u frames=%u crc32=%08lx\n",
        (unsigned)fileState.csv_send_total, (unsigned)fileState.csv_send_frames,
        (unsigned long)fileState.csv_send_crc32);
    fileSendStart();
}

static void csvScanNextBlock() {
    uint8_t block[FILE_CHUNK_SIZE];
    uint8_t block_len = 0;
    uint32_t next_offset = 0, total = 0, crc32 = 0;
    bool done = false;
    int code = fetchFileBlock(fileState.version, fileState.offset, block, block_len,
                              next_offset, total, crc32, done);

    if (code != 200) {
        if (code == 404) {
            fileStatus = "expired";
            abortFileTransfer();
        } else if (code == 409) {
            fileStatus = "overwritten";
            abortFileTransfer();
        } else {
            fileHandleNetError();
        }
        return;
    }

    fileState.offset = next_offset;
    fileState.total = total;
    fileState.crc32 = crc32;
    fileState.csv_raw_total = total;

    uint8_t conv[CSV_CONVERT_BUF];
    uint16_t conv_len = csvConvertBlock(block, block_len, conv, sizeof(conv));
    fileState.csv_scan_total += conv_len;
    fileState.csv_scan_crc32 =
        csvCrc32Update(fileState.csv_scan_crc32, conv, conv_len);

    if (fileState.offset >= fileState.total) {
        uint8_t flush_buf[8];
        uint16_t flush_len = csvConvertFlush(flush_buf, sizeof(flush_buf));
        if (flush_len > 0) {
            fileState.csv_scan_total += flush_len;
            fileState.csv_scan_crc32 =
                csvCrc32Update(fileState.csv_scan_crc32, flush_buf, flush_len);
        }
        csvFinishScan();
    }
}

static void csvSendNextConvertedFrame() {
    uint8_t conv[CSV_CONVERT_BUF];
    bool eof = false;

    while (true) {
        if (csvOutLen >= FILE_CHUNK_SIZE) {
            uint8_t frame_data[FILE_CHUNK_SIZE];
            csvQueueTake(frame_data, FILE_CHUNK_SIZE);
            fileBuildDataFrame(fileState.next_frame, frame_data, FILE_CHUNK_SIZE);
            fileState.phase = FILE_SENDING_DATA;
            fileSendPending();
            return;
        }

        if (eof) {
            if (csvOutLen > 0) {
                uint8_t frame_data[FILE_CHUNK_SIZE];
                csvQueueTake(frame_data, csvOutLen);
                fileBuildDataFrame(fileState.next_frame, frame_data, csvOutLen);
                fileState.phase = FILE_SENDING_DATA;
                fileSendPending();
                return;
            }
            fileBuildEndFrame();
            fileState.phase = FILE_WAIT_END_RESULT;
            fileStatus = "verifying";
            fileSendPending();
            return;
        }

        uint8_t block[FILE_CHUNK_SIZE];
        uint8_t block_len = 0;
        uint32_t next_offset = 0, total = 0, crc32 = 0;
        bool done = false;
        int code = fetchFileBlock(fileState.version, fileState.offset, block, block_len,
                                  next_offset, total, crc32, done);
        if (code != 200) {
            if (code == 404) {
                fileStatus = "expired";
                abortFileTransfer();
            } else if (code == 409) {
                fileStatus = "overwritten";
                abortFileTransfer();
            } else {
                fileHandleNetError();
            }
            return;
        }

        fileState.offset = next_offset;
        fileState.total = total;
        fileState.crc32 = crc32;
        fileState.csv_raw_total = total;
        if (fileState.offset >= fileState.total) eof = true;

        uint16_t conv_len = csvConvertBlock(block, block_len, conv, sizeof(conv));
        csvQueueAppend(conv, conv_len);
        if (eof) {
            uint8_t flush_buf[8];
            uint16_t flush_len = csvConvertFlush(flush_buf, sizeof(flush_buf));
            if (flush_len > 0) csvQueueAppend(flush_buf, flush_len);
        }
    }
}

void fetchAndSendNextDataFrame() {
    if (fileState.csv_encoding == CSV_ENC_NONE) {
        if (fileState.next_frame >= fileState.frames) {
            fileBuildEndFrame();
            fileState.phase = FILE_WAIT_END_RESULT;
            fileStatus = "verifying";
            fileSendPending();
            return;
        }

        uint8_t block[FILE_CHUNK_SIZE];
        uint8_t block_len = 0;
        uint32_t next_offset = 0, total = 0, crc32 = 0;
        bool done = false;
        int code = fetchFileBlock(fileState.version, fileState.offset, block, block_len,
                                  next_offset, total, crc32, done);

        if (code == 200) {
            fileState.offset = next_offset;
            fileState.total = total;
            fileState.crc32 = crc32;
            fileBuildDataFrame(fileState.next_frame, block, block_len);
            fileState.phase = FILE_SENDING_DATA;
            fileSendPending();
        } else if (code == 404) {
            fileStatus = "expired";
            abortFileTransfer();
        } else if (code == 409) {
            fileStatus = "overwritten";
            abortFileTransfer();
        } else {
            fileHandleNetError();
        }
        return;
    }

    if (fileState.csv_scanning && !fileState.csv_scan_done) {
        csvScanNextBlock();
        return;
    }

    if (!fileState.csv_pass2) {
        fileState.csv_pass2 = true;
        fileState.offset = 0;
        csvOutHead = 0;
        csvOutLen = 0;
        csvConvertReset();
    }
    csvSendNextConvertedFrame();
}

void abortFileTransfer() {
    fileState.active = false;
    fileState.phase = FILE_IDLE;
    fileState.retry_count = 0;
    fileState.net_fail_count = 0;
    cloudReportInterval = CLOUD_REPORT_INTERVAL;
    Serial.println("[CSV] 文件会话结束");
}

void startFileImport(uint32_t version) {
    if (last_csv_done && last_csv_version == version) {
        Serial.printf("[CSV] 版本 %u 已完成, 跳过重复导入\n", (unsigned)version);
        fileStatus = "success";
        fileProgress = 100;
        return;
    }

    if (fileState.active) {
        Serial.println("[CSV] 旧会话进行中, 先取消再开始新会话");
        cancelFileImport();
        pendingFileStartVersion = version;
        fileStartPending = true;
        return;
    }

    fileState.active = true;
    fileState.version = version;
    fileState.offset = 0;
    fileState.total = 0;
    fileState.crc32 = 0;
    fileState.next_frame = 0;
    fileState.frames = 0;
    fileState.retry_count = 0;
    fileState.net_fail_count = 0;
    fileState.phase = FILE_WAIT_START_RESULT;
    fileState.last_spi_attempt = 0;
    fileState.csv_encoding = CSV_ENC_NONE;
    fileState.csv_scanning = false;
    fileState.csv_scan_done = false;
    fileState.csv_pass2 = false;
    fileState.csv_raw_total = 0;
    fileState.csv_send_total = 0;
    fileState.csv_send_frames = 0;
    fileState.csv_send_crc32 = 0;
    fileState.csv_scan_total = 0;
    fileState.csv_scan_crc32 = 0;
    cloudReportInterval = FILE_REPORT_INTERVAL;
    fileStatus = "transferring";
    fileProgress = 0;

    uint8_t block[FILE_CHUNK_SIZE];
    uint8_t block_len = 0;
    uint32_t next_offset = 0, total = 0, crc32 = 0;
    bool done = false;
    int code = fetchFileBlock(version, 0, block, block_len, next_offset, total, crc32, done);

    if (code == 200) {
        fileState.total = total;
        fileState.crc32 = crc32;

        uint8_t enc = csvDetectEncoding(block, block_len);
        if (enc == CSV_ENC_NONE) {
            fileState.csv_send_total = total;
            fileState.csv_send_frames =
                (uint16_t)((total + FILE_CHUNK_SIZE - 1) / FILE_CHUNK_SIZE);
            fileState.csv_send_crc32 = crc32;
            fileState.frames = fileState.csv_send_frames;
            if (fileState.total == 0 || fileState.frames == 0) {
                Serial.println("[CSV] 服务器返回空文件或响应头解析失败, 中止导入");
                fileStatus = "failed";
                abortFileTransfer();
                return;
            }
            fileSendStart();
        } else {
            fileState.csv_encoding = enc;
            fileState.csv_scanning = true;
            fileState.csv_scan_done = false;
            fileState.csv_pass2 = false;
            fileState.offset = next_offset;
            fileState.csv_raw_total = total;
            fileState.phase = FILE_SCANNING;
            csvConvertReset();
            csvOutHead = 0;
            csvOutLen = 0;

            uint8_t conv[CSV_CONVERT_BUF];
            uint16_t conv_len = csvConvertBlock(block, block_len, conv, sizeof(conv));
            fileState.csv_scan_total += conv_len;
            fileState.csv_scan_crc32 =
                csvCrc32Update(fileState.csv_scan_crc32, conv, conv_len);
            Serial.printf("[CSV] 检测到 BOM/UTF-16, 进入转码扫描 (raw=%u)\n",
                (unsigned)total);

            if (fileState.offset >= fileState.total) {
                uint8_t flush_buf[8];
                uint16_t flush_len = csvConvertFlush(flush_buf, sizeof(flush_buf));
                if (flush_len > 0) {
                    fileState.csv_scan_total += flush_len;
                    fileState.csv_scan_crc32 =
                        csvCrc32Update(fileState.csv_scan_crc32, flush_buf, flush_len);
                }
                csvFinishScan();
            }
        }
    } else if (code == 404) {
        fileStatus = "expired";
        abortFileTransfer();
    } else if (code == 409) {
        fileStatus = "overwritten";
        abortFileTransfer();
    } else {
        fileHandleNetError();
    }
}

void cancelFileImport() {
    if (!fileState.active) {
        fileStatus = "cancelled";
        Serial.println("[CSV] 当前无导入会话");
        return;
    }
    fileCancelPending = true;
    fileStatus = "cancelled";
    Serial.println("[CSV] 取消命令已排队");
}

void fileTransferTick() {
    if (fileCancelPending && !irq_low && !response_pending) {
        fileCancelPending = false;
        fileSendCancel();
        abortFileTransfer();
    }

    if (fileStartPending && !fileState.active && !irq_low && !response_pending) {
        fileStartPending = false;
        startFileImport(pendingFileStartVersion);
    }

    if (!fileState.active) return;

    // 转码预扫描阶段：每次 tick 拉一块并转码，完成后自动发送 START
    if (fileState.csv_scanning && !fileState.csv_scan_done &&
        fileState.phase == FILE_SCANNING) {
        fetchAndSendNextDataFrame();
        return;
    }

    // START 尚未成功发出 (例如之前响应未消费), 通道空闲后重发
    if (fileState.phase == FILE_WAIT_START_RESULT && !irq_low && !response_pending &&
        fileState.last_spi_attempt == 0) {
        fileSendStart();
        return;
    }

    if (fileState.phase == FILE_WAIT_START_RESULT ||
        fileState.phase == FILE_SENDING_DATA ||
        fileState.phase == FILE_WAIT_END_RESULT) {
        if (irq_low && millis() - fileState.last_spi_attempt >= SPI_TIMEOUT_MS) {
            if (fileCancelPending) {
                fileCancelPending = false;
                Serial.println("[CSV] IRQ 等待超时, 取消导入");
            } else {
                Serial.println("[CSV] IRQ 等待超时, 中止导入");
                fileStatus = "failed";
            }
            abortFileTransfer();
            return;
        }
    }

    // 帧已被主控读取但长时间未收到回执 (NEXT/RESULT), 重发当前帧, 避免死等
    if (!irq_low && !response_pending && fileState.last_spi_attempt != 0 &&
        (fileState.phase == FILE_WAIT_START_RESULT ||
         fileState.phase == FILE_SENDING_DATA ||
         fileState.phase == FILE_WAIT_END_RESULT)) {
        if (millis() - fileState.last_spi_attempt >= SPI_TIMEOUT_MS) {
            fileState.retry_count++;
            if (fileState.retry_count > FILE_RETRY_LIMIT) {
                Serial.println("[CSV] 等待主控回执超时, 中止导入");
                fileStatus = "failed";
                abortFileTransfer();
                return;
            }
            if (fileState.phase == FILE_WAIT_START_RESULT) {
                Serial.printf("[CSV] 等待 START RESULT 超时, 重发 START (%u/%u)\n",
                    (unsigned)fileState.retry_count, (unsigned)FILE_RETRY_LIMIT);
                fileSendStart();
            } else if (fileState.phase == FILE_SENDING_DATA) {
                Serial.printf("[CSV] 等待 NEXT 超时, 重发当前帧 (%u/%u)\n",
                    (unsigned)fileState.retry_count, (unsigned)FILE_RETRY_LIMIT);
                fileSendPending();
            } else {
                Serial.printf("[CSV] 等待 END RESULT 超时, 重发 END (%u/%u)\n",
                    (unsigned)fileState.retry_count, (unsigned)FILE_RETRY_LIMIT);
                fileSendPending();
            }
            return;
        }
    }

    if (fileState.phase == FILE_WAIT_NET && millis() >= fileState.next_retry_at) {
        if (fileState.total == 0 && fileState.next_frame == 0) {
            abortFileTransfer();
            startFileImport(fileState.version);
        } else {
            fetchAndSendNextDataFrame();
        }
    }
}

void fileFlushPending() {
    if (!fileState.active && fileQueryPending && !irq_low && !response_pending) {
        fileQueryPending = false;
        handleStatusQuery(fileQuerySub, fileQuerySeq);
        return;
    }
    if (!fileState.active && fileCmdPending && !irq_low && !response_pending) {
        uint8_t c = pendingWebCmd;
        uint8_t s = pendingWebSub;
        String p = pendingWebPayload;
        fileCmdPending = false;
        processWebCommand(c, s, p.c_str());
    }
}

// ============================================================
// JSON 上报
// ============================================================
String buildDataJson() {
    char faultHex[4];
    snprintf(faultHex, sizeof(faultHex), "%02X", state.fault_code);

    String json = "{";
    json += "\"t\":" + String(millis()) + ",";
    json += "\"progress\":\"" + state.progress + "\",";
    json += "\"status\":\"" + state.status + "\",";
    json += "\"heater_on\":" + String(state.heater_on ? 1 : 0) + ",";
    json += "\"heater_state\":\"" + String(state.heater_on ? "HEATING" : "IDLE") + "\",";
    if (state.heater_temp_valid) {
        json += "\"heater_temp\":" + String(state.heater_temp, 1) + ",";
    }
    json += "\"fault\":\"" + String(faultHex) + "\",";
    json += "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
    json += "\"heartbeat\":1,";
    json += "\"spi\":" + String(spi_connected ? 1 : 0) + ",";
    json += "\"file_status\":\"" + fileStatus + "\",";
    json += "\"file_progress\":" + String(fileProgress) + ",";

    json += "\"logs\":[";
    for (size_t i = 0; i < logBuffer.size(); i++) {
        if (i > 0) json += ",";
        String s = logBuffer[i];
        s.replace("\\", "\\\\");
        s.replace("\"", "\\\"");
        json += "\"" + s + "\"";
    }
    json += "]";

    json += "}";
    return json;
}

// ============================================================
// SPI 回调
// ============================================================
#if SPI_CS_PROBE
void IRAM_ATTR cs_probe_isr(void *arg) {
    (void)arg;
    cs_probe_edge_count++;
    cs_probe_last_edge_us = esp_timer_get_time();
}
#endif

void IRAM_ATTR spi_post_setup_callback(spi_slave_transaction_t *trans) {
    (void)trans;
    trans_start_generation = tx_generation;
    trans_started_with_irq_low = irq_low;
    trans_started_with_response_pending = response_pending;
    cs_active_start_us = esp_timer_get_time();
}

void IRAM_ATTR spi_post_trans_callback(spi_slave_transaction_t *trans) {
    uint64_t now_us = esp_timer_get_time();
    if (cs_active_start_us != 0) {
        last_cs_active_us = (uint32_t)(now_us - cs_active_start_us);
        cs_active_start_us = 0;
    }
    transaction_count++;

    // Only clear when this transaction actually read a published frame or
    // response: it started with IRQ low (ESP-initiated frame) or with a
    // pending response, and no newer publish happened during the transfer.
    // A transaction that starts between publishTxBuffer() and irq_low=true
    // must not clear, otherwise the host would read an empty frame.
    if (trans_start_generation == tx_generation &&
        (trans_started_with_irq_low || trans_started_with_response_pending)) {
        response_pending = false;
        if (irq_low) {
            digitalWrite(GPIO_IRQ, HIGH);
            irq_low = false;
        }
        for (int i = 0; i < QUEUE_SIZE; i++) {
            memset(tx_pool[i], 0x00, PACKET_LENGTH);
        }
    }

    if (trans && trans->rx_buffer) {
        uint8_t next_head = (ring_head + 1) % RING_BUF_SIZE;
        if (next_head == ring_tail) {
            ring_drop_count++;   // 环形缓冲满, 丢弃本次数据
            return;
        }
        ring_bits[ring_head] = trans->trans_len;
        memcpy(ring_buffer[ring_head], (uint8_t*)trans->rx_buffer, PACKET_LENGTH);
        ring_head = next_head;
    }
}

// ============================================================
// WiFi 重连
// ============================================================
void handleWiFiReconnect() {
    if (!wifi_enabled) return;
    if (millis() - lastWifiCheck < wifi_retry_interval) return;
    lastWifiCheck = millis();

    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - wifi_connect_start < 8000) return;
        Serial.print("WiFi 重连中 (间隔 ");
        Serial.print(wifi_retry_interval / 1000);
        Serial.println("s)...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        wifi_connect_start = millis();
        wifi_retry_interval = min(wifi_retry_interval * 2, (unsigned long)WIFI_RETRY_MAX);
    } else {
        wifi_retry_interval = WIFI_RETRY_BASE;
    }
}

// ============================================================
// 云端上报
// ============================================================
void reportToCloud(String jsonData) {
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(String(CLOUD_HOST) + "/update");
    http.addHeader("Content-Type", "application/json");

    // 日志已打包进 jsonData, 发送前清空缓冲, 避免响应丢失后同一批日志反复上报
    logBuffer.clear();

    esp_task_wdt_reset();
    int code = http.POST(jsonData);
    if (code == 200) {
        String response = http.getString();
        parseCloudCommands(response);
    } else {
        Serial.printf("[CLOUD] Response: %d\n", code);
    }
    http.end();
    esp_task_wdt_reset();
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    prefs.begin("csvfile", false);
    last_csv_version = prefs.getUInt("version", 0);
    last_csv_done = prefs.getBool("done", false);
    Serial.printf("[CSV] NVS: last_csv_version=%u done=%d\n",
        (unsigned)last_csv_version, last_csv_done ? 1 : 0);

    esp_log_level_set("spi_slave", ESP_LOG_INFO);

    esp_task_wdt_init(15, true);
    esp_task_wdt_add(NULL);

    Serial.printf("Reset reason: %d (1=POWERON 3=SW 4=WDT 5=DEEPSLEEP 12=BROWNOUT)\n", esp_reset_reason());
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    pinMode(GPIO_IRQ, OUTPUT);
    digitalWrite(GPIO_IRQ, HIGH);
    Serial.println("[IRQ] GPIO13 初始化完成 (OUTPUT, HIGH)");

    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.mode(WIFI_STA);
    Serial.println("[WiFi] 等待 STM32 WiFi ON 指令后连接");

#if !SPI_CS_PROBE
    spi_bus_config_t bus_config = {
        .mosi_io_num     = GPIO_MOSI,
        .miso_io_num     = GPIO_MISO,
        .sclk_io_num     = GPIO_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4092
    };

    spi_slave_interface_config_t slave_interface_config = {
        .spics_io_num   = GPIO_CS,
        .flags          = 0,
        .queue_size     = QUEUE_SIZE,
        .mode           = 0,
        .post_setup_cb  = spi_post_setup_callback,
        .post_trans_cb  = spi_post_trans_callback
    };

    esp_err_t ret = spi_slave_initialize(SPI_SLAVE_HOST, &bus_config, &slave_interface_config, DMA_CHANNEL);
    if (ret != ESP_OK) {
        Serial.printf("SPI 从机初始化失败! 错误码: %d\n", ret);
        while (1) { delay(1000); }
    }
    Serial.println("[SPI] 从机初始化完成");

    gpio_set_pull_mode((gpio_num_t)GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)GPIO_CS,   GPIO_PULLUP_ONLY);

    memset(tx_buffer, 0x00, PACKET_LENGTH);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        memset(tx_pool[i], 0x00, PACKET_LENGTH);
        memset(rx_pool[i], 0x00, PACKET_LENGTH);
        memset(&trans_pool[i], 0, sizeof(trans_pool[i]));
        trans_pool[i].length    = PACKET_LENGTH * 8;
        trans_pool[i].tx_buffer = tx_pool[i];
        trans_pool[i].rx_buffer = rx_pool[i];
    }

    for (int i = 0; i < QUEUE_SIZE; i++) {
        esp_err_t qret = spi_slave_queue_trans(SPI_SLAVE_HOST, &trans_pool[i], pdMS_TO_TICKS(1000));
        if (qret != ESP_OK) {
            Serial.printf("[SPI] initial queue #%d failed: 0x%X, retrying...\n", i, qret);
            delay(100);
            qret = spi_slave_queue_trans(SPI_SLAVE_HOST, &trans_pool[i], pdMS_TO_TICKS(1000));
        }
        if (qret != ESP_OK) {
            Serial.printf("[SPI] queue #%d retry failed: 0x%X\n", i, qret);
        }
    }
    Serial.printf("[SPI] queued %d slave transactions\n", QUEUE_SIZE);
    if (xTaskCreate(spiRecycleTask, "spi_recycle", 4096, NULL, 5, NULL) == pdPASS) {
        Serial.println("[SPI] transaction recycle task started");
    } else {
        Serial.println("[SPI] recycle task create failed, loop fallback active");
    }
    last_spi_activity = millis();
#else
    gpio_install_isr_service(0);
    gpio_config_t cs_probe_cfg = {
        .pin_bit_mask = (1ULL << GPIO_CS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&cs_probe_cfg);
    gpio_isr_handler_add((gpio_num_t)GPIO_CS, cs_probe_isr, NULL);
    Serial.printf("[SPI] CS 探针模式: GPIO%d 统计边沿\n", GPIO_CS);
    last_spi_activity = millis();
#endif

    Serial.println("[Board] ESP32-C3 初始化完毕, 等待 STM32 通信...");
}

// ============================================================
// loop()
// ============================================================
void spiRecycleTask(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        spi_slave_transaction_t *ret_trans = NULL;
        esp_err_t ret = spi_slave_get_trans_result(SPI_SLAVE_HOST, &ret_trans, portMAX_DELAY);
        if (ret == ESP_OK && ret_trans != NULL) {
            esp_err_t q_ret = spi_slave_queue_trans(SPI_SLAVE_HOST, ret_trans, portMAX_DELAY);
            if (q_ret != ESP_OK) {
                static unsigned long lastQErrDbg = 0;
                if (millis() - lastQErrDbg > 10000) {
                    lastQErrDbg = millis();
                    Serial.printf("[SPI] recycle re-queue failed: 0x%X\n", q_ret);
                }
            }
        } else if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            static unsigned long lastRecoverDbg = 0;
            if (millis() - lastRecoverDbg > 10000) {
                lastRecoverDbg = millis();
                Serial.printf("[SPI] recycle get result err: 0x%X\n", ret);
            }
        }
    }
}

void loop() {
    esp_task_wdt_reset();

#if !SPI_CS_PROBE
    // 每次最多回收 QUEUE_SIZE 个已完成事务, 保证事务池持续满队列, 主控连续事务也能收到
    for (int i = 0; i < QUEUE_SIZE; i++) {
        spi_slave_transaction_t *ret_trans;
        esp_err_t ret = spi_slave_get_trans_result(SPI_SLAVE_HOST, &ret_trans, 0);
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_TIMEOUT) {
                static unsigned long lastRecoverDbg = 0;
                if (millis() - lastRecoverDbg > 10000) {
                    lastRecoverDbg = millis();
                    Serial.printf("[SPI] get_trans_result err: 0x%X\n", ret);
                }
            }
            break;
        }
        esp_err_t q_ret = spi_slave_queue_trans(SPI_SLAVE_HOST, ret_trans, 0);
        if (q_ret != ESP_OK) {
            static unsigned long lastQErrDbg = 0;
            if (millis() - lastQErrDbg > 10000) {
                lastQErrDbg = millis();
                Serial.printf("[SPI] re-queue failed: 0x%X\n", q_ret);
            }
        }
    }

    while (ring_tail != ring_head) {
        uint8_t local_buf[PACKET_LENGTH];
        uint32_t rxBits = ring_bits[ring_tail];
        memcpy(local_buf, ring_buffer[ring_tail], PACKET_LENGTH);
        ring_tail = (ring_tail + 1) % RING_BUF_SIZE;
        last_spi_activity = millis();

        if (rxBits == (uint32_t)(PACKET_LENGTH * 8)) {
            parsePacketLocal(local_buf);
        } else {
            Serial.printf("[SPI_WARN] #%u 帧边界异常, 丢弃: %u bits (%u bytes)\n",
                (unsigned)transaction_count, (unsigned)rxBits, (unsigned)(rxBits / 8));
        }
    }

    static unsigned long lastSpiIdleDbg = 0;
    if (millis() - lastSpiIdleDbg > 30000) {
        lastSpiIdleDbg = millis();
        Serial.println("========== SPI IDLE DEBUG ==========");
        Serial.printf("  trans_cnt=%u last_activity=%lu ms ago cs_low=%u us\n",
            (unsigned)transaction_count, (unsigned long)(millis() - last_spi_activity),
            (unsigned)last_cs_active_us);
        Serial.printf("  ring: head=%u tail=%u drops=%u\n",
            (unsigned)ring_head, (unsigned)ring_tail, (unsigned)ring_drop_count);
        Serial.printf("  gpio: CS=%d SCLK=%d MOSI=%d IRQ=%d irq_low=%d\n",
            gpio_get_level((gpio_num_t)GPIO_CS),
            gpio_get_level((gpio_num_t)GPIO_SCLK),
            gpio_get_level((gpio_num_t)GPIO_MOSI),
            gpio_get_level((gpio_num_t)GPIO_IRQ), irq_low);
        Serial.println("====================================");
    }
#endif

#if SPI_CS_PROBE
    static unsigned long lastCsProbeDbg = 0;
    if (millis() - lastCsProbeDbg >= 1000) {
        lastCsProbeDbg = millis();
        uint32_t since_us = 0;
        if (cs_probe_last_edge_us != 0) {
            since_us = (uint32_t)(esp_timer_get_time() - cs_probe_last_edge_us);
        }
        Serial.printf("[CS_PROBE] edges=%u CS=%d last_edge=%u us ago\n",
            cs_probe_edge_count, gpio_get_level((gpio_num_t)GPIO_CS), since_us);
    }
#endif

    spi_connected = (millis() - last_spi_activity < SPI_TIMEOUT_MS);

    fileTransferTick();
    fileFlushPending();

    handleWiFiReconnect();

    if (!fileState.active &&
        millis() - lastCloudReport > cloudReportInterval &&
        WiFi.status() == WL_CONNECTED) {
        String json = buildDataJson();
        reportToCloud(json);
        lastCloudReport = millis();
    }
}
