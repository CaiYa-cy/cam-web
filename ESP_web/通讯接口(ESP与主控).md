# ESP32-C3 与 STM32 主控 SPI 通讯接口文档

> 版本: v3.3  
> 日期: 2026-08-06  
> 硬件: ESP32-C3 (SPI 从机) ↔ STM32 (SPI 主机)

---

## 1. 硬件接线

| 信号 | ESP32-C3 | STM32 | 方向 | 说明 |
|------|----------|-------|------|------|
| SCK  | GPIO2    | SCK   | STM32→ESP32 | 时钟, 由 STM32 产生 |
| MOSI | GPIO3    | MOSI  | STM32→ESP32 | 主→从数据 |
| MISO | GPIO10   | MISO  | ESP32→STM32 | 从→主数据 |
| CS   | GPIO7    | CS    | STM32→ESP32 | 帧同步, 低有效, 上升沿锁存 |
| IRQ  | GPIO13   | EXTI  | ESP32→STM32 | 从机主动上报, 空闲高, 有数据拉低 |
| GND  | GND      | GND   | - | 共地 |

- 全部 **3.3V 直连**
- IRQ 建议加 **10kΩ 上拉**，防止上电误触发
- 上电顺序: **ESP32 先上电** 初始化 SPI 从机 → STM32 后上电

---

## 2. SPI 配置参数

| 参数 | 值 |
|------|-----|
| 模式 | Mode 0 (CPOL=0, CPHA=0) |
| 数据位宽 | 8 bit |
| 帧长 | **固定 128 字节** |
| 速率 | ≤ 5 MHz (建议 5 MHz) |
| DMA | ESP32 使用 GDMA (`SPI_DMA_CH_AUTO`) |
| CS 控制 | STM32 软件管理, ESP32 硬件 CS 边沿触发 |
| 帧长不可变原因 | ESP32 作为从机无法预知主机发送长度, 固定 128 字节是唯一可靠方案 |

---

## 3. 帧格式

128 字节固定帧, 不足末尾填 `0x00`。

| 字节偏移 | 字段 | 长度 | 说明 |
|---------|------|------|------|
| 0 | CMD | 1 | 命令类型 |
| 1 | SUBCMD | 1 | 子命令 |
| 2 | LEN | 1 | 有效载荷长度 (≤123) |
| 3 ~ 125 | PAYLOAD | 123 | 数据载荷, 常规命令为 ASCII/UTF-8; 文件 DATA 帧为原始字节 |
| 126 | SEQ | 1 | 事务 ID (0~255 循环), 请求-回复匹配 |
| 127 | RESERVED | 1 | 保留, 填 0x00 |

> **二进制载荷例外**：`CMD_CSV_UPLOAD (0x70)` 的 DATA 子帧 (`0x02`) 中, PAYLOAD 前 2 字节为帧号 (小端), 其后为 CSV 原始文件字节 (UTF-16 等编码含大量 `0x00`, 属正常内容)。主控必须按 `LEN` 精确提取字节, **禁止**按字符串、`0x00` 或换行截断。

---

## 4. 命令协议总表

### 4.1 主命令 (CMD)

| CMD | 值 | 方向 | 说明 |
|-----|-----|------|------|
| `CMD_HEARTBEAT`    | `0x00` | STM32→ESP | 心跳 / 空闲 |
| `CMD_DATA_UPDATE`  | `0x10` | STM32→ESP | 数据更新 (状态/进度/温度) |
| `CMD_SYS_CTRL`     | `0x20` | STM32→ESP | 系统控制 (WiFi 开关) |
| `CMD_STATUS_QUERY` | `0x30` | STM32→ESP | 状态查询 |
| `CMD_PROCESS_CTRL` | `0x40` | **ESP→STM32** | 贴片流程控制 (网页下发) |
| `CMD_LOG_DATA`     | `0x50` | STM32→ESP | 日志文本 |
| `CMD_HEATER_CTRL`  | `0x60` | **ESP→STM32** | 加热台控制 (网页下发) |
| `CMD_CSV_UPLOAD`   | `0x70` | **ESP→STM32** | CSV 文件上传 (START/DATA/END/CANCEL) |
| `CMD_FILE_CTRL`    | `0x71` | STM32→ESP | 文件传输回执 (NEXT/RESULT/CANCEL_ACK) |

---

### 4.2 数据更新 (CMD=0x10, STM32→ESP)

| SUBCMD | 值 | PAYLOAD 示例 | 说明 |
|--------|-----|-------------|------|
| `SUB_PROGRESS`    | `0x01` | `"25/100"` | 贴片进度 |
| `SUB_STATUS`      | `0x02` | `"SMTing"` | 设备状态: Waiting / Importing / SMTing / Heating / Finished |
| `SUB_HEATER_ON`   | `0x03` | `"1"` 或 `"0"` | 加热台是否开启 |
| `SUB_HEATER_TEMP` | `0x04` | `"150.0"` | 加热台当前温度 |

---

### 4.3 系统控制 (CMD=0x20, STM32→ESP)

| SUBCMD | 值 | 说明 |
|--------|-----|------|
| `CTRL_WIFI_ON`      | `0x01` | 开启 ESP32 WiFi 连接 (使用当前凭据) |
| `CTRL_WIFI_OFF`     | `0x02` | 断开 ESP32 WiFi |
| `CTRL_WIFI_CONNECT` | `0x03` | 连接指定 WiFi, PAYLOAD=`SSID\0PASSWORD` |

> **连接时机**：ESP32 上电后**不自动连接** WiFi, 处于待机状态; 只有收到 `CTRL_WIFI_ON (0x01)` 后才用当前凭据执行 `WiFi.begin(ssid, password)` 开始连接。`CTRL_WIFI_CONNECT (0x03)` 下发新 SSID/密码后会立即切换并连接, 不需要先发 ON。

> **CTRL_WIFI_CONNECT (0x03) PAYLOAD 格式**  
> 使用 `0x00` 字节分隔 SSID 和密码: `SSID\0PASSWORD`, `LEN = SSID长度 + 1 + 密码长度`。
> - SSID: 1~32 字节 (UTF-8 单字节场景下即字符数)
> - 密码: 8~63 字节, 允许包含空格
> - ESP32 校验失败时不切换 WiFi, 通过串口打印原因; 校验成功后断开旧连接并重新 `WiFi.begin(ssid, password)`

---

### 4.4 状态查询 (CMD=0x30, STM32→ESP)

| SUBCMD | 值 | 说明 | ESP 回复 (tx_buffer SUBCMD) |
|--------|-----|------|---------------------------|
| `QUERY_FAULT` | `0x01` | 查询故障码 | `RSP_FAULT (0xF1)`, payload=故障码HEX |
| `QUERY_WIFI`  | `0x02` | 查询 WiFi 状态 | `RSP_WIFI_STAT (0xF2)`, payload=`"1,-45"` (连接状态,RSSI) |
| `QUERY_ALL`   | `0x03` | 查询全部状态 | `RSP_COMPOSITE (0xFF)`, payload=`"25/100|SMTing|1|150.0|00"` |

> 注意：SPI 全双工特性导致回复天然晚一轮。STM32 通过 SEQ 字段匹配请求与回复。

---

### 4.5 贴片流程控制 (CMD=0x40, ESP→STM32) ★ 新增

网页端按钮 → 云端 → ESP32 → SPI → STM32

| SUBCMD | 值 | 对应网页按钮 | 说明 |
|--------|-----|-------------|------|
| `PROC_START`  | `0x01` | **开始** | 设备回原点初始化后, 从 P4 对准开始全部进程 |
| `PROC_PAUSE`  | `0x02` | **暂停** | 临时暂停, 电机回原点, 可选继续或结束 |
| `PROC_RESUME` | `0x03` | **继续** | 暂停后恢复流程 |
| `PROC_STOP`   | `0x04` | **结束** | 结束此次任务 |
| `PROC_ESTOP`  | `0x05` | **急停** | 立即停止, 设备不再进行后续动作 |

PAYLOAD 为空字符串。

---

### 4.6 加热台控制 (CMD=0x60, ESP→STM32) ★ 新增

网页端按钮 → 云端 → ESP32 → SPI → STM32

| SUBCMD | 值 | 对应网页按钮 | 说明 |
|--------|-----|-------------|------|
| `HEAT_START` | `0x10` | **开启加热** | 启动加热台加热任务 |
| `HEAT_STOP`  | `0x11` | **暂停加热** | 停止加热台加热任务 |

PAYLOAD 为空字符串。

> **状态同步**：ESP32 收到网页 `HEAT_START/HEAT_STOP` 后总是把命令发给 STM32, 不再因本地 `heater_on` 状态忽略重复命令; 调用 `sendCommandToSTM32()` 成功后本地同步 `heater_on` (START→true, STOP→false), 并随云端 JSON 上报 `heater_state` (`"HEATING"` / `"IDLE"`)。

---

### 4.7 日志数据 (CMD=0x50, STM32→ESP) ★ 新增

| SUBCMD | 值 | PAYLOAD | 说明 |
|--------|-----|---------|------|
| `SUB_LOG_TEXT` | `0x01` | UTF-8 日志文本 (≤123 字节) | STM32 每执行一步发送一条日志 |

ESP32 收到后缓存到 `logBuffer`，下一次云端上报时打包进 JSON 的 `"logs":[...]` 数组发给网页。

---

### 4.8 ESP 响应类型

| 响应 | 值 (tx_buffer[1]) | 说明 |
|------|-------------------|------|
| `RSP_IDLE`      | `0x00` | 空闲 (无响应) |
| `RSP_FAULT`     | `0xF1` | 故障码查询回复 |
| `RSP_WIFI_STAT` | `0xF2` | WiFi 状态查询回复 |
| `RSP_COMPOSITE` | `0xFF` | 全部状态查询回复 |

---

### 4.9 CSV 文件上传 (CMD=0x70, ESP→STM32) ★ 新增

网页上传 CSV → 服务器暂存 → ESP32 分块拉取 → SPI 逐帧发送。传输期间 `CMD_CSV_UPLOAD` 独占 `tx_buffer`, 主控按帧确认后 ESP32 才发下一帧。

| SUBCMD | 值 | PAYLOAD 格式 | 说明 |
|--------|-----|-------------|------|
| `SUB_CSV_START`  | `0x01` | `len=<size>,frames=<n>,crc32=<hex8>` | 会话开始, 主控清空重组缓冲, 保存 total/frames/crc32 |
| `SUB_CSV_DATA`   | `0x02` | 前 2 字节帧号 (小端) + 最多 121 字节原始文件字节 | 帧号从 0 开始, 内容可为任意字节 |
| `SUB_CSV_END`    | `0x03` | `crc32=<hex8>` | 数据发完, 主控重组后校验 |
| `SUB_CSV_CANCEL` | `0x04` | 空 | 网页取消导入, ESP32 通知主控放弃本次会话 |

补充约定：

- 数据帧净数据固定 **121 字节/帧** (最后一块可能少于 121), 帧数 = `ceil(size / 121)`。
- DATA 帧的 `LEN = 2 + 实际数据长度`; 帧号以 PAYLOAD 前 2 字节为准, **不能**用 Byte 126 (SEQ, 仅调试用途)。
- ESP32 填充 DATA 帧使用 `memcpy` 按长度复制, 禁止 `strlen`/String (内容含 `0x00`)。

**云端接口 (relay_server.py)**

| 接口 | 说明 |
|------|------|
| `POST /upload` | Body = CSV 原始文件字节, `Content-Type: application/octet-stream`; 上限 102400 字节, 超限返回 413; 成功返回 `{"ok":true,"id":"csv-current","version":N,"size":N,"crc32":"xxxxxxxx"}`; 新上传覆盖旧文件, version 自增 |
| `GET /file?version=V&offset=N` | 返回最多 121 字节原始二进制块; 响应头 `X-Version` / `X-Size` / `X-Offset` / `X-Next-Offset` / `X-Crc32` / `X-Done`; 取完时 `X-Done=1`; 文件不存在/过期返回 404, version 不匹配返回 409 |

- ESP32 用 `HTTPClient::getStream().readBytes()` 读取原始字节, 禁止 `getString()`。
- 服务器只保留一份最新文件, 30 分钟无拉取活动即过期。
- ESP32 收到 404 上报 `expired`, 收到 409 上报 `overwritten`; 拉块失败不推进 `offset`, 1 秒后重试, 连续 5 次失败进入 `network_error`, 之后每 30 秒自动重试。
- NVS 保存最近一次成功导入的 version; 重启后收到相同 version 的开始命令时直接上报 `success` 并跳过传输。

---

### 4.10 文件控制回执 (CMD=0x71, STM32→ESP) ★ 新增

| SUBCMD | 值 | PAYLOAD 格式 | 说明 |
|--------|-----|-------------|------|
| `SUB_FILE_NEXT`       | `0x01` | 前 2 字节期望下一帧号 (小端) | 每收一帧 DATA 回一次, 用于确认与推进 |
| `SUB_FILE_RESULT`     | `0x02` | `ok` 或 `fail:<code>` | START 接受/拒绝, END 校验结果 |
| `SUB_FILE_CANCEL_ACK` | `0x03` | 空 | 取消确认 (可选) |

`RESULT` 错误码：

| code | 含义 |
|------|------|
| `fail:1` | CRC32 校验失败 |
| `fail:2` | 重组后总长度与 START 的 `len` 不符 |
| `fail:3` | 帧号不连续 |
| `fail:4` | 重组缓冲不足 |

---

### 4.11 心跳与云端状态上报 (ESP32→云端→网页) ★ 新增

ESP32 在 WiFi 已连接后, 通过现有云端 `/update` 上报通道持续发送心跳:

- 每次 `buildDataJson()` 输出 `"heartbeat":1` 字段;
- 正常上报间隔 500ms, CSV 文件传输期间 5000ms;
- 网页收到任意带 `heartbeat` 的 JSON 即判定“贴片机已连接”;
- 连续 30 秒未收到心跳, 网页显示“贴片机未连接”。

该心跳走 WiFi/云端链路, 与 SPI 无关; 主控无需额外发送心跳指令。

`buildDataJson()` 上报的 JSON 字段:

| 字段 | 类型 | 说明 |
|------|------|------|
| `t` | number | ESP32 `millis()` 时间戳 |
| `progress` | string | 贴片进度, 如 `"25/100"` |
| `status` | string | Waiting / Importing / SMTing / Heating / Finished |
| `heater_on` | number | 加热台开关状态 0/1 |
| `heater_temp` | number | 加热台温度, 有效时存在 |
| `fault` | string | 故障码 HEX, 如 `"00"` |
| `wifi` | number | WiFi 连接状态 0/1 |
| `heartbeat` | number | 恒为 1 |
| `spi` | number | SPI 在线状态 0/1, 3 秒内无 SPI 事务视为离线 |
| `file_status` | string | idle/transferring/verifying/success/failed/expired/overwritten/cancelled/network_error |
| `file_progress` | number | 0~100 |
| `logs` | array | 日志文本数组, 最多 20 条 |
| `heater_state` | string | `"HEATING"` 或 `"IDLE"`, 与 `heater_on` 同步 |

---

### 4.12 网页控制命令云端链路 ★ 新增

网页按钮 → WebSocket → relay_server 命令队列 → ESP32 轮询 → SPI → STM32:

1. 网页 `sendControlCommand()` 通过 WebSocket 发送 `{"cmd":96,"subcmd":16,"payload":""}` 这类 JSON (cmd/subcmd 为数值); 网页端对相同 `cmd/subcmd` 另有 2 秒去重。
2. `relay_server.py` 将命令加入 `commands_queue` (上限 10 条), 相同 `(cmd,subcmd,payload)` 3 秒内去重。
3. ESP32 每 500ms POST `/update` (CSV 文件传输期间 5000ms), 服务器在响应中返回 `{"commands":[...]}` 并清空队列。
4. ESP32 `parseCloudCommands()` 解析后交给 `processWebCommand()`, 再通过 SPI 发给 STM32。

CSV 导入命令由网页直接发送, 不经过 `sendControlCommand()`:

| 命令 | JSON | 说明 |
|------|------|------|
| 开始导入 | `{"cmd":0x70,"subcmd":0x01,"payload":"v=<version>"}` | `version` 为 `POST /upload` 返回值; 固件 `sscanf(payload,"v=%u",&v)` 解析 |
| 取消导入 | `{"cmd":0x70,"subcmd":0x04,"payload":""}` | 固件按 `SUB_CSV_CANCEL (0x04)` 处理 |

> **已知不一致**: 当前 `index.html` 取消按钮发送 `{"cmd":0x70,"subcmd":0x02,"payload":""}`, 固件只按 `0x04` 处理, 0x02 会被忽略, 取消实际不生效; `CSV导入技术方案.md` 也按 0x02 描述。建议统一为 `0x04`。

---

## 5. 双向通信流程

### 5.1 场景 A: STM32 主动下发 (主→从)

```
STM32 检查 IRQ 电平
  ├─ IRQ = HIGH → 执行场景 A
  │    1. 拉低 CS
  │    2. 发送 128 字节 (MOSI), 同时接收 128 字节 (MISO, 上一轮 ESP 回复)
  │    3. 拉高 CS → ESP 回调触发
  │    4. 延时 2ms (防抖)
  │    5. 再次检查 IRQ (兜底竞态)
  │
  └─ IRQ = LOW → 优先执行场景 B
```

**CS 超时保护**：拉低 CS 后 100ms 内未完成传输, 强制拉高, 报通信异常。

**SEQ 匹配**：发送请求时记录 `(SEQ, 内容)`, 收到 MISO 后提取 SEQ→匹配→处理该请求的回复。

---

### 5.2 场景 B: ESP32 主动上报 (从→主) ★ 新增

```
ESP32 收到网页命令
  │
  ├─ 填充 tx_buffer (CMD_PROCESS_CTRL / CMD_HEATER_CTRL / CMD_CSV_UPLOAD)
  ├─ digitalWrite(GPIO_IRQ, LOW)   // 拉低 IRQ = 通知 STM32
  │
  ▼
STM32 EXTI 中断触发
  ├─ ISR: esp32_irq_flag = 1
  │
  ▼
STM32 主循环检测 esp32_irq_flag
  ├─ 拉低 CS
  ├─ 发送 128 字节哑元 (全 0xFF), 从 MISO 读取 ESP 数据
  ├─ 拉高 CS → ESP 回调触发
  ├─ 延时 2ms
  ├─ 清 esp32_irq_flag
  │
  ▼
ESP32 spi_post_trans_callback
  ├─ 检测 irq_low == true
  ├─ digitalWrite(GPIO_IRQ, HIGH)   // 拉高 IRQ = 数据已被读取
  └─ memset(tx_buffer, 0x00, 128)   // 整块清空, 防止主控后续轮询重复读到残留命令
```

**互斥规则**：STM32 每次主动下发前 (场景 A) 必须先检查 IRQ。IRQ 为低则优先执行场景 B，处理完后再下发。

---

### 5.3 CSV 文件会话逐帧确认 (场景 B 扩展) ★ 新增

文件会话期间, ESP32 与 STM32 按“拉取式确认”逐帧推进, 每发一帧必须等主控回执后才发下一帧:

```text
1. ESP32 填 START 帧 (0x70/0x01), IRQ LOW
2. STM32 读取 START, 同帧 MOSI 回 RESULT(ok / fail:<code>)
3. ESP32 解析 RESULT:
     ok   → 从服务器拉取 DATA#0, 填帧, IRQ LOW
     fail → 中止并上报 failed
4. STM32 读取 DATA#0, 回 NEXT(1)
5. ESP32 解析 NEXT(1) 与本地期望一致 → 拉取 DATA#1, 填帧, IRQ LOW
     不一致 → 重发当前帧, 最多重发 3 次, 之后中止
6. 重复直到 DATA#N (最后一帧)
7. STM32 回 NEXT(N+1), ESP32 解析后填 END 帧 (0x70/0x03), IRQ LOW
8. STM32 读取 END, 重组校验 (总长度 + 帧连续性 + CRC32)
9. STM32 回 RESULT(ok / fail:<code>)
10. ESP32 解析结果, 会话结束, 通过云端上报 file_status/file_progress
```

网络拉块失败时 `offset` 不推进, 1 秒后重试; 连续 5 次失败上报 `network_error`, 之后每 30 秒自动重试。

IRQ 电平语义保持不变：

- ESP32 每填好一帧就拉低 IRQ, 表示“有新帧可读”。
- STM32 事务完成后, ESP32 回调拉高 IRQ, 表示“本轮已读完”。
- ESP32 在内存准备好下一帧后再次拉低 IRQ。
- STM32 从 START 进入接收模式后, 每次看到 IRQ LOW 就继续读, 直到收到 END 或 CANCEL。

文件会话期间：

- STM32 **不要**主动发送状态查询 (0x30)、日志 (0x50) 等无关帧, 避免挤占 ESP32 的 rx ring。
- ESP32 收到 0x30 状态查询时暂不回复, 会话结束后补回最近一条; 收到网页 0x40/0x60 控制命令时缓存最多 1 条, 已有缓存时用新命令覆盖, 会话结束后补发。

---

## 6. 日志文本协议

STM32 在执行贴片流程的每一步时, 通过 `CMD_LOG_DATA` 发送日志文本。

### 6.1 预期日志序列

```
启动
开始第一次对准吸嘴
吸嘴第一次对准完成                    ← 失败则发 "吸嘴对准失败"
开始扫描mark点
mark点1扫描完成                       ← 失败则发 "mark点识别失败"
mark点2扫描完成
mark点3扫描完成
完成mark点识别
开始第二次对准吸嘴
第二次吸嘴对准完成                    ← 失败则发 "吸嘴对准失败"
开始识别元件
元件识别完成                          ← 失败则发 "元件无法识别"
开始修正元件偏差
  ├─ 若未吸取到元件: 重新识别 (最多3次)
  │   └─ 连续3次失败: "元件吸取失败"
  └─ 若偏差无法修正: "偏差修正失败"
元件偏差修正完成
开始贴装
贴装完成
  └─ 循环 P1+P3 直到全部元件贴装完成
加热台开始加热
达到预定温度
加热台加热完毕
结束
```

### 6.2 日志传递路径

```
STM32 send_log("开始扫描mark点")
  → SPI (CMD=0x50, SUB=0x01, payload="开始扫描mark点")
  → ESP32 parsePacketLocal → logBuffer.push_back("开始扫描mark点")
  → 下一次 buildDataJson() 生成 {"logs":["开始扫描mark点",...]}
  → reportToCloud() POST 到云端
  → relay_server 广播给浏览器
  → 浏览器 onmessage → addLog("开始扫描mark点")
```

---

## 7. 容错机制

| 机制 | 说明 |
|------|------|
| CS 超时 | STM32: 拉低 CS 后 100ms 未完成传输, 强制拉高, 报通信异常 |
| IRQ 防抖 | 场景 B 完成后延时 2ms 再检查 IRQ, 防止 ESP GPIO 释放延迟误判 |
| 上电顺序 | ESP32 先上电初始化, STM32 后上电 |
| 互斥检查 | STM32 场景 A 前必查 IRQ, IRQ 低则优先场景 B |
| SEQ 匹配 | 通过 Byte 126 的 SEQ 匹配异步请求与回复 |
| WiFi 重连 | ESP32 指数退避重连 (1s → 2s → 4s → ... → 60s) |
| 指令队列 | 网页端对相同 cmd/subcmd 2 秒去重, 服务端对相同 (cmd,subcmd,payload) 3 秒去重 (队列上限 10 条); 网页发命令时若 IRQ 忙或 SPI 通道忙, ESP32 缓存该命令; 文件会话期间网页 0x40/0x60 命令缓存最多 1 条, 已有缓存时用新命令覆盖, 通道空闲后补发 |
| 帧号校验 | STM32 收到 DATA 帧号不连续时回 `fail:3`; ESP32 收到 NEXT 与期望不符时重发当前帧, 最多重发 3 次, 之后中止 |
| IRQ 等待超时 | ESP32 填帧后 3 秒内未完成 SPI 事务, 中止文件会话并上报 failed |
| CRC 校验 | STM32 在 END 后按标准 CRC32 (与 Python `zlib.crc32` 一致) 校验, 失败回 `fail:1` |
| 文件过期/覆盖 | 服务器文件 30 分钟无拉取活动即过期; version 不匹配时 ESP32 中止, 上报 expired / overwritten |
| 会话互斥 | 文件会话期间 STM32 不主动发 0x30/0x50 等无关帧; 收到新 START 或 CANCEL 时丢弃未完成数据 |
| 网络拉块失败 | 拉块失败不推进 offset, 1 秒后重试; 连续 5 次失败上报 `network_error`, 之后每 30 秒自动重试 |
| 重启去重 | NVS 保存最近成功导入的 version; 重启后相同 version 直接 success, 不重复导入 |
| 开始命令排队 | 通道忙或旧会话进行中收到开始导入命令时, 先取消旧会话或排队, 通道空闲后发起新会话 |

---

## 8. 帧示例

### 8.1 网页发送 "开始贴片" (ESP→STM32)

```
字节:  [0]   [1]   [2]   [3..125]  [126]  [127]
内容:  0x40  0x01  0x00  0x00...   0x00   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑空payload  ↑SEQ  ↑保留
      贴片流程 开始   长度=0
```

> ESP→STM32 的网页控制命令 (0x40/0x60) 帧 `SEQ` 固定填 `0x00`。

### 8.2 STM32 发送日志 "开始扫描mark点" (STM32→ESP)

```
字节:  [0]   [1]   [2]   [3............21]  [126]  [127]
内容:  0x50  0x01  0x13  开始扫描mark点...   0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑payload(19字节)    ↑SEQ  ↑保留
      日志   文本   =19   UTF-8 字符串
```

### 8.3 STM32 查询全部状态 (STM32→ESP)

```
请求帧:
字节:  [0]   [1]   [2]   [3..125]  [126]  [127]
内容:  0x30  0x03  0x00  0x00...   0x05   0x00

回复帧 (下一次 SPI 传输时 ESP 通过 tx_buffer 返回):
字节:  [0]   [1]   [2]   [3............26]  [126]  [127]
内容:  0x00  0xFF  0x18  25/100|SMTing|...   0x05   0x00
      ↑CMD  ↑RSP  ↑LEN  ↑payload(24字节)     ↑回SEQ ↑保留
      心跳   COMP   =24   全部状态信息
```

### 8.4 STM32 下发指定 WiFi 凭据 (STM32→ESP)

```
字节:  [0]   [1]   [2]   [3...............19]  [126]  [127]
内容:  0x20  0x03  0x11  MySSID\0MyPassword...  0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑payload(17字节)         ↑SEQ  ↑保留
      系统控制 连接  =17  SSID(6)+分隔符(1)+密码(10)
```

> 示例中 `LEN = 6 + 1 + 10 = 17`。若密码含空格 (如 `"My Password"`), 空格字节原样放入 PAYLOAD, 不受影响。

---

### 8.5 CSV 上传 START 帧 (ESP→STM32)

```text
字节:  [0]   [1]   [2]   [3...................35]  [126]  [127]
内容:  0x70  0x01  0x21  len=6016,frames=50,crc32=abcdef12  0x00  0x00
      ↑CMD  ↑SUB  ↑LEN  ↑payload(33字节)                    ↑SEQ  ↑保留
      CSV   开始   =33   会话参数
```

### 8.6 CSV 上传 DATA 帧 (ESP→STM32, 含 `0x00` 原始字节)

```text
字节:  [0]   [1]   [2]   [3]   [4]   [5.............125]  [126]  [127]
内容:  0x70  0x02  0x7B  0x00  0x00  0xFF 0xFE 0x00 0x00 ...  0x00   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑帧号低8位 ↑帧号高8位 ↑CSV原始字节(最多121)
      CSV   DATA  =123  帧号=0 (小端)              内容可为任意值
```

> 所有 DATA 帧的 `SEQ` 填帧号低 8 位 (仅调试用途), 帧号以 `[3]`、`[4]` 两个字节为准。

### 8.7 CSV 上传 END 帧 (ESP→STM32)

```text
字节:  [0]   [1]   [2]   [3..............16]  [126]  [127]
内容:  0x70  0x03  0x0E  crc32=abcdef12       0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑payload(14字节)      ↑SEQ  ↑保留
      CSV   结束   =14
```

> END 帧 `SEQ` 填发送完成后的下一帧号低 8 位 (即总帧数 & 0xFF, 仅调试用途)。

### 8.8 文件控制回执 (STM32→ESP)

```text
NEXT 帧 (DATA#0 确认):
字节:  [0]   [1]   [2]   [3]   [4]   [5..125]  [126]  [127]
内容:  0x71  0x01  0x02  0x01  0x00  0x00...    0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑期望下一帧号=1 (小端)

RESULT 帧 (START 接受 / END 校验通过):
字节:  [0]   [1]   [2]   [3]   [4..125]  [126]  [127]
内容:  0x71  0x02  0x02  0x6F  0x6B      0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑'o' ↑'k'

RESULT 帧 (END 校验失败, CRC 错误):
字节:  [0]   [1]   [2]   [3.......8]  [126]  [127]
内容:  0x71  0x02  0x06  fail:1        0x??   0x00
      ↑CMD  ↑SUB  ↑LEN  ↑payload(6字节)
```

---

## 9. STM32 端实现要点

1. **IRQ 中断**：配置 EXTI 下降沿中断, ISR 中只置标志位 `esp32_irq_flag = 1`
2. **场景互斥**：每次 SPI 传输前检查 IRQ 电平 → 决定场景 A 或 B
3. **SEQ 管理**：维护 `(seq, 请求类型)` 映射表, 收到 MISO 后匹配回复
4. **CS 超时**：`HAL_GetTick()` 监控, 100ms 超时强制拉高
5. **命令处理**：收到 `CMD_PROCESS_CTRL`、`CMD_HEATER_CTRL`、`CMD_CSV_UPLOAD` 后分发到对应执行函数
6. **日志发送**：每个执行步骤调用 `send_log(const char* text)` 构建 CMD_LOG_DATA 帧
7. **CSV 接收模式**：识别 `CMD_CSV_UPLOAD (0x70)` 进入文件接收模式; 收到 START (`0x01`) 时清空重组缓冲, 保存 `total`、`frames`、`crc32`
8. **回执**：文件会话期间, 每个 SPI 事务的 MOSI 侧按当前阶段回 `CMD_FILE_CTRL (0x71)` 帧:
   - START 后回 `RESULT(ok / fail:<code>)`
   - 每帧 DATA 后回 `NEXT(期望下一帧号, 小端 2 字节)`
   - END 后回 `RESULT(ok / fail:<code>)`
   - 收到 CANCEL 可回 `CANCEL_ACK`
9. **DATA 重组**：按 `OFF_LEN` 精确提取 DATA 内容, 逐字节追加到重组缓冲; 不能按行、不能按字符串、不能依赖 `0x00` 截断 (UTF-16 CSV 含大量 `0x00` 属正常)
10. **帧号校验**：校验帧号连续性 (帧号以 PAYLOAD 前 2 字节小端值为准), 不连续回 `fail:3` 并丢弃会话
11. **缓冲预留**：重组缓冲建议 **128KB** (文件上限 100KB + 余量)
12. **END 校验**：全部帧收完后, 按标准 CRC32 校验 (与 Python `zlib.crc32` 一致), 同时校验总长度与 START 的 `len` 是否一致; 成功才把完整原文档交给主控已有 CSV 解析程序
13. **会话互斥**：文件会话期间 (START 后、RESULT 前) 不主动发送状态查询/日志等无关帧; 收到 CANCEL 或新 START 时丢弃未完成数据

---

## 10. 变更记录

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | - | 初始版本: CMD 0x10/0x20/0x30/0x00 |
| v2.0 | 2026-07-31 | 新增 IRQ 场景 B、CMD_PROCESS_CTRL (0x40)、CMD_LOG_DATA (0x50)、CMD_HEATER_CTRL (0x60) |
| v2.1 | 2026-08-02 | 新增 CTRL_WIFI_CONNECT (0x20/0x03): STM32 下发 SSID/密码, ESP32 校验后动态切换 WiFi |
| v3.0 | 2026-08-05 | 新增 CSV 文件上传协议 CMD 0x70/0x71、逐帧确认时序、主控重组/CRC32 校验要求; 补充二进制载荷说明, 修正日志与状态查询示例长度, 移除未实现的 ESP32 CS 200ms 自复位描述 |
| v3.1 | 2026-08-05 | ESP32 上电后不再自动连接 WiFi, 收到 CTRL_WIFI_ON (0x20/0x01) 后才连接; 新增 `heartbeat` 心跳字段, 网页连续 30 秒未收到心跳显示“贴片机未连接” |
| v3.2 | 2026-08-06 | 网页命令帧 SEQ 固定 0x00; 加热命令不再按 heater_on 状态忽略, 发送成功后本地同步并上报 `heater_state`; 命令缓存满时用新命令覆盖; SPI 事务完成后整块清空 tx_buffer; 补充云端 JSON 字段与网页命令链路 |
| v3.3 | 2026-08-06 | 补充云端 CSV 接口 (POST /upload、GET /file) 与网页开始/取消导入命令; 补充 NVS 重启去重、网络拉块失败重试、命令队列上限与双重去重; NEXT 不一致改为“最多重发 3 次”; 标注 index.html 取消命令 0x02 与固件 0x04 不一致 |
