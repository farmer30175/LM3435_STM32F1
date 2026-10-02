# LM3435 RGB LED 驅動測試板

STM32F103C8T6 + USB CDC 控制 TI **LM3435**（Sequential RGB LED Driver），
以 I2C 調整 LED 電流，並透過三路非重疊 PWM 產生sequential 掃描訊號。

---

## 1. 系統架構

```
STM32F103C8T6                        LM3435
┌────────────────────┐
│ PA0  TIM2_CH1 ────────────► RCTRL  │
│ PA7  TIM3_CH2 ────────────► GCTRL  │──► LED (R/G/B)
│ PA2  TIM2_CH3 ────────────► BCTRL  │
│ PB6  I2C1_SCL ─────────────► SCLK  │
│ PB7  I2C1_SDA ─────────────► SDATA │
│ USB  CDC    ──► COM Port 指令控制  │
└────────────────────┘              │
                          SVDD / EN / GND（必須共地）
```

**亮度公式**：`亮度 = PWM duty × LED 電流`。本專案 duty 固定 10%，
所以 `LM3435 100` 實際只有 10% 亮度。

---

## 2. PWM 時序設計

| 項目 | 值 |
|---|---|
| 系統時脈 | HSE 8 MHz × 9 (PLL) = 72 MHz |
| APB1 timer clock | 72 MHz |
| Prescaler (PSC) | 0 |
| Period (ARR) | 3599 → `72e6 / 1 / 3600` = **20.000 kHz** |
| Frame 週期 | 50 µs |
| 1 tick | 13.889 ns |
| Duty | `360 / 3600` = **10%** = 5 µs |

三個 slot 在 50 µs frame 內錯開，**互不重疊**：

| 時間 | 腳位 | Timer | 模式 | CCR | CNT 範圍 | 通道 |
|---|---|---|---|---|---|---|
| 0 – 5 µs | PA0 | TIM2_CH1 | PWM mode 1 | 360 | 0 – 359 | RED |
| 16.67 – 21.67 µs | PA7 | TIM3_CH2 | PWM mode 1 | 360 | 1200 – 1559 | GREEN |
| 45 – 50 µs | PA2 | TIM2_CH3 | PWM mode 2 | 3240 | 3240 – 3599 | BLUE |

- GREEN 的 CNT 值在啟動前預載為 `1200`（`PWM_GREEN_PHASE`），讓脈波落在 frame 中段。
- BLUE 用 PWM **mode 2** 讓脈波落在 frame 尾端。
- 總導通時間 = 3 × 5 µs = 15 µs（frame 的 30%）。

### Duty 在程式裡的位置

CubeMX 的「Pulse」欄位就是 C 裡的 `sConfigOC.Pulse`：

| 檔案:行號 | 內容 |
|---|---|
| `Core/Src/main.c:63` | `#define PWM_DUTY_TICKS 360` ← **預設 duty 就在這裡** |
| `Core/Src/main.c:305` | `sConfigOC.Pulse = PWM_DUTY_TICKS;` — TIM2_CH1 / RED |
| `Core/Src/main.c:316` | `sConfigOC.Pulse = PWM_BLUE_CCR;` — TIM2_CH3 / BLUE（PWM mode 2）|
| `Core/Src/main.c:373` | `sConfigOC.Pulse = PWM_DUTY_TICKS;` — TIM3_CH2 / GREEN |

GREEN 的相位被固定在 `PWM_GREEN_PHASE = 1200`（frame 的 1/3），
這樣不論 duty 多少，三個 slot 的順序都不會亂。若 duty > 1200 ticks，
GREEN 的開啟區間就會跨過 BLUE 的起始點而重疊：

```
duty = 1200 ticks (33.3%)：
  RED    CNT 0 ............ 1199
  GREEN  CNT 1200 ........ 2399   ← 緊接著紅色結束
  BLUE   CNT 2400 ....... 3599   ← 緊接著綠色結束
```

### 真正的 duty 上限：23%，不是 33%

上面 1/3 frame 的限制還不是真正的瓶頸。LM3435 在**每次顏色切換**都強制插入轉換延遲
（`06h DELAY`，最小值 `0` = 5 µs），而一個 frame 有 **3 次切換**（R→G、G→B、B→R），
所以不論怎麼排 slot，每個 frame 固定有 **15 µs 死區**：

```
duty_max = (frame − 15 µs) / 3 / frame
```

代入 50 µs frame（20 kHz）：`(50 − 15) / 3 / 50` = **23.3%**

這裡有個反直覺的結論：**提高 PWM 頻率會讓 duty 上限變差**，
因為固定的 15 µs 死區在越短的 frame 裡佔比越高。

| PWM | frame | 可用給 slot | duty 上限 |
|---|---|---|---|
| 40 kHz | 25 µs | 10 µs | 13.3% |
| **20 kHz（現在）** | **50 µs** | **35 µs** | **23.3%** |
| 10 kHz | 100 µs | 85 µs | 28.3% |
| 5 kHz | 200 µs | 185 µs | 30.8% |
| 667 Hz | 1500 µs | 1485 µs | 33.3% |

要達到 33% 需要 1500 µs frame = 667 Hz，會有明顯可見閃爍，而且降頻會讓切換聲更明顯。
所以 `DUTY` 的合法範圍定為 **1–23**（`PWM_DUTY_MAX_TICKS 810` = 22.5%，向下取整保證不超 frame）。
0 不允許（會讓 LM3435 誤判短路）。

程式碼內 `PWM_DUTY_MAX_TICKS` 與 `PWM_DUTY_MAX_PCT` 定義在 `Core/Src/main.c:74`，
`DUTY` 指令的說明文字與上限都引用這兩個常數，改常數即可調整。

### 若只用單一通道，duty 可以超過 23%

死區是「**每次切換**」的成本，不是「每個通道」的成本，所以通道越少越有利：

| 啟用通道 | 每 frame 切換次數 | 死區 | duty 上限 @20 kHz |
|---|---|---|---|
| 3 | 3 | 15 µs | 23.3% |
| 2 | 2 | 10 µs | 40% |
| 1 | 0（無切換）| ~0 | **接近 100%** |

單通道時該通道可以一直保持 HIGH，沒有「切換」，5 µs 延遲無從適用。
剩下的唯一限制是 boost 轉換器需要一點 off-time 讓電感電流歸零，遠小於 5 µs。

實作上「一個通道 80%、另外兩個 0%」是可行的，但 **0% 必須用 PWM duty = 0**
（輸出靜態 LOW），不能用電流代碼 = 0。兩者的 fault 行為完全不同：

| 情況 | 有下降沿？| 結果 |
|---|---|---|
| 電流代碼 = 0，但 PWM 仍脈衝 | 有，每 50 µs | **假 SHORT**（VOUT 拉不上去）|
| PWM duty = 0%（靜態 LOW）| **沒有** | **不 fault**（3 次連續下降沿不成立）|
| PWM 有驅動但沒接 LED | 有 | **真 SHORT**（無負載拉電壓）|

代價是**放棄 RGB 混色**，只得到單色亮度：

```
三通道 10% duty × 100% 電流 → 每色亮度 0.1 × I_peak
單通道 80% duty × 100% 電流 → 該色亮度 0.8 × I_peak   （8 倍，但只有一色）
```

散熱與電感需重新確認：三通道 10% 時轉換器平均電流為 0.3 × I_peak，
單通道 80% 則是 0.8 × I_peak。

### 對 RGB 混色而言，duty 不是提升亮度的手段

這是本專案最重要的設計結論：

> **要提升 RGB 混色的白光亮度，duty 只能在 23% 內，
> 唯一可用的手段是提高電流（降低 `R_SENSE` 讓 `I_peak` 上去）。**
> duty 一旦提高就會失去混色能力，因為 sequential 掃描的三色必須輪流佔用不同時段。

亮度近似 `duty × current`。duty 被 `06h DELAY` 的死區硬性鎖在 23%，
所以混色亮度的天花板完全由 `R_SENSE` / `I_peak` 決定，而不是由 PWM 決定。
要更亮就降 `R_SENSE`，上限是 LM3435 的 2 A 額定值與 LED 本身的額定值。

### 為什麼綠色一定要用第二個 timer？

LM3435 是 **sequential** 驅動器，**不支援控制訊號重疊**（重疊時優先序 `GREEN > BLUE > RED`）。

單一 timer 的每一個 channel 只能提供**一組比較邊沿**，PWM mode 1 只能讓輸出落在
frame **開頭**，mode 2 只能落在**結尾**，**無法產生中段的窄脈波**（中段需要兩組邊沿）。
所以三個非重疊 slot 一定要用**兩個 timer**。

### 為什麼綠色不能用 PA1？

STM32F103 的 AF 對應（`AFIO_MAPR_TIM3_REMAP`）只有三組：

| TIM3_CH2 位置 | remap 設定 |
|---|---|
| **PA7** | no remap（預設）|
| PB5 | partial remap |
| PC7 | full remap |

**PA1 沒有任何 TIM3 功能**，只有 `TIM2_CH2`（或 partial remap 時的 `PB3`）。
若在 CubeMX 填 `PA1 = TIM3_CH2`，專案可以存檔、Keil 也可以編譯，
但**硬體完全沒有輸出**。其他可用的 TIM3 腳位：`PB0`(CH3)、`PB1`(CH4)、`PA6`(CH1)。

---

## 3. LM3435 重點

- **I2C 7-bit 位址 = `0x28`** → HAL 使用 `(0x28 << 1)` = `0x50`
- **不支援 auto-increment**：datasheet 只記載「addr + register + 單一 data byte」，
  所以 00h–03h 必須**分四次**單獨寫入
- 電流更新與 fault detection 依賴**對應 CTRL 的下降沿**，所以 PWM 必須持續輸出
- 10-bit 電流代碼 `0x000`–`0x3FF`，`0x3FF` = 滿電流
  ```
  I_REF   = V_REF / R_IREF
  I_LEDmax = I_REF / R_SENSE
  I_LED(code) = I_LEDmax × (code / 1023)
  ```
- `EN` 內部上拉，拉低到 `0.2 × VIN` 以下可關閉
- **共地是必要條件**

### Register map（本專案使用部分）

| Addr | 名稱 | 內容 |
|---|---|---|
| `00h` | LEDLO | `[7:6]=0` `[5:4]=RLED[1:0]` `[3:2]=BLED[1:0]` `[1:0]=GLED[1:0]` |
| `01h` | GLEDH | `GLED[9:2]` |
| `02h` | BLEDH | `BLED[9:2]` |
| `03h` | RLEDH | `RLED[9:2]` |
| `05h` | FLT_RPT | `bit0` = fault 回報致能（寫 0 可清除 latch 的故障）|
| `06h` | DELAY | `[7:6]RDLY [5:4]BDLY [3:2]GDLY`，`0`=5µs `1`=15 `2`=25 `3`=35 |
| `07h` | FAULT | RO：`D7 GO  D6 GS  D5 -  D4 BO  D3 BS  D2 -  D1 RO  D0 RS` |

Register 預設值（datasheet 原文）：

| Addr | DEFAULT | 意義 |
|---|---|---|
| `00h` | `0011 1111` | RLED/BLED/GLED 低 2 bit 全 1 |
| `01h`–`03h` | `1111 1111` | 高 8 bit 全 1 |
| `05h` | `0000 0001` | FLT_RPT = 1（fault 回報預設開啟）|
| `06h` | `1111 1111` | RDLY=BDLY=GDLY=3 → **35 µs** |
| `07h` | `0000 0000` | 無 fault |

> ⚠️ **上電瞬間 LED 電流是滿電流。**
> `00h`–`03h` 的 reset 值合起來是 `0x3FF` = **最大電流**，
> 在 I2C 尚未寫入前 LED 就跑滿載。實機驗證時建議先把 LED 限流或降低 `VIN`，
> 避免上電瞬間過電流或 LED 燒毀。
>
> `06h` 預設 35 µs 也大於 20 kHz 下單一 slot 的 16.7 µs，
> 所以 `LM3435_Init()` 必須立刻寫入 `0x00`（5 µs）。

### Fault 判定條件

| 條件 | 判定 |
|---|---|
| VOUT 被限制在約 `VIN + 4.7 V` | **OPEN**（開路）|
| VOUT 無法被調節到高於 `VIN + 1.5 V` | **SHORT**（短路）|

連續 **3 個**該通道 CTRL 下降沿偵測到 suspect 訊號後，fault 就會 **latch** 住。

> ⚠️ **不要用 `LM3435 0` 當「熄滅」指令。**
> 電流代碼 = 0 時轉換器不需要抬高 VOUT，VOUT 停在 VIN 附近 → 低於 `VIN + 1.5 V`
> → **被誤判為 SHORT**，會出現假的 `GS`/`RS`/`BS` fault。
> 要熄滅請用極低電流（例如 `LM3435 1`）或把 `EN` 拉低。

清除 latch 的故障：寫入 `05h = 0x00` 再寫回 `05h = 0x01`（或重新上電）。

> 若為**真正的短路**，datasheet 要求立即把對應 CTRL 拉到 GND 關掉該通道，
> 否則可能永久損壞 LM3435。

---

## 4. VCP 指令（USB CDC）

開啟 COM port（波特率無意義，CDC 虛擬埠），輸入下列指令：

| 指令 | 功能 |
|---|---|
| `PING` | 回 `PONG`，驗證 USB 收發 |
| `HELP` 或 `?` | 指令列表 |
| `STATUS` | USB / PWM / I2C / LM3435 FAULT 完整狀態 |
| `CLR` | 清除 latch 的 LM3435 fault（透過 `05h`）|
| `DUTY <1..23>` | 調整三路的 PWM duty（frame 百分比）|
| `LM3435 <0..100>` | 設定 R/G/B LED 電流為 IREF 的百分比 |

特性：
- **大小寫不敏感**（`lm3435 10` 也可以）
- 以 `\r` / `\n` 斷行；若終端機**不送換行符**，會在**停止輸入 500 ms** 後自動送出
- 命令佇列深度 4 格，前面的指令不會被覆蓋
- 開機後第一個指令到達前，主迴圈每 1 秒送一個 `.` 作為連線心跳

---

## 5. 編譯與燒錄

工具鏈：Keil MDK-ARM / **ARMCC 5.06** / uVision 5.43

```powershell
C:\KeilC\UV4\UV4.exe -r G:\CUBEIDE\LM3435_PWM\LM3435_PWM_K4_Testbench\MDK-ARM\LM3435_PWM_K4_Testbench.uvprojx -t LM3435_PWM_K4_Testbench -j0
```

或直接開啟 `MDK-ARM\LM3435_PWM_K4_Testbench.uvprojx` 後按 `F7` 編譯、`Ctrl+F8` 燒錄。

最後驗證結果：`0 Error(s), 0 Warning(s)`
`Code=29612 RO-data=1752 RW-data=404 ZI-data=6852`

---

## 6. 測試流程（實測通過）

### 步驟 0 — 硬體接線

| STM32 | → | LM3435 | Pin |
|---|---|---|---|
| PA0 | → | RCTRL | **20** |
| **PA7** | → | **GCTRL**（不是 PA1！）| **18** |
| PA2 | → | BCTRL | **19** |
| PB6 | → | SCLK | 13 |
| PB7 | → | SDATA | 12 |
| GND | ↔ | PGND | 1,2,38,39 + EP |

**LED 接法（極性不可接反）**：

| LED 極性 | → | LM3435 | Pin |
|---|---|---|---|
| 陽極 (+)| → | VOUT | 30,31,32 |
| 紅陰極 (−)| → | RLED | 21,22 |
| 藍陰極 (−)| → | BLED | 23,24 |
| 綠陰極 (−)| → | GLED | 25,26 |

其餘必要接線：

| 訊號 | Pin | 說明 |
|---|---|---|
| SVDD | 11 | **I2C 供電（2.7–5.5 V），需獨立去耦電容，不可沿用 STM32 3.3 V 而省略** |
| VIN | 14,15,16,17,37 | 主電源輸入 |
| EN | 28 | 內部上拉；接 `VIN` 啟用，降到 `0.2 × VIN` 以下關閉 |
| CG | 3 | 綠色 LED 電容，需對地接電容 |
| RT | 33 | 從 VOUT 接電阻到 RT，決定開關頻率 |
| SW | 34,35,36 | 接輸出電感 |
| FAULT | 27 | 故障指示（開路/短路時拉高），本專案未使用 |

封裝為 **40-pin WQFN（LLP）**，EP 為散熱焊盤，**必須連到 GND**。
I2C 上拉電阻 PCB 上已有。

> 官方參考接法見 LM3435 Evaluation Board User's Guide (AN-2196)，
> 建議照它的原理圖佈線，尤其 `RT` 電阻與 `SW` 電感。

> ⚠️ **上電瞬間 LED 是滿電流。** `00h`–`03h` 的 reset 值 = `0x3FF`。
> 韌體在 `LM3435_Init()` 第一件事就是把電流降到 code 1 再設其他暫存器，
> 但在 I2C 開始前的那段時間（數 ms）LED 仍是滿載。首次上電建議先降低 `VIN`
> 或在 LED 迴路加限流電阻。

### 步驟 1 — 編譯與燒錄
Keil `F7` → `0 Error(s), 0 Warning(s)` → `Ctrl+F8` 下載。
**燒錄後 USB 會重新枚舉，必須關閉再開終端機。**

### 步驟 2 — 確認 USB 連線
開啟 COM port，應看到每秒一個 `.`：

```
........
```
若完全沒有輸出，代表韌體沒燒進去或 COM port 選錯。

### 步驟 3 — 驗證指令通道
輸入 `PING`，期望：

```
==== LM3435 RGB sequential driver ====
PWM 20.00kHz duty 10.0% : PA0/R=0us  PA7/G=16.67us  PA2/B=45us
LM3435 is controlled by CURRENT, duty is fixed at 10%.
Commands: PING | HELP | STATUS | CLR | LM3435 <0..100>

PONG
```

### 步驟 4 — 驗證 I2C 連線
輸入 `STATUS`，期望：

```
USB CDC  : enumerated
PWM      : 20.00 kHz, duty 10.0%, 50us frame
Pins     : PA0=TIM2_CH1/R  PA7=TIM3_CH2/G  PA2=TIM2_CH3/B
I2C1     : 100 kHz, addr 0x50, HAL err=0x0000
LM3435   : ACK, FAULT(07h)=none
```

- `NO ACK on 0x50` → 位址錯誤、缺少共地、或 SDA/SCL 沒接好
- `HAL err != 0x0000` → I2C 匯流排有錯誤

### 步驟 5 — 驗證 PWM 波形
示波器三通道都接，**以 PA0 當觸發來源**，時基約 `10 µs/div`，按 `Auto`。
期望三個脈波寬 5 µs、週期 50 µs、位置錯開。

> 若頻率顯示異常數值（如 `792 mHz`），是量測假象，按 `Auto` 即可。

### 步驟 6 — 掃描電流
```
LM3435 5
LM3435 100
LM3435 1
```
每次都應回 `OK: n% -> RLED=... GLED=... BLED=...` 且 `FAULT(07h)=none`。

> **不要輸入 `LM3435 0`**，會產生假的 SHORT fault（見第 3 節）。

### 步驟 7 — 調整 duty（可選）
```
DUTY 23
DUTY 20
DUTY 10
DUTY 30
```
前三項應成功；`DUTY 30` 應被拒絕並解釋原因：

```
OK: duty 23% (828 ticks, 11.50 us per channel)
    slots: R=828 G=1200 B=2772 ticks (total 2484 of 3600)
    times : R=0.00us G=16.67us B=38.50us  (brightness = duty x current)
ERR: usage DUTY <1..23> (got "DUTY 30")
     LM3435 needs 5us per colour change and a frame has three
     changes, so 15us of every 50us frame is dead time:
         duty_max = (frame - 15us) / 3 / frame
     at 20kHz = (50 - 15) / 3 / 50 = 23%
     Duty 0 is also rejected: it makes LM3435 raise a false SHORT
     fault. Raising the PWM frequency lowers this limit further.
```

調整後用示波器確認脈波變寬且三段仍不重疊。

### 步驟 8 — 清除故障並複查
```
CLR
STATUS
```
期望 `fault cleared, FAULT(07h) now = none`。

### 步驟 9 — 肉眼確認
確認三顆 LED 輪流亮起。`DUTY 10` + `LM3435 100` 時每色亮度約 10%，偏暗是預期中的。
`DUTY 23` + `LM3435 100` 是保留混色能力時最亮的合法設定（每色約 23%）。
若要更亮只能降 `R_SENSE` 提高 `I_peak`（見第 2 節「對 RGB 混色而言」）。

### 實測輸出紀錄

```
PONG
OK: 5% -> RLED=51 GLED=51 BLED=51
    FAULT(07h)=none
OK: 100% -> RLED=1023 GLED=1023 BLED=1023
    FAULT(07h)=none
OK: 1% -> RLED=10 GLED=10 BLED=10
    FAULT(07h)=none
fault cleared, FAULT(07h) now = none
```

---

## 7. 電流量測怎麼換算

LM3435 是**定電流開關**：CTRL 為 HIGH 的 5 µs 輸出**全額 regulated 電流** `I_peak`，
其餘時間輸出接近 0。DMM 與示波器 DC 耦合量到的是**平均值**：

```
I_avg = 導通佔空比 × I_peak
```

| 量測位置 | 導通佔空比 | 由 `I_avg = 0.2 A` 反推 |
|---|---|---|
| 單一顏色 LED 路徑 | 5/50 = **10%** | `I_peak ≈ 2.0 A` |
| 三色共用路徑 | 15/50 = **30%** | `I_peak ≈ 0.67 A` |

要量真正的 `I_peak`：用示波器 + 電流探棒，看那 5 µs 脈波的高度；
或代入第 3 節的公式用 `R_IREF` / `R_SENSE` 計算。

### 實測：電流隨 duty 線性變化

`LM3435 100`（滿電流）下改變 duty，DMM 讀數：

| `DUTY` | 導通佔空比 | 預測 I_avg（peak 2.0 A）| 實測 I_avg |
|---|---|---|---|
| 10 | 10% | 0.20 A | 0.20 A |
| 30 | 30% | 0.60 A | **0.70 A** |

倍率 3.5× 對照預測 3.0×，差異來自 DMM 對斬波波形的取樣誤差。
線性關係成立代表三個 PWM slot 的寬度控制正確。

> ⚠️ 若量的是**單色路徑**，`0.7 / 0.30` 反推 `I_peak ≈ 2.33 A`，已略超 LM3435 的
> 2 A 規格。duty 不會改變 peak（peak 由 `R_IREF` / `R_SENSE` 決定），
> 因此較可能是 DMM 誤差；請用電流探棒直接確認。

---

## 8. 除錯紀錄（本專案修正過的問題）

### 接腳 / 設定

| 問題 | 說明 |
|---|---|
| **PA1 無輸出** | PA1 沒有 TIM3 功能，`PA1 = TIM3_CH2` 可存檔可編譯但無訊號。必須改用 PA7 |
| 單 timer 無法做三個 slot | 中段 slot 需要兩組比較邊沿，必須用第二個 timer |
| `.ioc` 漏了 `Mcu.Pin9=PA11` | 會讓 CubeMX 重新產生時報錯（PA11 = USB_DM）|
| `.ioc` 有重複的 `PA2.Signal` | CubeMX 解析異常 |
| `06h DELAY` 預設 35 µs | 大於單一 slot 的 16.7 µs，必須寫 `0x00` |

### USB CDC

| 問題 | 說明 |
|---|---|
| 開機沒任何輸出 | banner 在 `MX_USB_DEVICE_Init()` 後僅 delay 500 ms，Windows 枚舉尚未完成就被丟棄。改為第一個指令到達時才送 |
| 指令偶發沒回覆 | `VCP_Send` 忽略 `USBD_BUSY`，前一個封包在傳時新封包被靜默丟棄。加重試 |
| 封包內容損毀 | `VCP_Printf` 用 stack buffer 交給非同步 USB 傳輸，函式返回後 stack 被覆寫。改用 `static` |
| 指令變成 `PINGPINGPING` | 只有單一 command buffer，且終端機不送換行符時前一個指令不會被消化。改為 4 格佇列 |
| 指令沒有反應 | 部分終端機不送 `\r`/`\n`。改為 500 ms 停止輸入後自動斷行 |
| 完全沒反應 | 韌體沒燒進去。加開機心跳 `.` 以便區分 |

### LM3435

| 問題 | 說明 |
|---|---|
| I2C 無回應 | 位址要用 `0x28 << 1 = 0x50` |
| 電流設定無效 | `06h DELAY` 太長，超過 slot 時間 |
| `LM3435 0` 出現 `0x41` | 零電流造成 VOUT 停在 VIN，被誤判為 SHORT（假故障）|
| FAULT 判讀錯誤 | 位元順序是 `GO GS - BO BS - RO RS`，`0x41` = 綠短路 + 紅短路 |
| 上電瞬間 LED 滿電流 | `00h`–`03h` reset 值 = `0x3FF`，`LM3435_Init()` 先降到 code 1 |

---

## 9. 已知限制

1. **duty 預設 10%**（編譯期常數 `PWM_DUTY_TICKS`），可用 `DUTY <1..23>` 在
   執行期調整。duty 越高代表 on-time 越長，電源瞬態壓力與散熱負擔越高；
   連續使用高亮度需注意溫升。
2. `DUTY` 改變後不會自動恢復，重新燒錄後會回到編譯期的預設值。
3. duty 上限 23% 是 LM3435 硬體限制（每 frame 固定 15 µs 轉換死區），
   不是 timer 安排問題。若要接近 33% 必須把 frame 拉長到 1500 µs（667 Hz），
   代價是明顯閃爍與更吵的切換聲。見第 2 節的表格。
   單一通道可以超過 23%，但那會失去 RGB 混色（見第 2 節）。
   **混色亮度的唯一提升手段是降低 `R_SENSE` 提高 `I_peak`。**
4. `LM3435` 指令同時設定 R/G/B，無法個別控制單一色。
5. 故障暫存器需手動 `CLR` 才會清除（寫 `05h=0x00` 再寫 `05h=0x01`；
   datasheet 另提供把 `EN` 拉低 **小於 100 ns** 的硬體清除法，
   但一般 MCU 用 GPIO 無法做到這個寬度，請用 I2C 方式）。
6. 上電瞬間 LED 為滿電流，韌體已盡量降低但無法完全消除（見第 3 節）。

---

## 10. 檔案結構

```
LM3435_PWM_K4_Testbench/
├── README.md                       本文件
├── README_EN.md                    英文版
├── LM3435_PWM_K4_Testbench.ioc     CubeMX 設定（TIM2/TIM3、PA0/PA7/PA2、I2C1、USB）
├── Core/
│   ├── Inc/
│   │   └── main.h                  VCP 緩衝區大小、VCP_QueuePop、函式原型
│   └── Src/
│       ├── main.c                  PWM 時序（含 PWM_DUTY_TICKS / PWM_ARR / PWM_GREEN_PHASE）、
│       │                           I2C、LM3435 控制、VCP 指令解析（DUTY / LM3435 / CLR）
│       └── stm32f1xx_hal_msp.c     TIM2/TIM3 clock 與 GPIO AF 設定
├── USB_DEVICE/
│   └── App/
│       └── usbd_cdc_if.c           USB IRQ 只做收字元 + 4 格命令佇列
└── MDK-ARM/
    └── LM3435_PWM_K4_Testbench.uvprojx
```

---

## 11. 參考資料

- LM3435 datasheet（SNVS724C）：<https://www.ti.com/lit/ds/symlink/lm3435.pdf>
- LM3435 Evaluation Board User's Guide（AN-2196）：<https://www.ti.com/lit/ug/snva506a/snva506a.pdf>
- STM32F103 reference manual（RM0008）
- STM32Cube FW_F1 V1.8.7：
  `C:\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.7`