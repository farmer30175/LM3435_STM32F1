# LM3435 RGB LED Driver Test Board

STM32F103C8T6 + USB CDC control of the TI **LM3435** (Sequential RGB LED Driver).
LED current is adjusted over I2C, and a three-channel non-overlapping PWM scheme
generates the sequential scan signals.

---

## 1. System Architecture

```
STM32F103C8T6                        LM3435
┌────────────────────┐
│ PA0  TIM2_CH1 ────────────► RCTRL  │
│ PA7  TIM3_CH2 ────────────► GCTRL  │──► LED (R/G/B)
│ PA2  TIM2_CH3 ────────────► BCTRL  │
│ PB6  I2C1_SCL ─────────────► SCLK  │
│ PB7  I2C1_SDA ─────────────► SDATA │
│ USB  CDC    ──► COM Port command  │
└────────────────────┘              │
                          SVDD / EN / GND (common ground required)
```

**Brightness formula**: `brightness = PWM duty × LED current`.
The duty in this project is fixed at 10%, so `LM3435 100` actually yields only
10% brightness.

---

## 2. PWM Timing Design

| Item | Value |
|---|---|
| System clock | HSE 8 MHz × 9 (PLL) = 72 MHz |
| APB1 timer clock | 72 MHz |
| Prescaler (PSC) | 0 |
| Period (ARR) | 3599 → `72e6 / 1 / 3600` = **20.000 kHz** |
| Frame period | 50 µs |
| 1 tick | 13.889 ns |
| Duty | `360 / 3600` = **10%** = 5 µs |

Three slots inside the 50 µs frame, **mutually non-overlapping**:

| Time | Pin | Timer | Mode | CCR | CNT range | Channel |
|---|---|---|---|---|---|---|
| 0 – 5 µs | PA0 | TIM2_CH1 | PWM mode 1 | 360 | 0 – 359 | RED |
| 16.67 – 21.67 µs | PA7 | TIM3_CH2 | PWM mode 1 | 360 | 1200 – 1559 | GREEN |
| 45 – 50 µs | PA2 | TIM2_CH3 | PWM mode 2 | 3240 | 3240 – 3599 | BLUE |

- The GREEN counter is preloaded with `1200` (`PWM_GREEN_PHASE`) before enabling the
  timer, so its pulse lands in the middle of the frame.
- BLUE uses PWM **mode 2** so its pulse sits at the end of the frame.
- Total on-time = 3 × 5 µs = 15 µs (30% of the frame).

### Where the duty lives in the code

The CubeMX "Pulse" field is `sConfigOC.Pulse` in C:

| File:line | Content |
|---|---|
| `Core/Src/main.c:63` | `#define PWM_DUTY_TICKS 360` ← **the default duty is here** |
| `Core/Src/main.c:305` | `sConfigOC.Pulse = PWM_DUTY_TICKS;` — TIM2_CH1 / RED |
| `Core/Src/main.c:316` | `sConfigOC.Pulse = PWM_BLUE_CCR;` — TIM2_CH3 / BLUE (PWM mode 2) |
| `Core/Src/main.c:373` | `sConfigOC.Pulse = PWM_DUTY_TICKS;` — TIM3_CH2 / GREEN |

GREEN is pinned to `PWM_GREEN_PHASE = 1200` (one third of the frame), which keeps
the three slots in order for any duty. Above 1200 ticks the GREEN on-window would
run past the start of BLUE and the pulses would overlap:

```
duty = 1200 ticks (33.3%):
  RED    CNT 0 ............ 1199
  GREEN  CNT 1200 ........ 2399   ← starts exactly where RED ends
  BLUE   CNT 2400 ....... 3599   ← starts exactly where GREEN ends
```

### The real duty ceiling is 23%, not 33%

The one-third-of-frame limit above is not the actual bottleneck. The LM3435
forces a transition delay at **every colour change** (`06h DELAY`, minimum value
`0` = 5 µs), and one frame contains **3 changes** (R→G, G→B, B→R). So every frame
reserves a fixed **15 µs of dead time**, however the slots are packed:

```
duty_max = (frame − 15 µs) / 3 / frame
```

With a 50 µs frame (20 kHz): `(50 − 15) / 3 / 50` = **23.3%**

Here is the counter-intuitive part: **raising the PWM frequency makes the ceiling
worse**, because the fixed 15 µs of dead time takes up a larger fraction of a
shorter frame.

| PWM | frame | available for slots | duty ceiling |
|---|---|---|---|
| 40 kHz | 25 µs | 10 µs | 13.3% |
| **20 kHz (current)** | **50 µs** | **35 µs** | **23.3%** |
| 10 kHz | 100 µs | 85 µs | 28.3% |
| 5 kHz | 200 µs | 185 µs | 30.8% |
| 667 Hz | 1500 µs | 1485 µs | 33.3% |

Reaching 33% needs a 1500 µs frame = 667 Hz, which flickers visibly, and lowering
the frequency also makes the switching noise more noticeable. `DUTY` therefore
accepts **1–23** (`PWM_DUTY_MAX_TICKS 810` = 22.5%, rounded down so the total can
never exceed the frame). 0 is rejected because it makes the LM3435 report a false
SHORT fault.

`PWM_DUTY_MAX_TICKS` and `PWM_DUTY_MAX_PCT` are defined at `Core/Src/main.c:74`.
The `DUTY` help text and the limit check both reference those constants, so
changing them is all that is needed to adjust the ceiling.

### With a single channel, duty can exceed 23%

The dead time costs a fixed amount **per transition**, not per channel, so fewer
active channels means a better deal:

| Active channels | Transitions per frame | Dead time | duty ceiling @20 kHz |
|---|---|---|---|
| 3 | 3 | 15 µs | 23.3% |
| 2 | 2 | 10 µs | 40% |
| 1 | 0 (no transition) | ~0 | **close to 100%** |

With one channel it can simply stay HIGH; there is no transition, so the 5 µs
delay does not apply. The only remaining limit is the off-time the boost converter
needs to decay the inductor current, which is far shorter than 5 µs.

Implementing "one channel at 80%, the other two at 0%" is feasible, but the 0%
**must be PWM duty = 0** (output statically LOW), not current code = 0. The two
have completely different fault behaviour:

| Situation | Falling edges? | Result |
|---|---|---|
| Current code = 0 but PWM still pulsing | yes, every 50 µs | **false SHORT** (VOUT cannot be pulled up) |
| PWM duty = 0% (statically LOW) | **no** | **no fault** (3 consecutive edges never occur) |
| PWM driving but no LED connected | yes | **real SHORT** (no load pulls the voltage) |

The cost is **giving up RGB colour mixing** in exchange for single-colour brightness:

```
three channels at 10% duty × 100% current → 0.1 × I_peak per colour
one channel at 80% duty × 100% current → 0.8 × I_peak for that colour (8x, but one colour only)
```

Thermal and inductor sizing must be rechecked: at three channels / 10% duty the
converter carries 0.3 × I_peak on average, whereas a single channel at 80% carries
0.8 × I_peak.

### For RGB mixing, duty is not a brightness lever

This is the single most important design conclusion in this project:

> **To raise the brightness of RGB-mixed white light, duty stays within 23% and
> the only available lever is current (lower `R_SENSE` to raise `I_peak`).**
> Raising duty costs you colour mixing, because the three colours of a sequential
> scan must each occupy their own time slot.

Brightness is approximately `duty × current`. Duty is hard-limited to 23% by the
`06h DELAY` dead time, so the ceiling for mixed brightness is set entirely by
`R_SENSE` / `I_peak`, not by PWM. To get brighter, lower `R_SENSE`, bounded by the
LM3435 2 A rating and the LED's own rating.

### Why must GREEN use a second timer?

The LM3435 is a **sequential** driver and **does not support overlapping control
signals** (priority if they overlap: `GREEN > BLUE > RED`).

Each channel of a single timer provides only **one compare edge**: PWM mode 1 can
only place the output at the **start** of the frame and mode 2 only at the **end**.
A **narrow pulse in the middle** requires *two* edges, so three non-overlapping
slots necessarily need **two timers**.

### Why can't GREEN use PA1?

On STM32F103 the AF options for `AFIO_MAPR_TIM3_REMAP` are only three:

| TIM3_CH2 pin | remap setting |
|---|---|
| **PA7** | no remap (default) |
| PB5 | partial remap |
| PC7 | full remap |

**PA1 has no TIM3 function at all** — only `TIM2_CH2` (or `PB3` under partial
remap). Assigning `PA1 = TIM3_CH2` in CubeMX saves and compiles without error,
but produces **no hardware output whatsoever**. Other usable TIM3 pins:
`PB0` (CH3), `PB1` (CH4), `PA6` (CH1).

---

## 3. LM3435 Essentials

- **I2C 7-bit address = `0x28`** → HAL uses `(0x28 << 1)` = `0x50`
- **No auto-increment**: the datasheet only documents "addr + register + single
  data byte", so 00h–03h must be written as **four separate transactions**
- Current updates and fault detection depend on the **falling edge** of the
  corresponding CTRL signal, so the PWM must keep running
- 10-bit current code `0x000`–`0x3FF`, where `0x3FF` = full current
  ```
  I_REF    = V_REF / R_IREF
  I_LEDmax = I_REF / R_SENSE
  I_LED(code) = I_LEDmax × (code / 1023)
  ```
- `EN` is internally pulled up; pulling it below `0.2 × VIN` disables the device
- **A common ground connection is mandatory**

### Register map (registers used by this project)

| Addr | Name | Content |
|---|---|---|
| `00h` | LEDLO | `[7:6]=0` `[5:4]=RLED[1:0]` `[3:2]=BLED[1:0]` `[1:0]=GLED[1:0]` |
| `01h` | GLEDH | `GLED[9:2]` |
| `02h` | BLEDH | `BLED[9:2]` |
| `03h` | RLEDH | `RLED[9:2]` |
| `05h` | FLT_RPT | `bit0` = fault reporting enable (writing 0 clears the latched fault) |
| `06h` | DELAY | `[7:6]RDLY [5:4]BDLY [3:2]GDLY`, `0`=5µs `1`=15 `2`=25 `3`=35 |
| `07h` | FAULT | RO: `D7 GO  D6 GS  D5 -  D4 BO  D3 BS  D2 -  D1 RO  D0 RS` |

Register default values (from the datasheet):

| Addr | DEFAULT | Meaning |
|---|---|---|
| `00h` | `0011 1111` | RLED/BLED/GLED low 2 bits all 1 |
| `01h`–`03h` | `1111 1111` | High 8 bits all 1 |
| `05h` | `0000 0001` | FLT_RPT = 1 (fault reporting enabled by default) |
| `06h` | `1111 1111` | RDLY=BDLY=GDLY=3 → **35 µs** |
| `07h` | `0000 0000` | no fault |

> ⚠️ **LED current is at maximum immediately after power-up.**
> The reset values of `00h`–`03h` combine to `0x3FF` = **full current**, so the
> LEDs run at full load before any I2C write happens. During bring-up, limit the
> LED current or lower `VIN` first to avoid an overcurrent or a burnt LED.
>
> The `06h` default of 35 µs is also longer than the 16.7 µs of a single 20 kHz
> slot, so `LM3435_Init()` must immediately write `0x00` (5 µs).

### Fault detection conditions

| Condition | Verdict |
|---|---|
| VOUT clamped to about `VIN + 4.7 V` | **OPEN** |
| VOUT cannot be regulated above `VIN + 1.5 V` | **SHORT** |

The fault is **latched** after the suspect signal is seen at **3 consecutive**
falling edges of that channel's CTRL signal.

> ⚠️ **Never use `LM3435 0` as an "off" command.**
> With a current code of 0 the converter does not need to raise VOUT, so VOUT
> stays at VIN → below `VIN + 1.5 V` → **falsely reported as SHORT**, producing
> bogus `GS`/`RS`/`BS` flags. To switch off, use a very low current (e.g.
> `LM3435 1`) or pull `EN` low.

To clear a latched fault: write `05h = 0x00` then write `05h = 0x01` again
(or power-cycle).

> For a **real** short circuit the datasheet requires pulling the corresponding
> CTRL pin to GND immediately to disable that channel, otherwise the LM3435 may
> be permanently damaged.

---

## 4. VCP Commands (USB CDC)

Open the COM port (baud rate is irrelevant for a virtual CDC port) and type:

| Command | Purpose |
|---|---|
| `PING` | Replies `PONG`, verifies the USB link |
| `HELP` or `?` | Command list |
| `STATUS` | Full USB / PWM / I2C / LM3435 FAULT state |
| `CLR` | Clear the latched LM3435 fault (via `05h`) |
| `DUTY <1..23>` | Adjust the PWM duty of all three channels (percent of one frame) |
| `LM3435 <0..100>` | Set R/G/B LED current as a percentage of IREF |

Behaviour:
- **Case-insensitive** (`lm3435 10` works too)
- Lines are terminated by `\r` / `\n`; if the terminal **sends no line ending**,
  the command is sent automatically after **500 ms of typing silence**
- The command queue holds 4 entries, so earlier commands are never overwritten
- Until the first command arrives, the main loop emits one `.` per second as a
  link heartbeat

---

## 5. Build and Flash

Toolchain: Keil MDK-ARM / **ARMCC 5.06** / uVision 5.43

```powershell
C:\KeilC\UV4\UV4.exe -r G:\CUBEIDE\LM3435_PWM\LM3435_PWM_K4_Testbench\MDK-ARM\LM3435_PWM_K4_Testbench.uvprojx -t LM3435_PWM_K4_Testbench -j0
```

Or open `MDK-ARM\LM3435_PWM_K4_Testbench.uvprojx` and press `F7` to build,
`Ctrl+F8` to download.

Last verified result: `0 Error(s), 0 Warning(s)`
`Code=29612 RO-data=1752 RW-data=404 ZI-data=6852`

---

## 6. Test Procedure (verified on hardware)

### Step 0 — Wiring

| STM32 | → | LM3435 | Pin |
|---|---|---|---|
| PA0 | → | RCTRL | **20** |
| **PA7** | → | **GCTRL** (not PA1!) | **18** |
| PA2 | → | BCTRL | **19** |
| PB6 | → | SCLK | 13 |
| PB7 | → | SDATA | 12 |
| GND | ↔ | PGND | 1,2,38,39 + EP |

**LED connection (polarity matters)**:

| LED polarity | → | LM3435 | Pin |
|---|---|---|---|
| Anode (+) | → | VOUT | 30,31,32 |
| Red cathode (−) | → | RLED | 21,22 |
| Blue cathode (−) | → | BLED | 23,24 |
| Green cathode (−) | → | GLED | 25,26 |

Other mandatory connections:

| Signal | Pin | Description |
|---|---|---|
| SVDD | 11 | **I2C supply (2.7–5.5 V), needs its own decoupling** |
| VIN | 14,15,16,17,37 | Main supply input |
| EN | 28 | Internally pulled up; tie to `VIN` to enable, below `0.2 × VIN` to disable |
| CG | 3 | Green LED capacitor, connect a capacitor to ground |
| RT | 33 | Resistor from VOUT to RT sets the switching frequency |
| SW | 34,35,36 | Connect the output inductor |
| FAULT | 27 | Fault indicator (pulls high on open/short), unused in this project |

The package is a **40-pin WQFN (LLP)**; the exposed pad (EP) is the thermal pad and
**must be connected to GND**. I2C pull-up resistors are already present on the PCB.

> For the official reference layout, see the LM3435 Evaluation Board User's
> Guide (AN-2196), especially the `RT` resistor and `SW` inductor.

> ⚠️ **LED current is at maximum right after power-up.** The reset value of
> `00h`–`03h` is `0x3FF`. The firmware drops the current to code 1 as the very
> first thing in `LM3435_Init()`, but for the few milliseconds before I2C starts
> the LEDs are still at full load. For the first power-up, lower `VIN` or add a
> series current-limiting resistor.

### Step 1 — Build and flash
Keil `F7` → `0 Error(s), 0 Warning(s)` → `Ctrl+F8` to download.
**Flashing re-enumerates USB, so the terminal must be closed and reopened.**

### Step 2 — Confirm the USB link
Open the COM port; you should see one `.` per second:

```
........
```
If there is no output at all, either the firmware was not flashed or the wrong
COM port is open.

### Step 3 — Verify the command link
Type `PING`, expect:

```
==== LM3435 RGB sequential driver ====
PWM 20.00kHz duty 10.0% : PA0/R=0us  PA7/G=16.67us  PA2/B=45us
LM3435 is controlled by CURRENT, duty is fixed at 10%.
Commands: PING | HELP | STATUS | CLR | LM3435 <0..100>

PONG
```

### Step 4 — Verify the I2C link
Type `STATUS`, expect:

```
USB CDC  : enumerated
PWM      : 20.00 kHz, duty 10.0%, 50us frame
Pins     : PA0=TIM2_CH1/R  PA7=TIM3_CH2/G  PA2=TIM2_CH3/B
I2C1     : 100 kHz, addr 0x50, HAL err=0x0000
LM3435   : ACK, FAULT(07h)=none
```

- `NO ACK on 0x50` → wrong address, missing common ground, or SDA/SCL not connected
- `HAL err != 0x0000` → the I2C bus reported an error

### Step 5 — Verify the PWM waveforms
Connect all three scope channels and **trigger on PA0**, timebase around
`10 µs/div`, then press `Auto`.

Expect three 5 µs wide pulses of 50 µs period, offset in time.

> If the frequency readout looks implausible (e.g. `792 mHz`), that is a
> measurement artefact — press `Auto`.

### Step 6 — Sweep the current
```
LM3435 5
LM3435 100
LM3435 1
```
Each should reply `OK: n% -> RLED=... GLED=... BLED=...` with `FAULT(07h)=none`.

> **Do not enter `LM3435 0`** — it produces a false SHORT fault (see section 3).

### Step 7 — Adjust the duty (optional)
```
DUTY 23
DUTY 20
DUTY 10
DUTY 30
```
The first three should succeed; `DUTY 30` must be rejected with an explanation:

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

Confirm on the scope that the pulses widened and still do not overlap.

### Step 8 — Clear the fault and re-check
```
CLR
STATUS
```
Expect `fault cleared, FAULT(07h) now = none`.

### Step 9 — Visual confirmation
Confirm the three LEDs light up in rotation. At `LM3435 100` with `DUTY 10`
each colour is about 10% brightness, which is expected to be dim.
`DUTY 23` plus `LM3435 100` is the brightest legal setting that keeps colour
mixing (about 23% per colour). To go brighter, lower `R_SENSE` to raise `I_peak`
(see "For RGB mixing, duty is not a brightness lever" in section 2).

### Measured session log

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

## 7. Interpreting Current Measurements

The LM3435 is a **constant-current switch**: while CTRL is HIGH for those 5 µs it
sources the full regulated current `I_peak`; the rest of the time it sources
almost nothing. A DMM (and the scope in DC coupling) measures the **average**:

```
I_avg = conduction duty × I_peak
```

| Measurement point | Conduction duty | Back-calculated from `I_avg = 0.2 A` |
|---|---|---|
| Single-colour LED path | 5/50 = **10%** | `I_peak ≈ 2.0 A` |
| Shared three-colour path | 15/50 = **30%** | `I_peak ≈ 0.67 A` |

To measure the real `I_peak`: use a scope with a current probe and read the
height of the 5 µs pulse, or plug your `R_IREF` / `R_SENSE` values into the
formula in section 3.

### Measured: current scales linearly with duty

DMM readings at `LM3435 100` (full current) while varying the duty:

| `DUTY` | Conduction duty | Predicted I_avg (peak 2.0 A) | Measured I_avg |
|---|---|---|---|
| 10 | 10% | 0.20 A | 0.20 A |
| 30 | 30% | 0.60 A | **0.70 A** |

The 3.5× measured ratio against a 3.0× predicted ratio is within DMM sampling
error on a chopped waveform. The linear relationship confirms that the width
of the three PWM slots is controlled correctly.

> ⚠️ If you are measuring the **single-colour path**, `0.7 / 0.30` back-calculates
> `I_peak ≈ 2.33 A`, slightly above the LM3435 2 A specification. Duty does not
> change the peak (that is set by `R_IREF` / `R_SENSE`), so this is most likely
> DMM error; confirm with a current probe.

---

## 8. Troubleshooting History (issues fixed in this project)

### Pins / configuration

| Problem | Explanation |
|---|---|
| **No output on PA1** | PA1 has no TIM3 function. `PA1 = TIM3_CH2` saves and compiles but emits nothing. Must use PA7 |
| A single timer cannot make three slots | A middle slot needs two compare edges, so a second timer is mandatory |
| `.ioc` was missing `Mcu.Pin9=PA11` | Causes a CubeMX regeneration error (PA11 = USB_DM) |
| `.ioc` had a duplicated `PA2.Signal` | Confuses CubeMX parsing |
| `06h DELAY` defaults to 35 µs | Longer than a 16.7 µs slot, so `0x00` must be written |

### USB CDC

| Problem | Explanation |
|---|---|
| No output at boot | The banner was generated only 500 ms after `MX_USB_DEVICE_Init()`, long before Windows finished enumerating, so it was dropped. Now sent when the first command arrives |
| Commands occasionally unanswered | `VCP_Send` ignored `USBD_BUSY` and silently dropped packets sent while the previous one was still in flight. Now retries |
| Packet contents corrupted | `VCP_Printf` handed a **stack** buffer to asynchronous USB; the stack was overwritten after the function returned. Changed to `static` |
| Commands merged into `PINGPINGPING` | Only a single command buffer existed, and without a line terminator the previous command was never consumed. Now a 4-entry queue |
| Commands with no reaction at all | Some terminals send no `\r`/`\n`. Now a command is dispatched after 500 ms of typing silence |
| Complete silence | The firmware was never flashed. Added a `.` heartbeat to distinguish the two |

### LM3435

| Problem | Explanation |
|---|---|
| No I2C response | The address must be `0x28 << 1 = 0x50` |
| Current setting had no effect | `06h DELAY` was longer than the slot time |
| `LM3435 0` produced `0x41` | Zero current leaves VOUT at VIN, which is falsely reported as SHORT |
| FAULT misread | The bit order is `GO GS - BO BS - RO RS`; `0x41` = green short + red short |
| LEDs at full current on power-up | The reset value of `00h`–`03h` is `0x3FF`; `LM3435_Init()` drops to code 1 first |

---

## 9. Known Limitations

1. **Duty defaults to 10%** as the compile-time constant `PWM_DUTY_TICKS`, and can
   be changed at runtime with `DUTY <1..23>`. A higher duty means a longer on-time,
   which increases supply transient stress and thermal load; watch the temperature
   when running at high brightness for extended periods.
2. A `DUTY` change does not survive a reflash — the compile-time default is restored.
3. The 23% ceiling is an LM3435 hardware limit (a fixed 15 µs of transition dead
   time per frame), not a timer arrangement issue. Getting near 33% would require
   stretching the frame to 1500 µs (667 Hz), at the cost of visible flicker and
   louder switching noise. See the table in section 2.
   A single channel can exceed 23%, but that gives up RGB colour mixing (see
   section 2). **The only lever for mixed brightness is lowering `R_SENSE` to
   raise `I_peak`.**
4. The `LM3435` command sets R, G and B together; individual channels cannot be
   controlled separately.
5. The fault register must be cleared manually with `CLR` (write `05h=0x00` then
   `05h=0x01`). The datasheet also offers a hardware clear by pulling `EN` low for
   **less than 100 ns**, but a normal MCU GPIO cannot achieve that pulse width, so
   use the I2C method.
6. LEDs are at full current right after power-up; the firmware minimises this but
   cannot eliminate it (see section 3).

---

## 10. File Structure

```
LM3435_PWM_K4_Testbench/
├── README.md                       中文版
├── README_EN.md                    This document (English)
├── LM3435_PWM_K4_Testbench.ioc     CubeMX config (TIM2/TIM3, PA0/PA7/PA2, I2C1, USB)
├── Core/
│   ├── Inc/
│   │   └── main.h                  VCP buffer size, VCP_QueuePop, prototypes
│   └── Src/
│       ├── main.c                  PWM timing (PWM_DUTY_TICKS / PWM_ARR / PWM_GREEN_PHASE),
│       │                           I2C, LM3435 control, VCP parser (DUTY / LM3435 / CLR)
│       └── stm32f1xx_hal_msp.c     TIM2/TIM3 clock and GPIO AF setup
├── USB_DEVICE/
│   └── App/
│       └── usbd_cdc_if.c           USB IRQ only collects chars + 4-entry queue
└── MDK-ARM/
    └── LM3435_PWM_K4_Testbench.uvprojx
```

---

## 11. References

- LM3435 datasheet (SNVS724C): <https://www.ti.com/lit/ds/symlink/lm3435.pdf>
- LM3435 Evaluation Board User's Guide (AN-2196): <https://www.ti.com/lit/ug/snva506a/snva506a.pdf>
- STM32F103 reference manual (RM0008)
- STM32Cube FW_F1 V1.8.7:
  `C:\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.7`