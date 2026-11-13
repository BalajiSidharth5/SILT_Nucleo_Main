# Winch + Current Sensor Driver (SILT Boat)

Drives two H-bridge winches from Pi commands and stops a winch automatically
when its INA260 current sensor sees a stall.

Hardware: NUCLEO-F411RE · Greartisan 12 V 10 RPM worm-gear winch · Adafruit INA260 (4226)

---

## Files

| File | Job |
|---|---|
| `winchDriver.c/.h` | H-bridge state machine for one winch (start, idle, up, down, cooldown, stalled, fault) |
| `currentSensor.c/.h` | INA260 over I2C: reads current/voltage, decides "is this a stall?" |
| `winchSystem.c/.h` | Owns every winch and its sensor, links them, and is the **only** API `main.c` uses |

`winchDriver` and `currentSensor` don't know about each other. `winchSystem.c` is the only file that connects them.

Put the `.c` files in `Core/Src` and the `.h` files in `Core/Inc`.
**Only ever `#include` the `.h` files.** Including a `.c` file causes "multiple definition" linker errors.

---

## Winch ↔ sensor mapping

Each winch is matched to its sensor by **index**: entry `i` in `winches[]` uses entry `i` in `sensors[]`
(both tables are at the top of `winchSystem.c`). The index is also what you pass to every `WinchSys_*` function.

| Index | Winch | CubeMX user labels (GPIO_Output) | INA260 address | A0/A1 jumpers | Pi packet field |
|---|---|---|---|---|---|
| `0` | Winch 1 | `Right_High`, `Right_Low`, `Left_High`, `Left_Low` | `0x40` | none (default) | 4th (`winch1`) |
| `1` | Winch 2 | `W2_Right_High`, `W2_Right_Low`, `W2_Left_High`, `W2_Left_Low` | `0x41` | A0 bridged | 5th (`winch2`) |

- Winch 2 is only built once the four `W2_*` labels exist in CubeMX. Until then the system runs with one winch, and calls for index 1 are ignored.
- The labels must be spelled **exactly** as above. CubeMX turns them into `#define`s in `main.h`, and the code looks for those names.
- If you add a winch, add a row to **both** tables. A compile-time check (`_Static_assert`) fails the build if the two tables have different lengths.

### Sensor wiring (each INA260)

| INA260 pin | Connect to |
|---|---|
| VCC | Nucleo 3.3 V |
| GND | Nucleo GND (common with the 12 V supply ground) |
| SCL / SDA | I2C1: PB8 = D15 (SCL), PB9 = D14 (SDA). Shared by all sensors |
| VIN+ / VIN− | **In series with the 12 V feed to that winch's H-bridge** (VIN+ toward the supply) |

On the high side, one sensor measures the motor current in both directions.
The boards already have I2C pull-up resistors, so don't add your own.

---

## How it works

Every main-loop pass, `WinchSys_Update()` does this for each winch:

1. **Read the sensor:** at most every 5 ms, to match the chip's update rate.
2. **Run the winch state machine** with the Pi's command. If the sensor isn't answering, the command is treated as **stop**.
3. **Check for a stall:** only while the winch is driving.
   - It ignores the first `blankMs` after the motor starts, because the startup surge looks like a stall.
   - If the current then stays at or above `stallmA` for `stallMs`, it calls `Winch_Stall()`. Outputs go off right away.

### Winch states

| # | State | Meaning | Leaves when |
|---|---|---|---|
| 0 | `WINCH_START` | Just booted. Waits for cmd 0, so it won't take off if the Pi is already sending "run" | cmd = 0 → IDLE |
| 1 | `WINCH_IDLE` | Off, ready | cmd 1/2 → DOWN/UP |
| 2 | `WINCH_UP` | Driving up | cmd changes → COOLDOWN |
| 3 | `WINCH_DOWN` | Driving down | cmd changes → COOLDOWN |
| 4 | `WINCH_COOLDOWN` | Off for 500 ms between moves, to avoid stressing the gears and the H-bridge | 500 ms → IDLE |
| 5 | `WINCH_STALLED` | Stopped by the current sensor | cmd = 0 → COOLDOWN (the operator has to release) |
| 6 | `WINCH_FAULT` | Locked off: sensor missing at boot, or the shoot-through guard tripped | reset only |

Pi command values: `0` = stop, `1` = down, `2` = up.

### Stall settings (`sensors[]` in `winchSystem.c`)

Requires testing to find real world values for stall mA/Ms and start up period 

| Field | Default | Meaning |
|---|---|---|
| `stallmA` | 2000 | Current that counts as "too high" |
| `stallMs` | 100 | How long it must stay too high to count as a stall |
| `blankMs` | 300 | Startup-surge window to ignore after the motor starts |

**The defaults are placeholders.** Bench-test each motor first: log its normal-load current, its stall current and how long the startup surge lasts.
Set `stallmA` between normal-load and stall current, and set `blankMs` a bit longer than the surge.

### INA260 setup (done automatically in `CS_Init`)

- Reads register `0xFE` and expects `0x5449` ("TI"), to confirm the chip is there.
- Writes `0x8000` to the config register (resets the chip), then `0x6227`: average 4 samples, 1.1 ms current measurements, continuous mode. A new reading is ready about every 5 ms.
- Current is register `0x01` (signed, 1.25 mA per count). Bus voltage is register `0x02` (1.25 mV per count).

---

## Making it work in a project

### 1. CubeMX (`.ioc`)
- **I2C1:** Connectivity → I2C1 → mode **I2C**. Move the pins to **PB8 (SCL)** and **PB9 (SDA)**. Set **Fast Mode, 400000 Hz**. No interrupts needed.
- **Winch pins:** add the eight user labels from the mapping table as **GPIO_Output** pins.
- Save to generate the code. That creates `hi2c1` and `MX_I2C1_Init()`.

### 2. `main.c` (the only changes)
```c
/* USER CODE BEGIN Includes */
#include "winchSystem.h"

/* USER CODE BEGIN 2 */           // runs after MX_GPIO_Init() and MX_I2C1_Init()
WinchSys_Init(&hi2c1);

/* in while(1), after the failsafe block */
WinchSys_SetCommand(0, winch1Cmd);
WinchSys_SetCommand(1, winch2Cmd);
WinchSys_Update();

/* USER CODE BEGIN Error_Handler_Debug */
WinchSys_StopAll();
```

---

## API (`winchSystem.h`)

| Function | Use |
|---|---|
| `WinchSys_Init(&hi2c1)` | Once at startup. Sets up the winches, finds the sensors, and faults any winch whose sensor is missing |
| `WinchSys_SetCommand(i, cmd)` | Give winch `i` its latest Pi command (0/1/2) |
| `WinchSys_Update()` | Every loop: reads the sensors, runs the state machines, checks for stalls |
| `WinchSys_StopAll()` | All outputs off, using GPIO writes only. Safe in `Error_Handler` |
| `WinchSys_Count()` | Number of winches actually built (1 or 2) |
| `WinchSys_GetState(i)` | Winch state, a number 0–6 |
| `WinchSys_GetCurrentmA(i)` | Last current reading, in mA (signed) |
| `WinchSys_GetBusmV(i)` | Last supply voltage reading, in mV |

---