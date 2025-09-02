STM32F407 Cable Tension Analysis System
=================================================
Real-time, single-sensor cable tension estimation on an STM32F407 with on-device analytics and SD-card data logging. Designed for embedded structural health monitoring (SHM) scenarios where short windows of acceleration are processed continuously at the edge.

Academic license: This project is provided for academic and research purposes only. Commercial use requires explicit permission from the author.

-------------------------------------------------
Project Status
-------------------------------------------------
- Current app: Real-time single-sensor tension analysis + data logging
- Target MCU: STM32F407VGT6 (ARM Cortex-M4 @ 168 MHz)
- Toolchain: Keil MDK-ARM (uVision) project included
- Serial: 115200 baud (default)
- SD card: FATFS, files read/written via SDIO

-------------------------------------------------
Features
-------------------------------------------------
- Timer-triggered pipelines: 100 ms period triggers windowed processing.
- Real-time tension estimation from acceleration (dominant frequency + mode).
- Packet-loss aware processing (optional mask per sample).
- On-device logging: raw, reconstructed signals and analysis results to SD card.
- External SRAM utilization for minute-scale sliding windows.
- Interactive control via KEY0/KEY1/WK_UP and LED status indicators.
- Clear HAL/CMSIS separation with reusable BSP modules.

-------------------------------------------------
Folder Structure (overview)
-------------------------------------------------
Drivers/
  BSP/ (KEY, LED, NORFLASH, RNG, SDIO, SPI, SRAM, TIMER)
  CMSIS/ (Device headers, startup, DSP headers & lib)
  STM32F4xx_HAL_Driver/ (Inc, Src, Legacy)
Middlewares/
  DATALOG/ (signal & result logging)
  FATFS/ (FatFs + examples/resources)
  MALLOC/ (multi-pool heap: SRAMIN/SRAMEX/SRAMCCM)
  TENSION/ (tension estimation core)
Output/ (example hex files)
Projects/MDK-ARM/ (Keil project: cable_force.uvprojx)
User/ (config.h, main.c, stm32f4xx_it.*, stm32f4xx_hal_conf.h)

-------------------------------------------------
Hardware & I/O
-------------------------------------------------
- Board: Any STM32F407 with SDIO, external SRAM, user LEDs, and keys.
- Sensor: 1-axis acceleration (demo reads float samples from SD; adapt for ADC DMA for live input).
- Storage: microSD card (FAT32).
- SRAM: External SRAM via Drivers/BSP/SRAM.
- LEDs:
  - LED0 blinks during analysis
  - LED1 toggles on timer trigger
- Keys:
  - KEY0: Load data files and start analysis
  - KEY1: Stop and reset system
  - WK_UP: Toggle SD logging on/off

-------------------------------------------------
Build & Flash (Keil MDK-ARM)
-------------------------------------------------
1) Open Projects/MDK-ARM/cable_force.uvprojx in Keil uVision.
2) Select STM32F407VG (or your exact 407 variant).
3) Ensure optimization, FPU, and DSP settings are correct.
4) Build (F7) and flash (Ctrl+F8).
5) Connect UART at 115200 8N1 to view logs.
Note: Clock set to 168 MHz via sys_stm32_clock_init(336, 8, 2, 7).

-------------------------------------------------
Quick Start (SD-based demo)
-------------------------------------------------
1) Prepare microSD (FAT32) with two files in root:
   - real_signal.bin — float32 samples, length multiple of SIGNAL_LEN
   - real_mask.bin (optional) — uint8 mask (1=valid, 0=lost), same length as signal
2) Insert SD, power on board.
3) Open serial terminal at 115200 baud.
4) Press KEY0 to load and start analysis.
5) Observe live results and logging to SD.
6) Press KEY1 to stop, finalize logs, and reset.
7) Press WK_UP to enable/disable logging at any time.

-------------------------------------------------
How It Works (100 ms loop)
-------------------------------------------------
1) Select window: SIGNAL_LEN samples (and mask) from the minute-scale buffer.
2) Detect packet loss and compute window loss rate.
3) Extract frequency and mode features.
4) Estimate tension and confidence.
5) Log to SD and print to UART.
6) Advance by WINDOW_STRIDE and repeat.

Timer: gtim_timx_int_init(9999, 8399) for ~100 ms period at 84 MHz timer clock.

-------------------------------------------------
Data, Logging & Files
-------------------------------------------------
Inputs (SD root):
- real_signal.bin — float32 (little-endian), up to 60000 samples by default.
- real_mask.bin — uint8 (1/0). If missing or length mismatch, defaults to all 1s.

Outputs (see Middlewares/DATALOG for exact filenames):
- Original signal data (windowed)
- Reconstructed signal data (if provided by the tension module)
- Tension results per window (tension, frequency, mode, confidence, timing)
- Analysis log (session summary)

-------------------------------------------------
Configuration
-------------------------------------------------
- SIGNAL_LEN: samples per processing window
- WINDOW_STRIDE: hop size between windows
- DESIGN_TENSION: for sanity checks and error reporting
- Memory pools and logging sizes: see MALLOC/malloc.h and DATALOG headers
- SDIO/FATFS settings: FATFS/source/* and BSP/SDIO/*

Memory pools in system_init():
- my_mem_init(SRAMIN);
- my_mem_init(SRAMEX);
- my_mem_init(SRAMCCM);

-------------------------------------------------
Runtime Controls & Messages
-------------------------------------------------
- KEY0: start (load SD files, start timer + analysis)
- KEY1: stop (finalize logs, reset)
- WK_UP: toggle data logging
- UART prints per-window details and periodic stats.

Example UART line:
Tension: 3152.43 kN | Frequency: 1.62 Hz | Mode: 18 | Confidence: 0.987

-------------------------------------------------
Extending the Project
-------------------------------------------------
- Live ADC input via DMA ring buffer feeding process_realtime_data().
- Multi-sensor scheduling or interleaved pipelines.
- Alternative tension models or modal ID in Middlewares/TENSION.
- Edge/cloud split: keep feature extraction on-device and stream features.

-------------------------------------------------
Known Limits
-------------------------------------------------
- External SRAM buffers default to 60000 samples (tune to your memory map).
- SD throughput depends on card quality and FATFS configuration.
- Demo assumes float32 input (LE) for SD files.

-------------------------------------------------
Build Notes
-------------------------------------------------
- Link CMSIS DSP: CMSIS/DSP/Lib/ARM/arm_cortexM4lf_math.lib
- Enable single-precision FPU in Keil target options.
- Startup: CMSIS/Device/ST/STM32F4xx/Source/Templates/arm/startup_stm32f407xx.s
- HAL modules included under STM32F4xx_HAL_Driver.

-------------------------------------------------
Versioning
-------------------------------------------------
- V1.0 (2025-06-01): Initial public release (standardized headers, English docs).
- V2.0 (2025-07-16): Real-time pipeline + logging, expanded runtime stats, SD demo.

-------------------------------------------------
Citation
-------------------------------------------------
If this project supports your research, please cite:
Dan Li, "STM32F407 Cable Tension Analysis System (Edge SHM Toolkit)", 2025.

-------------------------------------------------
License & Contact
-------------------------------------------------
Copyright © 2025 Southeast University.
Academic and research use only; commercial use requires explicit permission.

Contact: Dan Li — danli@seu.edu.cn
