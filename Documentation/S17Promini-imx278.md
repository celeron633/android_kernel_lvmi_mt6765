# S17Promini IMX278 camera port

2026-10-11. This is a source port for the existing MediaTek imgsensor v1
framework and stock vendor HAL. The user confirmed the combined r13 camera
repair on 2026-10-11; see `S17Promini-gc5025sub.md` for the follow-up.

## Source provenance

The donor is [h0sch180/android_kernel_cubot_max](https://github.com/h0sch180/android_kernel_cubot_max/tree/7450eaa6b7662499002982526e81973ca0e5f433/drivers/misc/mediatek/imgsensor/src/mt6753/imx278_mipi_raw),
commit `7450eaa6b7662499002982526e81973ca0e5f433`, MT6753 IMX278 C/header.
Its MT6735 version and [ReMeizu/android_kernel_meizu_mt675x](https://github.com/ReMeizu/android_kernel_meizu_mt675x/tree/b86c36b01ef08399f064cd2f60cb8fcf6d0b164c/drivers/misc/mediatek/imgsensor/src/mt6755/imx278_mipi_raw),
commit `b86c36b01ef08399f064cd2f60cb8fcf6d0b164c`, were also examined.
The Meizu version has board-specific preview clocks/Bayer/mirror changes;
those changes are not used for S17.

Stock gzip kernel SHA256:
`5851e16038244576d0a863d9f36b4ad602af68ea45477f5c6f44a9dfb0127286`.
Raw Image VA base is `0xffffff8008080000`. Targeted analysis uses the user's
Ghidra 12.1.4 installation. Work-directory scripts are
`tools/S17CameraDecompile.java` and `tools/reverse_imx278.py`;
`extracted/metadata/ghidra-camera/` contains the decompilation,
and `stock-imx278-audit.json` contains the register/crop/gain/power evidence.
The stock absolute kallsyms address table starts at Image offset `0x1136400`.
IMX278 init is `0x877010`; sensor init is `0x879d70..0x87bb18`;
preview setting is `0x87bca4..0x87c5a4`.

## S17 settings

The stock I2C write-address list is `0x34, 0x20, 0x40, 0xff`. ID is read
through OTP page `0x13`, registers `0x0a28/0x0a29`, and must equal `0x0278`.
Mirror/flip is written as `0x0101=3` during stock initialization;
the advertised output format is RAW_B. MCLK is 24 MHz, four MIPI lanes.

| Mode | Output | Pixel clock | Line length | Frame length | Max fps |
| --- | --- | --- | --- | --- | --- |
| Preview | 2100 x 1558 | 480 MHz | 4976 | 3200 | 30 |
| Capture | 2100 x 1558 | 480 MHz | 4976 | 3200 | 30 |
| Capture cap1 declaration | 2100 x 1558 | 388 MHz | 4976 | 3220 | 24 |
| Normal video | 2100 x 1558 | 480 MHz | 4976 | 3200 | 30 |
| High speed video | 1316 x 480 | 480 MHz | 4976 | 800 | 120 |
| Slim video | 1400 x 782 | 184 MHz | 4976 | 1228 | 30 |

Stock capture uses the same 72 register writes as preview, including scaling
to 2100 x 1558. Its cap1 declaration changes software timing but retains that
same register sequence; this port preserves that stock behavior. Do not use
the donor's full-resolution capture declaration/registers without further
hardware/HAL validation.

All 249 stock initialization writes are retained in order. Compared with the
donor, values differ at `0x7709`, `0x9707`, `0x989f`, `0x98c3`, `0x3011`,
and stock adds `0x0101=3`. All four mode-setting sequences contain 72 writes.
The five crop structures match stock. Gain mapping entry 60 is
`{198, 347}`; the donor's `{198, 247}` is corrected.

Both S17 sensors use the stock power sequence:
MCLK high; DOVDD 1.8 V/10 ms; AVDD 2.8 V/10 ms; DVDD 1.2 V/10 ms;
AFVDD 2.8 V/15 ms; PDN low/10 ms; RST low/10 ms;
PDN high/10 ms; RST high/10 ms. Off states are zero as in stock.
For the paired S17 sensor configuration, front AVDD uses the regulator
backend (`cam1_vcama`), rather than the donor board's GPIO backend.
Stock leaves AFVDD's backend slot at zero, which selects the regulator.

## Framework integration

`imgsensor_sensor_list.c/.h` register and declare IMX278. The existing
MT6765 Makefile discovers its common/v1 directory from the sensor config.
Registry order preserves stock indices: GC5025sub is 0 and IMX278 is 1.
`S17Promini_boot_test.config` selects both stock sensors. The initial IMX278
test used `-s17-bringup-r12-camera`; the GC5025sub follow-up uses r13,
as described in `S17Promini-gc5025sub.md`.

The port uses the current struct/enum names and `pr_debug`, removing obsolete
xlog/kd_camera_hw includes. Zero FPS requests are guarded before division.
The donor crop-query fallthrough into the three-slot HDR query is fixed.
Donor ISO/AWB extensions absent in stock are omitted. GC5025sub's sensor
implementation is retained; this change aligns its S17 power routing/table.

## Validation and device follow-up

`python3 tools/reverse_imx278.py` checks the verified stock hash and compares
all initialization/mode register writes, crop structures, and gain mappings
with the port. It uses Ghidra output for writes, handling the shared high-speed
and slim-video tail explicitly. `--symbols-only` generates Ghidra TSV bounds.

The driver passes kernel checkpatch without errors or warnings. Final full
boot build and image verification are recorded in the work-directory
`docs/103-imx278-camera-port.md`.

On the device, test main/front opening, live preview, still capture, video,
camera switching and closing/reopening. Capture full `dmesg`,
`logcat -b all -d`, and `dumpsys media.camera` if anything fails.
Compile success and stock register matching do not establish working
SENINF/ISP/HAL behavior, autofocus, or front sensor OTP calibration.
