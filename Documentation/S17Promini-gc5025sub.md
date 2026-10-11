# S17 GC5025sub frontend correction

2026-10-11. The r12 device logs detect IMX278 at write address 0x34/ID 0x278
and GC5025sub at 0x6e/ID 0x5026. The app's failing capture sessions use camera
1, advertised as FRONT, and the kernel executes GC5025sub capture control.
SENINF then reports incomplete vertical input and the HAL provider dies after
timeouts. The logs do not establish working IMX278 image capture.

The common/v1 GC5025sub driver was reconstructed for marlon. It advertises
1296x972 preview, 175.2 MHz capture, 2920 line length and 2008 frame length,
and programs a different set of registers during mode changes, including
streaming control through 0x3e. It cannot be assumed to match the S17 module.

## Board-specific source

S17 now selects a project driver under
`drivers/misc/mediatek/imgsensor/src/mt6765/camera_project/r65v3_pmz_w1a/gc5025sub_mipi_raw/`.
The existing Makefile gives project drivers precedence over common/v1;
the common driver is retained for its existing users.

The donor is [deadman96385/android_kernel_zte_mt6761](https://github.com/deadman96385/android_kernel_zte_mt6761/tree/b0c0f1ec4cb6d616090e7879a7df0785a1323c5d/drivers/misc/mediatek/imgsensor/src/common/v1/gc5025_mipi_raw),
commit `b0c0f1ec4cb6d616090e7879a7df0785a1323c5d`, with the older GC5025
OTP/control implementation. Its board settings are replaced with verified
S17 stock settings, rather than used directly.

Stock gzip kernel SHA256:
`5851e16038244576d0a863d9f36b4ad602af68ea45477f5c6f44a9dfb0127286`.
Ghidra 12.1.4 targeted analysis uses `tools/S17CameraDecompile.java` in the
work directory, with `stock-gc5025sub-bounds.tsv`. Functions occupy Image
offsets `0x873b78..0x877010`. No whole-kernel auto-analysis is needed.

## Stock alignment

- All 107 global initialization writes match stock, including bank changes,
  reset `0xfe=0x80`, mirror `0x17=0xc3`, MIPI configuration, and `0x3f=0x91`.
- All five legacy scenarios report 2592x1944, pixel clock 72 MHz, line 2400,
  frame 988, max FPS 300. All five crop structures match stock. Frame/shutter
  units follow this driver, including stock's factor of two for shutter.
- Output is RAW_R, MCLK 24 MHz, ISP driving current 4 mA, two MIPI lanes;
  checksum is `0xf7375923`, SensorModeNum is 3 as in stock.
- Scenario changes retain the initialized sensor registers. Streaming
  suspend/resume and donor pixel-rate extensions absent in stock are omitted.
- Physical ID 0x5025 is mapped to the front driver's logical ID 0x5026.
- Shutter writes bank 0, registers 0x03/0x04, with doubled/clamped exposure.
  Gain uses the stock two-stage 64/92 thresholds and 0xb6/0xb1/0xb2 registers.
  Frame-rate changes update software state without donor VB register writes.
- OTP reads use stock 0xd4/0xd5/0xf3/0xd7 access, page 0/page 1 split,
  DPC checksum address 0x75, defect expansion and H/V coordinate transform,
  SRAM programming, and chip register patches from address 0x62.
  The donor's extra version probe/reset mode and digital gain compensation
  are removed. Module/WB metadata that stock only saves/logs is not exposed;
  this port does not apply additional AWB correction.

The donor's empty-count sorting underflow and last-slot lookahead are removed
using a bounded sort by y then x. OTP record count is checked before filling
the 200-byte input buffer. Up to 50 records can expand into 200 SRAM defects.
Malformed counts are marked invalid. Valid records retain stock handling,
including diagnostic-only checksum mismatch and no duplicate coalescing.

## Validation

`tools/check_s17_gc5025sub.py` verifies stock init/crop/timing and compiles
the actual driver OTP, shutter, gain and mode functions with mocked register
IO under ASan/UBSan. Cases cover empty OTP, 0/4/200 defects, invalid count,
SRAM ordering, chip patching, exposure/gain boundaries and stable mode writes.
The C driver passes checkpatch with zero errors/warnings.

Full r13 boot build/image results are recorded in the work directory's
`docs/104-camera-front-gc5025sub-stock.md`. Device image delivery does not
establish live capture or autofocus success; both FRONT and BACK need retest.

2026-10-11: the user confirmed the camera repair works and corrected the
earlier default-camera report: it was the front camera. The combined r13
camera repair is device validated. No separate mode/AF/video results or new
logs were supplied with that confirmation.
