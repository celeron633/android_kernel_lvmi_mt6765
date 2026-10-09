# S17Promini AXS15260 reconstruction

2026-10-09: the original panel implementation has been reconstructed and the
LCM subtree and updated DTB compile. No complete kernel/boot image or device
test has been performed.

## Evidence

Stock gzip kernel SHA256:
`5851e16038244576d0a863d9f36b4ad602af68ea45477f5c6f44a9dfb0127286`.
It decompresses to a 25743360-byte ARM64 Image, Linux 4.19.191.
Virtual Image base is `0xffffff8008080000`.

The unique name string is at Image offset `0x1461939`; its pointer occurs at
`0x176e8b0`, the start of the stock LCM_DRIVER. The next pointers identify:

| Callback/helper | Image offset |
| --- | --- |
| set_util_funcs | 0x6e77d4 |
| get_params | 0x6e7808 |
| init | 0x6e78c4 |
| suspend | 0x6e794c |
| resume | 0x6e79a8 |
| compare_id | 0x6e79c0 |
| bias enable | 0x6e7b80 |
| push_table | 0x6e7bc8 |
| bias register write | 0x6e7c4c |
| AXS touch reset (axs_reset_proc) | 0x9f8598 |

The stock separate init_power/suspend_power/resume_power pointers are null.
Bias handling is inside init/suspend; resume calls init.

`set_util_funcs` copies 0xf8 bytes, matching the current LCM_UTIL_FUNCS.
Stock get_params clears 0x518 bytes; the current LCM_PARAMS is 0x4e0 bytes.
The reconstructed callback clears its own structure size. All values written
by stock are inside that current structure, with matching member offsets.

## Active initialization table

The actual table at Image `0x176e9f8` has 8 entries of 0x48 bytes, with u32
command, u8 count, and 64-byte payload:

| Command | Payload / action |
| --- | --- |
| BB | 00 00 00 00 00 00 5A A5 |
| B2 | 01 00 00 06 00 0A 04 21 9C 00 1E |
| B2 | 41 |
| 11 | 00 |
| 0FFE | wait 120 ms |
| 29 | 00 |
| 0FFE | wait 10 ms |
| 0FFF | end |

Keep the one-byte payloads on 11/29: that is what the original sends through
dsi_set_cmdq_V2. Do not substitute the larger ESP32 AXS15260D panel table.
The same 576 bytes occur in the original LK at file offset `0x90780`.

The suspend table at Image `0x176ec38` sends 28/00, waits 120 ms, sends 10/00,
waits 30 ms, then ends. Suspend additionally waits 120 ms before disabling
bias, then another 20 ms. LK has different suspend delays (20 and 120 ms);
use the kernel behavior for the kernel port.

## Power and resets

Stock bias enable selects display GPIO state 10 (`lcd_bias_enp1_gpio`, GPIO
150), waits 20 ms, and writes registers 00 and 01 with value 12 to I2C address
3e. Suspend selects state 9 (`lcd_bias_enp0_gpio`). The stock DT node is
`/i2c5@11016000/lcm_i2c@3e`, compatible `mediatek,I2C_LCD_BIAS`.
The new panel registers a matching I2C driver. It reports missing clients,
short writes and pinctrl errors and stops initialization on a bias failure.
Unlike LK, the kernel bias helper does not send register FF/value 80.

After bias enable, stock init calls `axs_reset_proc(20)`. This is not a plain
20-ms delay: it drives touch reset high, sleeps 1000-1100 us, drives it low,
sleeps 10000-11000 us, drives high, then waits 20 ms. LK confirms GPIO 174;
stock touch pinctrl also uses pinmux 0xae00.

The panel then resets its own line high/20 ms, low/20 ms, high/100 ms before
sending the table. compare_id uses bias and panel reset, but no touch reset;
it submits 00013700, waits 10 ms, reads one byte from DA/DB/DC, and matches
the assembled value 005260.

For this display-only stage, the new bias driver temporarily owns touch reset
through `touch-reset-gpios` added to its DT node, using the actual GPIO
controller `/pinctrl@1000b000` (phandle 1f), not the legacy gpio register node.
When the AXS touch driver is ported, move this coordination to that driver;
two drivers must not independently request the same reset line. This helper
does not restore touch input, firmware loading or touch suspend state logic.
Stock vtouch regulator is vldo28 with regulator-default-on; runtime supply
behavior still needs verification with the complete kernel.

## Display parameters and integration

384x854, SYNC_EVENT_VDO_MODE, 3 lanes, RGB888, PS=2, packet_size=256,
intermediate buffers=2. Vertical sync/back/front = 2/20/170; horizontal
sync/back/front = 20/180/180; PLL_CLOCK=210. Other fields, including ESD
check enable and SSC controls, remain zero as in stock.

The header had RGB888=2; stock kernel and LK both store format=3 with RGB888
packet selection and matching LK logs. Restore RGB666_LOOSE=2 and RGB888=3.
Existing drivers using named enum constants continue to select the same
named formats.

The new driver is registered in both LCM lists. The LCM Makefile no longer
unconditionally appends the two marlon panels: marlon_defconfig already
lists both, while S17 selects only axs15260_dsi_6580_hd.

## Validation and reproduction

Use the toolchain arguments in `S17Promini-bringup.md`, with targets
`drivers/misc/mediatek/lcm/ dtbs`. Build the parent directory so inherited
LCM/display include paths apply; a bare child object target misses them.
The LCM directory build, including its built-in archive, succeeded.
The updated full DTB also built; inherited stock binding/decompilation
warnings remain. The new reset property resolves to GPIO controller 1f,
pin 174, active high.

In the analysis workspace `~/S17Promini_LineageOS`:

```sh
python3 tools/reverse_axs15260.py
```

The hash-checked tool emits the tables, helper disassembly and replays the
stock get_params straight-line stores. Comparing compiled reconstructed C
data found:

- init table: all 576 bytes match stock;
- suspend table: all 360 bytes match stock;
- get_params: all 1248 bytes of the current structure match the stock prefix.

These checks establish data recovery and compilation, not hardware behavior.
Bootloader handoff, actual bias/reset readiness, first display frame, suspend
and resume still require a complete candidate kernel and device testing.
