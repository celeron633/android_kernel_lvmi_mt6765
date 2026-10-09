# S17Promini AXS15260 basic touch bringup

2026-10-09. The display reconstruction was committed as `b11b89b4bdad`.
The initial touch implementation is `drivers/input/touchscreen/axs15260_s17.c`
and its packet decoder `axs15260_s17.h`. It is enabled by the stock-named
`CONFIG_TOUCHSCREEN_MTK_AXS` in `S17Promini_defconfig`.

This stage reads existing firmware and reports contacts. It does not restore
automatic firmware upgrade, gesture wake or all vendor recovery paths. The
touch object and updated DTB compile, but no device test has been performed.

## Stock evidence

Use the exact stock kernel hash and Image base in `S17Promini-axs15260.md`.
The original touch probe starts at Image offset `0x9f886c`.

- At `0x9f88c4`, probe compares client address with **0x3b**, then replaces
  other addresses with 0x3b. The BSP DT node `touch@38` is not the actual
  bus address used by this firmware.
- At `0x9f8e10`, the allocated report buffer's read length is set to 32.
  The report IRQ thread starts at `0x9f92a4` and calls `0x9f9c9c`, a plain
  one-message I2C read with no register-address write before it.
- Reports have two header bytes followed by up to five six-byte records.
  `buf[0] != 0` or `buf[1] & 0xf0` takes a special/ESD path; ordinary count
  is `buf[1] & 0x0f`. Stock stops at the first contact ID >= 5.
- Record X = ((p[0] & 0x0f) << 8) | p[1]; Y similarly uses p[2]/p[3].
  Event = p[0] >> 6, ID = p[2] >> 4, pressure = p[4], area = p[5] >> 4.
  Active events are 0/2; zero pressure/area defaults are 63/9.
- Original input reports are type A, device name `axs_ts`, with inclusive
  X/Y maxima 384/854. Recovery input-device logs corroborate these axes.
- Version reader `0x9f84a8` sends command **0x0c**, then reads two bytes in
  one combined transfer, assembling them in big-endian order.
- The scheduled firmware work at `0x9fc40c` references a 50192-byte embedded
  image at `0x10d0e89`. Backend version base is 0x5034, with another offset
  taken from firmware header bytes 0x16/0x17, yielding 0x7634 and version
  **0007**. SHA256 is
  `92332cbd1e3da9e39647951a3bd198ee65a43ef199b9db01aad83cd7d2536e1d`.
  Upgrade decision/dispatch is at `0x9fb644`; writing flash remains later work.

## Device integration

The board DTS removes the old I2C0 `touch@38` child and creates
`axs15260@3b`, compatible `lvmi,s17promini-axs15260`.

- IRQ is GPIO 0, falling edge, matching the stock 0x2002 IRQ request flags.
- Default pinctrl uses the stock `eint@0` group (phandle 0x105).
- `vtouch-supply` points to the same MT6357 vldo28 regulator (phandle 0x8c).
- `touchscreen-size-x/y` are 385/855 because the standard input helper
  subtracts one; this reproduces the stock inclusive maxima 384/854.

The LCM bias driver continues to own/reset GPIO 174. The touch driver does
not request that GPIO or reset the controller again at probe. This avoids
competing GPIO requests while display/touch firmware coordination is being
reconstructed. Supply reference counting preserves the touch power supply
through screen/system suspend at this stage.

The new driver reports all active contacts using the original type-A layout.
It explicitly releases a previous contact on a valid empty frame or a failed
I2C read, and ignores special headers. It rejects counts > 5 rather than
silently clamping them like stock. These defensive handling choices are not
a byte-identical recreation of the vendor IRQ handler.

Framebuffer early powerdown and late unblank gate IRQ reporting; system PM
has a separate suspend flag so nested callbacks do not unbalance IRQ depth.
IRQ teardown precedes managed input teardown. No vendor sleep command or
firmware/flash writes have been introduced. IRQ observation during panel
reset and readiness after unblank still need runtime validation.

Probe logs the current firmware version or that the version read is not ready,
then registers the input path for diagnosis. Thus a registered `axs_ts` node
alone does not establish that firmware is running or reports are arriving.

## Validation

Use the same compiler arguments as `S17Promini-bringup.md`, with targets
`S17Promini_defconfig drivers/input/touchscreen/axs15260_s17.o dtbs`.
The object and updated full DTB built successfully. Checkpatch reports no
errors or warnings for the driver/header.

In `~/S17Promini_LineageOS`:

```sh
python3 tools/reverse_axs_touch.py
python3 tools/check_axs_touch_protocol.py
```

The first tool hash-checks the original image, saves selected disassembly and
extracts the stock firmware into the ignored `extracted/kernel-reverse` tree.
It does not upload the blob. The second compiles the actual packet header and
checks maxima, five contacts, event flags, default pressure/area, sentinel IDs,
empty reports, special headers and malformed packets. These are synthetic
packet checks, not an emulated controller or an actual IRQ/input-device test.

Build the complete kernel and obtain a test boot before checking I2C probe,
firmware version, IRQ counts and real multitouch in both system and recovery.
Decide from that evidence whether the original flash/firmware logic is needed.
