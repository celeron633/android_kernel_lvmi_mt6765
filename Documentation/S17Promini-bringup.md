# S17Promini board configuration

Initial board description on `S17Promini-bringup`, based on the marlon tree at
`9fd0d5ca5f05` and S17Promini stock firmware. This is a configuration/DTB baseline,
not a boot-tested kernel.

## Inputs and layout

- `arch/arm64/configs/S17Promini_defconfig`: stock `kernel.config`, normalized
  through this tree's Kconfig and `savedefconfig`. Project `r65v3_pmz_w1a`,
  384x854 display, and PowerVR `rgx doma m1.11ED5425693` match stock.
- `arch/arm64/boot/dts/mediatek/s17promini-stock.dtsi`: complete stock boot DTB
  after applying stock DTBO entry 0. Numeric phandles and stock board wiring
  are kept together, including regulator, GPIO and reserved-memory properties.
- `arch/arm64/boot/dts/mediatek/S17Promini.dts`: board identity and actual LK
  panel name. LK logs identify `axs15260_dsi_6580_hd`; the static stock DTBO
  contains a stale NT35695B BSP name.
- The MediaTek DTS Makefile selects only `S17Promini.dtb` for this project.
  The output is a full standalone DTB, not an overlay. Appended-DTB and DTBO
  build features are disabled. Later boot packaging must use the full DTB
  and must not apply the same stock overlay again.

Inputs from `~/S17Promini_LineageOS/extracted/metadata`:

| Input | SHA256 |
| --- | --- |
| `dtb-0.dtb` | `c6a9172dbf31c851cceb44fab4ffd964f2364037f6a554dc6a3b1a840611beab` |
| `stock-overlay-0.dtbo` | `4f8b6e54d2ac754243ede0af22e52abcb1f80e98717d76aa43361c1f4363123e` |

Import procedure:

```sh
fdtoverlay -i dtb-0.dtb -o stock-merged.dtb stock-overlay-0.dtbo
dtc -I dtb -O dts -o stock-merged.dts stock-merged.dtb
```

The `.dtsi` contains that decompiled tree, with `/dts-v1/;` replaced by an
import provenance comment. Numeric references generate dtc warnings because
decompilation loses source reference markers. Stock BSP binding warnings also
remain; hardware properties have not been rewritten merely to silence them.

## Configuration prerequisites

The checkout had Kconfig `source` entries pointing at deleted OPPO directories
(foreground I/O, owakelock, fingerprint, touch, charging, audio, proc stats,
and oext4). Kconfig treats `#ifdef` as comments, so these entries broke every
configuration target. Remove missing source entries and use `fs/ext4/Kconfig`.
Initialize the pinned MySU submodule so its existing Kconfig source is available:

```sh
git submodule update --init MySU
```

MySU remains disabled in the S17 configuration. SELinux bootparam support and
the existing DSI diagnostics are enabled for bringup; stock SELinux defaults
remain enforcing.

Example from the kernel root, with this checkout's available tools:

```sh
S17_LINEAGE=/home/dengxh/android3/lineage
S17_CLANG="$S17_LINEAGE/prebuilts/clang/host/linux-x86/clang-r416183b/bin"
S17_GCC64="$S17_LINEAGE/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-"
S17_GCC32="$S17_LINEAGE/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-"
make O=/tmp/s17-kernel-bringup/out ARCH=arm64 \
  CC="$S17_CLANG/clang" CLANG_TRIPLE=aarch64-linux-gnu- \
  CROSS_COMPILE="$S17_GCC64" CROSS_COMPILE_ARM32="$S17_GCC32" \
  LLVM_IAS=1 S17Promini_defconfig dtbs
```

For this validation do not override LD with ld.lld: this tree adds
`-fuse-ld=lld` to compiler-only Kconfig probes, and Clang 12 then rejects the
unused argument under `-Werror`. Full kernel compiler/linker work is separate.

## Driver gaps

| Stock component | Current status |
| --- | --- |
| AXS15260 LCM | Reconstructed and registered; LCM subtree compiled. See `S17Promini-axs15260.md`. Device validation pending. |
| AXS touchscreen | `TOUCHSCREEN_MTK_AXS` absent. MTK touch selection is empty; ILITEK is not selected as a substitute. |
| PSC5425E charger | `CHARGER_PSC5425E` absent. Original DT node preserved; MT6370 is not enabled as a substitute. |
| WUSB3801 Type-C | WUSB3801X source exists, but remains disabled pending binding/driver review: stock node `wusb3801@60`, compatible `mediatek,usb_type_c`; driver searches `usb_type_c_wusb3801x` and matches `mediatek,usb_type_c_wusb3801`. |
| IMX278 camera | Driver absent; stock sensor string preserved for later camera work. GC5025sub source exists. |
| MIR3DA / GSENSOR_PROXIMITY | Stock `MTK_MIR3DA` and `GSENSOR_PROXIMITY` symbols absent; accel/ALSPS board nodes preserved, driver port pending. |
| R62_SYE_R62_BAT_1900 | Stock custom battery profile symbol absent; check battery tables and gauge behavior before charging tests. |

Unsupported stock boolean symbols are dropped by Kconfig normalization; the
table above records them explicitly. String selections do not validate that
their driver directories exist. Full kernel compilation still requires driver
work, and no boot image has been generated or flashed.

## Validation

At the board-baseline commit, `S17Promini_defconfig` and the kernel `dtbs`
target completed successfully.
Binary FDT comparison with the merged stock tree found 806 identical node
paths and 3652 properties. Only three property values differ: root `model`,
root `compatible`, and `/chosen/atag,videolfb-lcmname`. All remaining property
bytes match stock, including numeric phandle references.
The later AXS15260 reconstruction adds a touch reset GPIO property to the
bias node, while retaining the imported stock `.dtsi` unchanged.
