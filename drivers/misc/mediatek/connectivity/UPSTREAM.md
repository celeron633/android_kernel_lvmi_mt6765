# S17 in-tree connectivity sources

Imported on 2026-10-11 for MT6765 / CONNAC1x. Existing connadp sources
remain from this kernel. Copyright and license notices remain in the imports.

Motorola repositories, tag `MMI-S3SG32.39-191R2`:

| Directory | Repository | Commit |
| --- | --- | --- |
| common (except existing adapter files) | https://github.com/MotorolaMobilityLLC/vendor-mediatek-kernel_modules-connectivity-common | 7d4b50bdd13a6f0e9642864c1b9c0a22e02141a2 |
| wlan/adaptor | https://github.com/MotorolaMobilityLLC/vendor-mediatek-kernel_modules-connectivity-wlan-adaptor | 507897d578adfdab84cf1ba7c016799ca0d2acc7 |
| wlan/core/gen4m | https://github.com/MotorolaMobilityLLC/vendor-mediatek-kernel_modules-connectivity-wlan-core-gen4m | e98a79d085446a96344450da24282c2881fcef63 |
| bt/mt66xx/wmt and bt/mt66xx/include | https://github.com/MotorolaMobilityLLC/vendor-mediatek-kernel_modules-connectivity-bt-mt66xx | 92dbcc5627c168a6798b3be8c7faafab0b036b01 |

connfem comes from
https://github.com/MotorolaMobilityLLC/vendor-mediatek-kernel_modules-connectivity-connfem,
branch `android-12-release-sst`, commit
`a0b2549be29f5cf4f3f212da2baa9523ab64698f`. The WMT release tag is absent
from that repository. The Wi-Fi consumer API and connfem types are present;
compile and runtime compatibility still require validation.

Local changes:

- Kconfig and top-level Kbuild select the five drivers as built-ins, using
  local source paths and MT6765 / AXI / CONNAC / CONNAC1x options.
- Per-driver Makefiles use CONNECTIVITY_ROOT, fixed composite object names,
  and the in-tree build flag. gen4m's duplicated object rules are removed.
- MTK_WLAN_SERVICE=yes selects the bundled service sources and headers
  required by this release's enabled QA interface, matching the official
  Motorola module build parameters.
- WMT's mmi_get_bootarg is file-local. gen4m retains its separate global
  implementation for wlan_lib.c, avoiding a built-in symbol collision
  while keeping the two parsers' bootargs buffer ownership separate.
- WMT and Wi-Fi configuration selectors stop at the empty product-table
  terminator. Devices without Motorola bootargs use the existing defaults,
  instead of requesting WMT_SOC_.cfg or .cfg.
- MT6765's driver DMA MPU region is 24, matching S17's stock Wi-Fi module
  disassembly. Access permissions and the actual allocated range are kept;
  the region/range/permission words are logged for runtime verification.
- The loader's initialization chain calls WMT, BT and Wi-Fi. FM and GPS
  callbacks are excluded. Initialization is serialized and caches the real
  result, including failures. The in-tree BT and Wi-Fi hooks have no weak
  success fallback.
- Motorola's channel 12/13 exclusion flag is explicitly set to 0; the
  driver uses its normal regulatory/NVRAM handling. The numeric #elif
  requires a defined value under -Wundef.
- External Android.mk recipes are omitted. Android must build these drivers
  only through this kernel, not as additional vendor modules.
- Imported trailing whitespace, mixed leading indentation and blank lines
  at EOF are normalized where reported by git diff --check.

WMT detection and connfem register through normal kernel initcalls.
The existing vendor wmt_loader ioctl initializes the remaining drivers after
vendor is mounted. Keep wmt_loader, wmt_launcher, HALs and stock firmware;
remove vendor insmod actions for these drivers. S17 does not enable GPS/FM.

The integration passed Kconfig, Kbuild rule and init-script checks. The user
built it and confirmed the reported compile/link issues are resolved after
the local build fixes. After the configuration selector and DMA MPU region
fixes, the user confirmed Wi-Fi startup, scanning, connection and Internet
access, as well as Bluetooth device discovery. Suspend/resume, Bluetooth
pairing and audio testing remain pending.
