# Device build (aarch64, Debian 11, glibc 2.31)

```bash
cd flightsim
tools/build_device.sh                 # Docker: builds the image in this folder, then builds inside it
ls dist/                              # flightsim-probe-portmaster.zip, flightsim-portmaster.zip (after WP-00)
```

`tools/build_device.sh` has three ways to get the same Debian 11 environment (all run
`build_in_container.sh`, which builds, packages and checks):

| Mode | When |
|---|---|
| `--docker` (default if a daemon is reachable) | Developer machines and GitHub Actions (`.github/workflows/device.yml`). Image from `Dockerfile`. |
| `--chroot DIR` | No Docker (e.g. this sandbox). Create the chroot once with `tools/build_device.sh --setup-chroot DIR` (debootstrap + `install_toolchain.sh`). Needs root. |
| `--native` | You are already inside a bullseye environment with the toolchain. |

`install_toolchain.sh` is the single source of truth for the toolchain (cross g++ 10.2 from
`crossbuild-essential-arm64`, `libsdl2-dev:arm64` 2.0.14, cmake 3.18, ninja, qemu-user), used by both the
Dockerfile and the chroot. `bullseye` is past LTS: if `deb.debian.org` stops serving it, pass
`--mirror http://archive.debian.org/debian`.

Useful options: `--jsbsim-src DIR` (use a local JSBSim v1.3.3 checkout, no network), `--require-main`
(fail if the game itself cannot be built), `--skip-probe`.

## What gets built and checked

1. **Probe**: `tools/probe` configured with `cmake/toolchain-aarch64.cmake` (`-mcpu=cortex-a53`,
   `-static-libstdc++ -static-libgcc`, `RUNPATH=$ORIGIN/libs.aarch64`), JSBSim built as `libJSBSim.so`.
2. **Game**, when `CMakeLists.txt` and `src/main.cpp` exist (WP-00). Contract the packaging relies on:
   - target `flightsim`, output `<build>/flightsim`;
   - JSBSim built as a shared library, `libJSBSim.so*` anywhere under the build dir;
   - runtime layout in `<build>/jsbsim/` and `<build>/assets/` next to the binary (BUILD_AND_DEPLOY section 2;
     if there is no `<build>/assets`, the source `assets/` is used);
   - options `FSIM_WERROR` and `CMAKE_TOOLCHAIN_FILE` as in WP-00.
3. **Packaging**: `port/package_port.sh` assembles `Flight Sim.sh` + `flightsim/` (binary renamed to
   `flightsim.aarch64`, `libs.aarch64/`, `jsbsim/`, `assets/`, `licenses/`, `port.json`, `gameinfo.xml`,
   `README.md`, `screenshot.png`) into `dist/flightsim-portmaster.zip`, and the probe into
   `dist/flightsim-probe-portmaster.zip`.
4. **Checks** (`port/check_package.sh`, run on every zip, fail the build): every ELF is aarch64; the highest
   `GLIBC_x.y` among imported symbols in `objdump -T` is <= 2.31; `DT_NEEDED` is limited to libc, libm,
   libdl, libpthread, librt, ld-linux-aarch64, `libSDL2-2.0.so.0` and libraries shipped in the zip; no
   libstdc++/libgcc_s dependency; launch script has the PortMaster template calls and does not start gptokeyb
   with `-c`; `port.json` and `gameinfo.xml` are consistent with the script and folder names; licence files
   are present.
5. **Smoke test**: the aarch64 probe runs its JSBSim benchmark (`--cpu-only --quick`) under `qemu-aarch64`.

## Known caveat: libJSBSim needs glibc 2.29

`libJSBSim.so` imports `exp`, `log` and `pow` at `GLIBC_2.29` (the new, faster aarch64 versions), so the shipped
library needs a device glibc >= 2.29 although the main binary only needs 2.17. This is within the 2.31 floor of
decision D15. If the probe reports an older glibc, pin those three symbols to `GLIBC_2.17` with `.symver`
(force-included header when compiling JSBSim).
