# SD-card zip

`make_sdcard_zip.py` (Python 3.8+, standard library only) builds one zip whose contents go
straight into the ports folder (`Roms/PORTS` on the RG40XXH stock OS).

```bash
python3 flightsim/tools/sdcard/make_sdcard_zip.py \
  --game pocket-vectors-game.zip \
  --data pdx_oth.zip \
  --plan trip.fms \
  --out pocket-vectors-ports.zip
```

- `--game` is a release zip, a device-workflow zip, or a folder. It must contain `FlightSim.sh`
  next to `flightsim/flightsim.aarch64`. A single wrapper folder is fine. The probe is not copied.
- `--data` is a data-pack zip or folder (`data/terrain` and `data/nav`, or those two at the top).
  Repeat the flag to merge packs. Later packs replace files from earlier ones.
- `--plan` copies a `.fms` file to `flightsim/data/plans/`. Repeat for more than one.

The script checks that `flightsim.aarch64` is present, `libs.aarch64/libJSBSim.so.1` is a regular
file (not a symlink), and `data/terrain` plus `data/nav/nav.ini` exist. `.sh` files are written
with LF line endings. The zip has no top-level wrapper folder.

```bash
python3 flightsim/tools/sdcard/test_make_sdcard_zip.py
```
