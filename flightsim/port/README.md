## Notes

Thanks to the [JSBSim team](https://github.com/JSBSim-Team/jsbsim) for the open-source flight dynamics
model that flies every aircraft here, to the SDL project, and to the OurAirports, Copernicus DEM and FAA
data providers behind the real-world scenery and airports.

Flight Sim is an experimental port. It needs a **data pack** (terrain tiles, airports, navigation data)
that you build on a PC with the project's offline tools. See `flightsim/docs/DATA.md` in the project.
Copy the pack to `flightsim/data/`, or set `data_dir` in `flightsim/conf/settings.ini`.

Run the **Flight Sim Probe** port first on a new device: it measures the device and writes
`probe_report.txt`.

## Controls

The full mapping is in `flightsim/docs/CONTROLS.md` in the project. Exit the game with **Select + Start**.
Remaps are stored in `flightsim/conf/controls.ini`.

## Compile

The device build runs in a Debian 11 (glibc 2.31) container with an aarch64 cross compiler.

    git clone <this repository>
    cd flightsim
    tools/build_device.sh        # writes dist/flightsim-portmaster.zip
