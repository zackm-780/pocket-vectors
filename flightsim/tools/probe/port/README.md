## Notes

A measurement tool for the Flight Sim project, not a game. Thanks to the SDL project (the bitmap font
comes from SDL's test library) and the JSBSim team (the CPU benchmark steps their flight model).

Run it once on the handheld and follow the prompts on screen. The whole run takes about two minutes:
system information, an input test (you press the buttons it asks for), a GPU benchmark, a CPU benchmark
and a graphics configuration sweep. Then send back `probe_report.txt` (and `log.txt`) from this
port's folder. The report is rewritten after every step, so it is still useful if you have to quit
early with Select + Start.

## Controls

Do what the screen says. **Do not press Select and Start together** during the input test: that exits.

## Compile

    git clone <this repository>
    cd flightsim
    tools/build_device.sh        # writes dist/flightsim-probe-portmaster.zip
