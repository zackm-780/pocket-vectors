#!/usr/bin/env python3
"""Build one zip that unzips into the ports folder (Roms/PORTS).

Python 3.8+, standard library only. The zip's top level is FlightSim.sh and flightsim/.
A data pack is required: the result must contain flightsim/data/terrain and
flightsim/data/nav/nav.ini. Optional .fms files are copied to flightsim/data/plans/.

Example:
  python3 make_sdcard_zip.py --game game.zip --data pack.zip --plan trip.fms --out ports.zip
"""

import argparse
import os
import shutil
import stat
import sys
import tempfile
import zipfile


def fail(message):
    print("make_sdcard_zip: " + message, file=sys.stderr)
    return 2


def is_zip_symlink(info):
    mode = info.external_attr >> 16
    return stat.S_ISLNK(mode)


def safe_parts(name):
    name = name.replace("\\", "/").lstrip("/")
    parts = [p for p in name.split("/") if p not in ("", ".")]
    if any(p == ".." for p in parts):
        raise ValueError("unsafe path " + name)
    return parts


def extract_zip(path, dest):
    with zipfile.ZipFile(path) as zf:
        for info in zf.infolist():
            if info.is_dir():
                continue
            if is_zip_symlink(info):
                raise ValueError("symlink in zip: " + info.filename)
            parts = safe_parts(info.filename)
            if not parts:
                continue
            target = os.path.join(dest, *parts)
            parent = os.path.dirname(target)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with zf.open(info, "r") as src, open(target, "wb") as out:
                shutil.copyfileobj(src, out)


def materialize(path, dest):
    if os.path.isdir(path):
        # Copy so later steps can reject symlinks without touching the source.
        for dirpath, dirnames, filenames in os.walk(path, followlinks=False):
            rel = os.path.relpath(dirpath, path)
            for name in dirnames + filenames:
                src = os.path.join(dirpath, name)
                if os.path.islink(src):
                    raise ValueError("symlink: " + os.path.join(rel, name))
            target_dir = dest if rel == "." else os.path.join(dest, rel)
            os.makedirs(target_dir, exist_ok=True)
            for name in filenames:
                src = os.path.join(dirpath, name)
                if os.path.islink(src):
                    raise ValueError("symlink: " + os.path.join(rel, name))
                shutil.copyfile(src, os.path.join(target_dir, name))
        return
    if zipfile.is_zipfile(path):
        extract_zip(path, dest)
        return
    raise ValueError("not a zip or folder: " + path)


def find_game(root):
    found = None
    for dirpath, dirnames, filenames in os.walk(root):
        if "FlightSim.sh" not in filenames:
            continue
        port = os.path.join(dirpath, "flightsim")
        binary = os.path.join(port, "flightsim.aarch64")
        if os.path.isdir(port) and os.path.isfile(binary) and not os.path.islink(binary):
            found = dirpath
            break
    if found is None:
        raise ValueError("game has no FlightSim.sh next to flightsim/flightsim.aarch64")
    return found


def find_data(root):
    for dirpath, dirnames, filenames in os.walk(root):
        if os.path.basename(dirpath) != "data":
            continue
        if os.path.isdir(os.path.join(dirpath, "terrain")) or os.path.isfile(
            os.path.join(dirpath, "nav", "nav.ini")
        ):
            return dirpath
    if os.path.isdir(os.path.join(root, "terrain")) or os.path.isfile(
        os.path.join(root, "nav", "nav.ini")
    ):
        return root
    raise ValueError("data pack has no data/terrain or data/nav")


def copy_tree(src, dst):
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target_dir = dst if rel == "." else os.path.join(dst, rel)
        os.makedirs(target_dir, exist_ok=True)
        for name in filenames:
            source = os.path.join(dirpath, name)
            if os.path.islink(source) or os.path.islink(dirpath):
                raise ValueError("symlink: " + os.path.join(rel, name))
            target = os.path.join(target_dir, name)
            if name.endswith(".sh"):
                with open(source, "rb") as inp:
                    data = inp.read().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
                with open(target, "wb") as out:
                    out.write(data)
            else:
                shutil.copyfile(source, target)


def validate(stage):
    script = os.path.join(stage, "FlightSim.sh")
    port = os.path.join(stage, "flightsim")
    binary = os.path.join(port, "flightsim.aarch64")
    lib = os.path.join(port, "libs.aarch64", "libJSBSim.so.1")
    terrain = os.path.join(port, "data", "terrain")
    nav = os.path.join(port, "data", "nav", "nav.ini")
    if not os.path.isfile(script) or os.path.islink(script):
        raise ValueError("FlightSim.sh is missing")
    if not os.path.isfile(binary) or os.path.islink(binary) or os.path.getsize(binary) == 0:
        raise ValueError("flightsim/flightsim.aarch64 is missing or empty")
    if not os.path.isfile(lib) or os.path.islink(lib) or os.path.getsize(lib) == 0:
        raise ValueError("libs.aarch64/libJSBSim.so.1 must be a regular file")
    if not os.path.isdir(terrain):
        raise ValueError("data/terrain is missing")
    if not os.path.isfile(nav):
        raise ValueError("data/nav/nav.ini is missing")
    for dirpath, dirnames, filenames in os.walk(port):
        for name in dirnames + filenames:
            path = os.path.join(dirpath, name)
            if os.path.islink(path):
                raise ValueError("symlink in the package: " + path)


def write_zip(stage, out_path):
    parent = os.path.dirname(os.path.abspath(out_path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with zipfile.ZipFile(out_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for dirpath, dirnames, filenames in os.walk(stage):
            for name in filenames:
                full = os.path.join(dirpath, name)
                if os.path.islink(full):
                    raise ValueError("symlink: " + full)
                arc = os.path.relpath(full, stage).replace(os.sep, "/")
                zf.write(full, arc)


def build(game, data_packs, plans, out_path):
    with tempfile.TemporaryDirectory(prefix="fsim-sdcard-") as tmp:
        game_src = os.path.join(tmp, "game")
        os.makedirs(game_src)
        materialize(game, game_src)
        game_root = find_game(game_src)
        stage = os.path.join(tmp, "stage")
        os.makedirs(stage)
        shutil.copyfile(os.path.join(game_root, "FlightSim.sh"), os.path.join(stage, "FlightSim.sh"))
        with open(os.path.join(stage, "FlightSim.sh"), "rb") as inp:
            text = inp.read().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        with open(os.path.join(stage, "FlightSim.sh"), "wb") as out:
            out.write(text)
        copy_tree(os.path.join(game_root, "flightsim"), os.path.join(stage, "flightsim"))
        for index, pack in enumerate(data_packs):
            pack_src = os.path.join(tmp, "pack-%d" % index)
            os.makedirs(pack_src)
            materialize(pack, pack_src)
            data_root = find_data(pack_src)
            copy_tree(data_root, os.path.join(stage, "flightsim", "data"))
        if plans:
            plan_dir = os.path.join(stage, "flightsim", "data", "plans")
            os.makedirs(plan_dir, exist_ok=True)
            for plan in plans:
                if not plan.lower().endswith(".fms"):
                    raise ValueError("plan is not a .fms file: " + plan)
                if not os.path.isfile(plan):
                    raise ValueError("plan not found: " + plan)
                shutil.copyfile(plan, os.path.join(plan_dir, os.path.basename(plan)))
        validate(stage)
        write_zip(stage, out_path)


def main(argv):
    parser = argparse.ArgumentParser(description="Assemble a Roms/PORTS zip for Pocket Vectors.")
    parser.add_argument("--game", required=True, help="game zip or folder (FlightSim.sh + flightsim/)")
    parser.add_argument(
        "--data",
        action="append",
        required=True,
        help="data pack zip or folder (repeat to merge; later packs win)",
    )
    parser.add_argument("--plan", action="append", default=[], help="a .fms file for flightsim/data/plans/")
    parser.add_argument("--out", required=True, help="zip to write")
    args = parser.parse_args(argv)
    try:
        build(args.game, args.data, args.plan, args.out)
    except (ValueError, OSError, zipfile.BadZipFile) as exc:
        return fail(str(exc))
    print("wrote " + args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
