#!/usr/bin/env python3
"""Tiny-fixture tests for make_sdcard_zip.py. Python 3.8+, standard library only."""

import os
import stat
import subprocess
import sys
import tempfile
import unittest
import zipfile

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "make_sdcard_zip.py")


def write(path, body):
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "wb") as out:
        out.write(body if isinstance(body, bytes) else body.encode("utf-8"))


def make_game(root, script=b"#!/bin/bash\necho hi\r\n"):
    write(os.path.join(root, "FlightSim.sh"), script)
    write(os.path.join(root, "flightsim", "flightsim.aarch64"), b"elf-stub")
    write(os.path.join(root, "flightsim", "libs.aarch64", "libJSBSim.so.1"), b"lib-stub")
    write(os.path.join(root, "flightsim", "port.json"), b"{}\n")


def make_pack(root, marker, extra=None):
    write(os.path.join(root, "data", "terrain", "pack.ini"), b"levels = 5\n")
    write(os.path.join(root, "data", "nav", "nav.ini"), b"airports = airports.csv\n")
    write(os.path.join(root, "data", "ATTRIBUTION.txt"), marker)
    if extra:
        write(os.path.join(root, "data", extra), b"from-pack\n")


def zip_dir(src, dest):
    with zipfile.ZipFile(dest, "w") as zf:
        for dirpath, _, filenames in os.walk(src):
            for name in filenames:
                full = os.path.join(dirpath, name)
                arc = os.path.relpath(full, src).replace(os.sep, "/")
                zf.write(full, arc)


class MakeSdcardZipTest(unittest.TestCase):
    def run_script(self, args):
        return subprocess.run(
            [sys.executable, SCRIPT] + args, capture_output=True, text=True
        )

    def test_merges_packs_plans_and_normalizes_script(self):
        with tempfile.TemporaryDirectory() as tmp:
            game = os.path.join(tmp, "game")
            pack_a = os.path.join(tmp, "a")
            pack_b = os.path.join(tmp, "b")
            make_game(game)
            make_pack(pack_a, b"pack A copernicus\n", "from-a.txt")
            make_pack(pack_b, b"pack B copernicus\n")
            game_zip = os.path.join(tmp, "game.zip")
            # One wrapper folder, the way some download tools unzip.
            wrapped = os.path.join(tmp, "wrapped")
            os.makedirs(os.path.join(wrapped, "PocketVectors"))
            for dirpath, _, filenames in os.walk(game):
                for name in filenames:
                    src = os.path.join(dirpath, name)
                    rel = os.path.relpath(src, game)
                    dest = os.path.join(wrapped, "PocketVectors", rel)
                    os.makedirs(os.path.dirname(dest), exist_ok=True)
                    with open(src, "rb") as inp, open(dest, "wb") as out:
                        out.write(inp.read())
            zip_dir(wrapped, game_zip)
            pack_zip = os.path.join(tmp, "pack-b.zip")
            zip_dir(pack_b, pack_zip)
            plan = os.path.join(tmp, "KPDX-KOTH.fms")
            write(plan, "I\n1100 Version\n")
            out = os.path.join(tmp, "ports.zip")
            result = self.run_script(
                ["--game", game_zip, "--data", pack_a, "--data", pack_zip, "--plan", plan, "--out", out]
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            with zipfile.ZipFile(out) as zf:
                names = zf.namelist()
                self.assertIn("FlightSim.sh", names)
                self.assertIn("flightsim/flightsim.aarch64", names)
                self.assertIn("flightsim/libs.aarch64/libJSBSim.so.1", names)
                self.assertIn("flightsim/data/terrain/pack.ini", names)
                self.assertIn("flightsim/data/nav/nav.ini", names)
                self.assertIn("flightsim/data/from-a.txt", names)
                self.assertIn("flightsim/data/plans/KPDX-KOTH.fms", names)
                self.assertNotIn("FlightSimProbe.sh", names)
                self.assertTrue(all("/" not in n or not n.startswith("/") for n in names))
                self.assertFalse(any(n.startswith("PocketVectors/") for n in names))
                script = zf.read("FlightSim.sh")
                self.assertNotIn(b"\r", script)
                self.assertIn(b"pack B copernicus", zf.read("flightsim/data/ATTRIBUTION.txt"))
                info = zf.getinfo("flightsim/libs.aarch64/libJSBSim.so.1")
                self.assertFalse(stat.S_ISLNK(info.external_attr >> 16))

    def test_symlink_library_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            game = os.path.join(tmp, "game")
            pack = os.path.join(tmp, "pack")
            make_game(game)
            real = os.path.join(game, "flightsim", "libs.aarch64", "libJSBSim.so.1.real")
            os.rename(os.path.join(game, "flightsim", "libs.aarch64", "libJSBSim.so.1"), real)
            os.symlink("libJSBSim.so.1.real", os.path.join(game, "flightsim", "libs.aarch64", "libJSBSim.so.1"))
            make_pack(pack, b"attr\n")
            out = os.path.join(tmp, "ports.zip")
            result = self.run_script(["--game", game, "--data", pack, "--out", out])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("symlink", result.stderr.lower())
            self.assertFalse(os.path.exists(out))

    def test_missing_nav_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            game = os.path.join(tmp, "game")
            pack = os.path.join(tmp, "pack")
            make_game(game)
            write(os.path.join(pack, "data", "terrain", "pack.ini"), b"levels = 5\n")
            out = os.path.join(tmp, "ports.zip")
            result = self.run_script(["--game", game, "--data", pack, "--out", out])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("nav.ini", result.stderr)


if __name__ == "__main__":
    unittest.main()
