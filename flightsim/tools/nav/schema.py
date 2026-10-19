"""Column lists and enumerations of the nav database v1 (DATA.md section 5). Single source of
truth shared by the builder, the validator and the tests."""
from __future__ import annotations

FORMAT_NAME = "flightsim-nav"
FORMAT_VERSION = 1

AIRPORTS_COLS = ["ident", "type", "name", "lat_deg", "lon_deg", "elev_ft", "iso_country",
                 "iso_region", "municipality", "iata", "has_ils"]
RUNWAYS_COLS = ["airport_ident", "rwy_ident", "opposite_ident", "thr_lat_deg", "thr_lon_deg",
                "thr_elev_ft", "start_lat_deg", "start_lon_deg", "end_lat_deg", "end_lon_deg",
                "end_elev_ft", "heading_true_deg", "length_ft", "width_ft", "displaced_ft",
                "surface", "hard", "lighted", "approx"]
NAVAIDS_COLS = ["ident", "type", "name", "freq_khz", "lat_deg", "lon_deg", "elev_ft", "magvar_deg",
                "range_nm", "dme_lat_deg", "dme_lon_deg", "iso_country"]
ILS_COLS = ["airport_ident", "rwy_ident", "loc_ident", "freq_mhz", "loc_lat_deg", "loc_lon_deg",
            "loc_course_true_deg", "loc_width_deg", "has_gs", "gs_lat_deg", "gs_lon_deg",
            "gs_elev_ft", "gs_angle_deg", "tch_ft", "dme_lat_deg", "dme_lon_deg", "category"]
FIXES_COLS = ["ident", "region", "lat_deg", "lon_deg"]
AIRWAYS_COLS = ["airway", "seq", "fix_ident", "fix_lat_deg", "fix_lon_deg", "mea_ft"]

AIRPORT_TYPES = ("large", "medium", "small", "seaplane")
SURFACES = ("asphalt", "concrete", "grass", "dirt", "gravel", "water", "other")
NAVAID_TYPES = ("VOR", "VORDME", "VORTAC", "TACAN", "NDB", "NDBDME", "DME")

TABLES = {
    "airports.csv": AIRPORTS_COLS,
    "runways.csv": RUNWAYS_COLS,
    "navaids.csv": NAVAIDS_COLS,
    "ils.csv": ILS_COLS,
    "fixes.csv": FIXES_COLS,
    "airways.csv": AIRWAYS_COLS,
}
