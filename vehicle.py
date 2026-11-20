"""
vehicle.py - the measured properties of THIS machine and mast, in one place.

Every program that turns a GNSS fix into "where the vehicle is" imports from
here. Before this file existed, amiga_nav.py and rover_log.py each carried
their own ANTENNA_OFFSET_M (0.65 and 0.794), and rover_log.py never applied
HEADING_OFFSET_DEG at all - so marks were shifted the wrong way along the
heading and the controller then drove to them with a different offset again.
One definition, one function, one test.

Re-measure every number here on the Dallas machine. See docs/calibration.md.

MACHINE PROFILE. The numbers below are the College Station machine's. A
different machine (or the same GNSS kit moved to another Amiga) gets its own
measured numbers in a profile file, written by `commission.py` - no code
edits:

    ~/amiga_vehicle.json          (or the path in $AMIGA_PROFILE)

    {"machine": "dallas", "measured": "2026-11-25",
     "heading_offset_deg": 179.4, "antenna_offset_m": 0.62,
     "antenna_lateral_m": 0.01,
     "nav": {"CREEP_SPEED_MM_S": 30, "ARC_RADIUS_M": 1.8, ...}}

"nav" values override the same-named constants in amiga_nav.py. Set
AMIGA_PROFILE=none to ignore any profile (the tests do).
"""

import json
import math
import os

# ============================================================================
#  Calibration - properties of the machine and mast, not of the software
# ============================================================================

# The LG580P reports heading 180 deg from the direction of travel with the
# mast wired Antenna-1 forward, Antenna-2 rear. Measured 2026-10-27 against
# course-over-ground (circular mean -177.6 deg, sd 5.5 deg, 1839 samples) and
# confirmed at a mark pointing at the next mark (raw 317.3 vs bearing 137.5).
# Set to 0.0 if the antenna cables are ever swapped.
HEADING_OFFSET_DEG = 180.0

# Distance from the vehicle's centre of rotation FORWARD to the GNSS antenna
# (the position GGA reports), along the vehicle axis. Measured 2026-10-27 with `amiga_nav.py calib-offset`:
# 0.794 and 0.813 m. Average several readings when re-measuring.
ANTENNA_OFFSET_M = 0.80

# Distance from the centreline to the antenna, LEFT positive. Zero with the
# mast front-to-back on the centreline (College Station). A mast mounted
# left-right puts the antenna to one side; commission.py measures it.
ANTENNA_LATERAL_M = 0.0

EARTH_R = 6371000.0


# ============================================================================
#  Machine profile
# ============================================================================

PROFILE_ENV = "AMIGA_PROFILE"
DEFAULT_PROFILE = os.path.expanduser("~/amiga_vehicle.json")
PROFILE = {}            # the loaded file, {} if none
PROFILE_PATH = None     # where it came from
NAV_OVERRIDES = {}      # amiga_nav constants to replace


def profile_path():
    p = os.environ.get(PROFILE_ENV)
    if p is not None:
        return None if p.strip().lower() in ("", "none", "off") else \
            os.path.expanduser(p)
    return DEFAULT_PROFILE


def load_profile(path=None):
    """Apply a machine profile to this module. Returns the dict, or {}."""
    global PROFILE, PROFILE_PATH, NAV_OVERRIDES
    global HEADING_OFFSET_DEG, ANTENNA_OFFSET_M, ANTENNA_LATERAL_M
    path = path if path is not None else profile_path()
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as fh:
        prof = json.load(fh)
    if "heading_offset_deg" in prof:
        HEADING_OFFSET_DEG = float(prof["heading_offset_deg"])
    if "antenna_offset_m" in prof:
        ANTENNA_OFFSET_M = float(prof["antenna_offset_m"])
    if "antenna_lateral_m" in prof:
        ANTENNA_LATERAL_M = float(prof["antenna_lateral_m"])
    PROFILE, PROFILE_PATH = prof, path
    NAV_OVERRIDES = dict(prof.get("nav") or {})
    return prof


def describe():
    """One line for the top of a run log."""
    if not PROFILE:
        return (f"machine: built-in College Station values (no profile file; "
                f"commission.py writes {DEFAULT_PROFILE})")
    return (f"machine: {PROFILE.get('machine', '?')} profile {PROFILE_PATH} "
            f"(measured {PROFILE.get('measured', '?')}): heading offset "
            f"{HEADING_OFFSET_DEG:.1f} deg, antenna {ANTENNA_OFFSET_M:.3f} m "
            f"forward, {ANTENNA_LATERAL_M:+.3f} m left")


load_profile()


# ============================================================================
#  Conversions
# ============================================================================

def true_heading(raw_deg):
    """Receiver heading (PQTMTAR field 8) -> vehicle heading, deg from north
    clockwise. The ONLY place HEADING_OFFSET_DEG is applied."""
    if raw_deg is None:
        return None
    return (float(raw_deg) + HEADING_OFFSET_DEG) % 360.0


def centre_shift(heading_deg, offset_m=None, lateral_m=None):
    """(east, north) metres from the ANTENNA to the rotation centre, for a
    TRUE vehicle heading. The one place the antenna geometry is applied:
    rotation_centre() here and amiga_nav.control_point() both use it."""
    L = ANTENNA_OFFSET_M if offset_m is None else offset_m
    S = ANTENNA_LATERAL_M if lateral_m is None else lateral_m
    th = math.radians(heading_deg)
    # forward unit (sin, cos); left unit (-cos, sin). centre = antenna -
    # L * forward - S * left.
    return (-L * math.sin(th) + S * math.cos(th),
            -L * math.cos(th) - S * math.sin(th))


def rotation_centre(lat, lon, heading_deg, offset_m=None, lateral_m=None):
    """Antenna fix + TRUE vehicle heading -> lat/lon of the rotation centre.

    The antenna is ahead of the rotation centre, so the centre is found by
    stepping BACK along the heading. Pass the true heading (true_heading()),
    never the raw receiver value - the raw value points backwards and puts
    the centre in front of the antenna instead of behind it."""
    if lat is None or lon is None or heading_deg is None:
        return None, None
    de, dn = centre_shift(heading_deg, offset_m, lateral_m)
    clat = lat + math.degrees(dn / EARTH_R)
    clon = lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))
    return clat, clon


def circular_mean_deg(values):
    """Mean of angles in degrees, safe across the 359/0 wrap.
    Returns (mean_deg, circular_sd_deg) or (None, None) if empty."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None
    s = sum(math.sin(math.radians(v)) for v in vals) / len(vals)
    c = sum(math.cos(math.radians(v)) for v in vals) / len(vals)
    mean = math.degrees(math.atan2(s, c)) % 360.0
    R = min(1.0, math.hypot(s, c))
    sd = math.degrees(math.sqrt(-2.0 * math.log(R))) if R > 0 else 180.0
    return mean, sd


def metres_between(lat1, lon1, lat2, lon2):
    """Small-distance separation in metres, local tangent plane."""
    k = math.cos(math.radians((lat1 + lat2) / 2.0))
    dx = math.radians(lon2 - lon1) * EARTH_R * k
    dy = math.radians(lat2 - lat1) * EARTH_R
    return math.hypot(dx, dy)
