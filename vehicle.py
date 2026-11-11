"""
vehicle.py - the measured properties of THIS machine and mast, in one place.

Every program that turns a GNSS fix into "where the vehicle is" imports from
here. Before this file existed, amiga_nav.py and rover_log.py each carried
their own ANTENNA_OFFSET_M (0.65 and 0.794), and rover_log.py never applied
HEADING_OFFSET_DEG at all - so marks were shifted the wrong way along the
heading and the controller then drove to them with a different offset again.
One definition, one function, one test.

Re-measure every number here on the Dallas machine. See docs/calibration.md.
"""

import math

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

EARTH_R = 6371000.0


# ============================================================================
#  Conversions
# ============================================================================

def true_heading(raw_deg):
    """Receiver heading (PQTMTAR field 8) -> vehicle heading, deg from north
    clockwise. The ONLY place HEADING_OFFSET_DEG is applied."""
    if raw_deg is None:
        return None
    return (float(raw_deg) + HEADING_OFFSET_DEG) % 360.0


def rotation_centre(lat, lon, heading_deg, offset_m=None):
    """Antenna fix + TRUE vehicle heading -> lat/lon of the rotation centre.

    The antenna is ahead of the rotation centre, so the centre is found by
    stepping BACK along the heading. Pass the true heading (true_heading()),
    never the raw receiver value - the raw value points backwards and puts
    the centre in front of the antenna instead of behind it."""
    if lat is None or lon is None or heading_deg is None:
        return None, None
    L = ANTENNA_OFFSET_M if offset_m is None else offset_m
    th = math.radians(heading_deg)
    dn = -L * math.cos(th)                      # metres north
    de = -L * math.sin(th)                      # metres east
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
