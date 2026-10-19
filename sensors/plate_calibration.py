"""Per-sensor range calibration for the six plate sensors.

Applied ONCE, in ``arduino_sensors`` before ``/distance_sensors`` is published,
so every consumer (``wall_parallel_controller``, ScanWall's arm approach, the
whole-body sweep's plane fit) sees corrected ranges and none of them carries
its own copy of these numbers.

Measured 2026-09-21 with the plate PARALLEL to a flat wall — all four GPR
casters touching — and the readings held for 10 frames:

    U1 11 cm  U2 12 cm  U3 12 cm  S1 54.4 mm  S2 60.8 mm  S3 56.5 mm

In that pose the six true ranges are equal. The ToF faces sit 0.083 m in
front of the plate (the reader adds it), so the three ToF put the sensor
plane at 137.4 / 143.8 / 139.5 mm; the datum is their median, 139.5 mm, and
each offset is that sensor's reading minus the datum. The ultrasonics report
whole centimetres, so their offsets carry +/- 5 mm of quantisation — the
consumers' per-sensor sigma (6 mm ultrasonic, 2 mm ToF) covers that.

This REPLACES the 2026-09-17 fit against the arm's forward kinematics
(U2 -4.4 cm, S2 +3.6 cm, the rest near zero). That fit solved for the wall
plane's orientation and the six offsets together, and the two are not
separable: a constant tilt between the assumed wall and the real one is
exactly what a diagonal offset pattern produces. It settled on a wall 6.2 deg
from the true one and offsets that made every pose read 6.2 deg off — the
17 poses fit the FK's RELATIVE motion equally well with either set (0.20 vs
0.22 deg pairwise), which is the tell. The consequence, from 09-17 to 09-21,
was a controller that "squared" the plate to a wall 6 deg from the real one
and put a corner (the top-right caster) into it first on every contact. The
parallel pose is a direct measurement with no free wall parameter, which is
why it wins. It also moves the sensor-plane distance at contact from the
15.0 cm the old fit reported to 14.0 cm.

Residual scatter is unchanged — ultrasonics 4-6 mm, ToF 1.4-1.7 mm.

Redo this (plate square on a flat wall, ten frames, offsets = reading minus
the ToF median) if a sensor is moved, re-mounted or replaced; the ToF
``+0.083`` in the reader is a separate mounting-depth constant and stays.
"""

# published minus true, metres, [U1, U2, U3, S1, S2, S3]
RANGE_OFFSET_M = (-0.0285, -0.0195, -0.0195, -0.0021, 0.0043, 0.0000)

# Raw values the firmware uses to mean "no reading": ultrasonic 0 cm, ToF 255 mm.
# They must pass through untouched or the consumers' validity windows stop
# recognising them.
ULTRASONIC_INVALID_CM = 0
TOF_INVALID_MM = 255


def apply_range_calibration(published, raw_ultra_cm, raw_tof_mm):
    """Subtract the offsets from the six published ranges, sentinels excepted.

    ``published`` is the list the node is about to put on ``/distance_sensors``;
    ``raw_ultra_cm`` / ``raw_tof_mm`` are the three raw values each was built
    from, used only to recognise the invalid sentinels.
    """
    out = list(published)
    for i in range(3):
        if raw_ultra_cm[i] != ULTRASONIC_INVALID_CM:
            out[i] = published[i] - RANGE_OFFSET_M[i]
    for i in range(3):
        if raw_tof_mm[i] != TOF_INVALID_MM:
            out[3 + i] = published[3 + i] - RANGE_OFFSET_M[3 + i]
    return out
