"""Per-sensor range calibration for the six plate sensors.

Applied ONCE, in ``arduino_sensors`` before ``/distance_sensors`` is published,
so every consumer (``wall_parallel_controller``, ScanWall's arm approach, the
whole-body sweep's plane fit) sees corrected ranges and none of them carries
its own copy of these numbers.

Fitted 2026-09-17 against the UR's forward kinematics: 17 held poses at
13-23 cm, rotations about tool X and Y, and two wall contacts at different
tilts, with the pendant TCP ``Sensor_plate`` (-80, 0, +320 mm — the GPR
contact point) constrained to lie on the wall at both contacts. The fit
closes to +/-0.2 mm at both. Sensor gains all came out 0.95-1.05, so a
constant offset per sensor is the whole correction. The numbers are the
published value minus the true range, in metres, in ``/distance_sensors``
order ``[U1, U2, U3, S1, S2, S3]``.

It is not a per-type offset. S1 and S3 are exact; U2 reads 4.4 cm short and
S2 3.6 cm long. Left uncorrected, that pattern happens to CANCEL real plate
tilt in the plane fit: the raw fit read the plate at 2.6 deg when the arm
said 7.5, and 5.2 deg when the arm said 2.7 — wrong, and sometimes in the
wrong direction. Every wall sweep before this date aligned the plate to that
biased plane, so it was sweeping ~6 deg off the wall; the 2026-09-15 31 N
overload was a plate the controller read as 1 deg off arriving 7.7 deg off,
corner first. Corrected, the fitted tilt tracks the arm to ~0.6 deg RMS.

Residual scatter after correction — ultrasonics 4-6 mm, ToF 1.4-1.7 mm — is
what the consumers' per-sensor sigma should reflect.

Redo the fit (same procedure, any held poses plus two contacts) if a sensor
is moved, re-mounted or replaced; the ToF ``+0.083`` in the reader is a
separate, older mounting-depth constant and stays.
"""

# published minus true, metres, [U1, U2, U3, S1, S2, S3]
RANGE_OFFSET_M = (-0.0106, -0.0444, -0.0139, -0.0004, 0.0357, 0.0006)

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
