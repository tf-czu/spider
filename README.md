# Spider

## List of code files:

* `localization.py`:
  - Contains a class `Localization(Node)` that fuses GPS and odometry+IMU to
    compute the trajectory of the robot.
  - Can choose one of the three algorithms available:
    * Alignment of odometry+IMU according to GPS utilizing the Procrustes
      Analysis (rotating and scaling the measured positions to meet the least
      squares criterion)
    * Kalman filter with only GPS on its input
    * Kalman filter with GPS, odometry, and IMU on its input

* `lib/tracker.py`:
  - Contains an abstract class `Tracker(ABC)` which describes the interface
    to compute trajectory from asynchronously provided GPS, odometry, and
    IMU.

* `lib/tracker_lsqr.py`:
  - Contains a class `TrackerLeastSquares(Tracker)` which computes the
    trajectory from asynchronously provided GPS, odometry, and IMU
    utilizing the Procrustes Analysis.

* `lib/tracker_kalman_gps_only.py`:
  - Contains a class `TrackerKalmanGPSOnly(Tracker)` which computes the
    trajectory from GPS only utilizing a Kalman filter.

* `lib/tracker_kalman.py`:
  - Contains a class `TrackerKalman(Tracker)` which computes the
    trajectory from asynchronously provided GPS, odometry, and IMU
    utilizing a Kalman filter.

* `lib/anglescaleestimator.py`:
  - Contains a class `AngleScaleEstimator` which takes positions from two
    sources (gps and odometry+IMU) and estimates rotation and scale
    between them.
  - Utilized by: `lib/tracker_kalman.py`

* `lib/speedometer.py`:
  - Contains classes `Speedometer` and `ConverterOdoIMUtoXYZ` utilized by
    `lib/tracker_kalman.py`.

* `lib/kalman.py`:
  - Contains a class `KalmanFilterLocalization` which implements a Kalman
    filter for 3D positions involving velocity and acceleration.
  - Utilized by: `lib/tracker_kalman.py`, `lib/tracker_kalman_gps_only.py`

* `lib/rmsd.py`:
  - Contains a `class RootMeanSquareDeviationCounter` which computes root
    mean square deviation from two asynchronously provided series of
    `[x,y]` or `[x,y,z]` positions.
  - Utilized by `localization.py` to computed RMSD between the resulting
    trajectory and a precise RTK-GPS which serves as a reference.
