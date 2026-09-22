# Shifting Wall & Slipping Base Challenge

A simulated differential-drive robot navigating a 20-meter corridor designed
to be geometrically degenerate for LiDAR-based localization, flat,
featureless walls that give scan matching almost nothing to constrain
position along the corridor's length. A wall panel partway down the hall
shifts sideways every 15 seconds, and the robot carries a 16-channel 3D
LiDAR and a 6-axis IMU.

The project has three parts: building the environment and robot, injecting
realistic sensor corruption and fusing it back together with an EKF, and
detecting when localization becomes unreliable due to the corridor's
geometry.

## The simulation

The corridor, walls, and shifting panel are generated programmatically
(`corridor_sim`), with the panel toggling position on a repeating 15-second
timer. The robot (`corridor_robot_description`) is a differential-drive
base with the LiDAR and IMU mounted on top. Each piece, driving, the LiDAR
point cloud, the IMU readings, was checked individually against expected
physical values (e.g. LiDAR floor returns matching the expected geometry
from the sensor's mounting height and field of view) before anything was
combined.

## Sensor corruption

A dedicated node (`noise_injector.py`) intercepts the ground-truth
odometry and IMU and applies the required corruption models: a scale-factor
slip on forward velocity in a "dusty patch" partway down the corridor, and a
random-walk gyro bias plus white noise on the IMU. Both were verified
quantitatively rather than just visually. The odometry slip's measured lag
came out to 1.48m against a theoretical 1.5m, and the gyro bias's growth
matched the expected `σ·√t` random-walk scaling when checked against
windowed statistics from a real run.

## Fusion and degeneracy detection

The EKF (`ekf_fusion.py`) tracks `[x, y, θ]`, predicting from the corrupted
odometry and IMU and correcting against point-to-plane ICP scan matching (a
new scan matched against a periodically refreshed keyframe scan, rather
than always the immediately previous one). The predict step was verified in
isolation. With the correction step's influence heavily damped, the fused
estimate tracks raw odometry closely, confirming that half of the pipeline
is solid.

Degeneracy detection comes from the eigenvalues of the ICP information
matrix. The smallest eigenvalue is checked every correction step, and a
`LOCALIZATION_DEGENERACY_WARNING` fires when it drops below a threshold,
tuned from real logged eigenvalue data rather than picked arbitrarily.

## Limitations

The correction step has a real, unresolved bug. When fully trusted, it
produces a biased estimate, most visibly a steady drift in `y` even though
the robot only ever drives straight (`media/trajectory_comparison.png`
shows this directly, side by side against a run with correction heavily
damped). Digging into it: the ICP information matrix's eigenvalues, once
actually measured from real runs, came out far larger than expected.
Hundreds to thousands, rather than near-zero along the corridor's length as
pure theory predicts for a degenerate direction. The eigenvalue is
consistently smallest along the corridor's axis, correctly matching where
constraint should be weakest, but it's not as small as it should be. My
working theory is that the point cloud is dominated by floor points, and
their estimated surface normals are never perfectly vertical. Summed over
thousands of them, this likely injects noise into the information matrix
that dilutes the real signal coming from the walls. I also never filtered
`inf`/`-inf` points out of the raw point cloud before handing it to Open3D,
which is a real gap and a plausible contributor. I looked at the point
cloud directly in RViz and it looked geometrically correct, but that was
just a visual check, not a rigorous look at the normals or correspondence
quality actually driving the math.

The map-lifecycle piece (clearing the "ghost" obstacle left behind when the
panel retracts) also isn't implemented. The right approach would be ray-casting
free-space carving through an occupancy grid, using the fused pose.

Next steps I'd take: filter invalid points from the raw scan before ICP,
filter or downweight floor correspondences specifically when computing the
degeneracy eigenvalues so the wall signal isn't diluted, run a proper
filter-consistency check (like NIS) rather than just eyeballing whether the
covariance grows or shrinks sensibly, and log the raw ICP transform across
a keyframe window to see whether the y-bias scales with time since the last
keyframe. That would confirm or rule out my current theory.

## Deliverables

- This repository. World, robot description, noise node, EKF.
- Demo video (drive, panel toggling, live degeneracy warnings firing).
  Linked via Google Drive in the submission email.
- `media/trajectory_comparison.png`. Ground truth vs. raw odometry vs.
  fused estimate, damped and fully-trusted correction side by side.
- `media/ekf_output_log.txt`. Timestamped covariance matrices and
  degeneracy warnings from the submitted (damped) configuration.
