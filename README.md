# Point to stars

Use this script to make PR2 point with its right arm
to different space stuff (moon, sun, pluto...).

Crappy video showcasing it (using Google Home to trigger it):

[![Video of the tool working on PR2](http://img.youtube.com/vi/KJ-je-zJe_M/0.jpg)](http://youtu.be/KJ-je-zJe_M)



You'll need to point PR2 towards mars and give that mars deviation.

We have no compass in the robot so we can't know where he is facing initially.

Using a map created with the laser, being localized in it, and
giving an orientation to the map, we could overpass this.


Usage:

```
rosrun point_space point_to_stars.py mars 274.05
[INFO] [WallTime: 1480587985.167889] Initializing PointToStars
[INFO] [WallTime: 1480587985.169439] Initalizing GoToPose...
[INFO] [WallTime: 1480587985.171256] Waiting for /execute_kinematic_path service...
[INFO] [WallTime: 1480587985.301177] Connected!
[INFO] [WallTime: 1480587985.303557] mars is at altitude: 0.609291017056 radians  azimuth: 4.74535179138 radians.
[INFO] [WallTime: 1480587985.304757] mars is at altitude: 34.9098037725 degrees  azimuth: 271.888629951 degrees.
[INFO] [WallTime: 1480587985.305576] Correcting azimuth with: 274.05
[INFO] [WallTime: 1480587985.306535] Corrected azimuth: -2.16137004898

```

To get the initial mars azimuth to correct run it without parameters:

```
osrun point_space point_to_stars.py 
Usage:
scripts/point_to_stars.py planet_name mars_azimuth_correction

Available planets: moon, saturn, mars, venus, sun, uranus, jupiter, neptune, pluto
Current mars azimuth: 267.901302117
Example:
scripts/point_to_stars.py mars

```
