#!/usr/bin/env python

import rospy
from go_to_pose import GoToPose
import sys
from math import degrees, radians, pi
from datetime import datetime
# sudo pip install pyephem
import ephem

"""
Make PR2 point to stars.
You need to position itself looking
to mars if you want it to actually point to stars.
Also roslaunch pr2_moveit_config move_group.launch
must be launched.
PR2 has no GPS and compass so he can't know
where he is facing. TODO: Having mapped
the environment use that as compass.

Author: Sammy Pfeiffer
"""


def mars_current_azimuth():
    lab_location = ephem.Observer()
    lab_location.lat = '-33.88397'  # +N
    lab_location.lon = '151.1969783'  # +E
    lab_location.elevation = 80
    lab_location.date = datetime.utcnow()
    mars = ephem.Mars()
    mars.compute(lab_location)
    return mars.az * 180.0 / pi


class PointToStars(object):
    def __init__(self):
        rospy.loginfo("Initializing PointToStars")
        self.initialize()
        self.go_to = GoToPose()

    def initialize(self):
        # The Magic Lab GPS http://www.maps.ie/coordinates.html
        self.lab_location = ephem.Observer()
        self.lab_location.lat = '-33.88397'  # +N
        self.lab_location.lon = '151.1969783'  # +E
        self.lab_location.elevation = 80  # meters, 7th floor
        self.degrees_per_radian = 180.0 / pi
        self.planets = {'moon': ephem.Moon(),
                        'saturn': ephem.Saturn(),
                        'mars': ephem.Mars(),
                        'venus': ephem.Venus(),
                        'sun': ephem.Sun(),
                        'uranus': ephem.Uranus(),
                        'jupiter': ephem.Jupiter(),
                        'neptune': ephem.Neptune(),
                        'pluto': ephem.Pluto()}

    def get_planet_alt_az(self, planet_name):
        self.lab_location.date = datetime.utcnow()
        planet = self.planets.get(planet_name, None)
        if planet is None:
            rospy.logerr("We don't know planet '" + str(planet_name) + "', " +
                         "try any of: " + str(self.planets.keys()))
            return None, None
        planet.compute(self.lab_location)
        return planet.alt * self.degrees_per_radian, planet.az * self.degrees_per_radian

    def point_at(self, alt, az):
        joint_names = ['r_shoulder_pan_joint', 'r_upper_arm_roll_joint',
                       'r_shoulder_lift_joint', 'r_forearm_roll_joint',
                       'r_elbow_flex_joint', 'r_wrist_flex_joint',
                       'r_wrist_roll_joint']
        positions = [-az, 0.0, -alt, 0.0, 0.0, 0.0, 0.0]
        self.go_to.go_to_pose(joint_names, positions, wait_for_execution=True)

    def point_at_planet(self, planet_name, azimuth_correction=None):
        alt, az = self.get_planet_alt_az(planet_name)
        rospy.loginfo(planet_name + " is at altitude: " + str(radians(alt)) +
                      " radians " + " azimuth: " + str(radians(az)) +
                      " radians.")
        rospy.loginfo(planet_name + " is at altitude: " + str(alt) +
                      " degrees " + " azimuth: " + str(az) +
                      " degrees.")

        if alt is not None and az is not None:
            if azimuth_correction is not None:
                rospy.loginfo("Correcting azimuth with: " +
                              str(azimuth_correction))
                az = az - azimuth_correction
                rospy.loginfo("Corrected azimuth: " + str(az))
                if az < -90.0:
                    az = 360 + az
                    rospy.loginfo("Overshooted corrected azimuth: " + str(az))
            self.point_at(radians(alt), radians(az))


if __name__ == '__main__':
    argv = sys.argv
    if "--help" in argv or "-h" in argv or len(argv) < 3:
        print "Usage:"
        print argv[0] + " planet_name mars_azimuth_correction"
        print
        print "Available planets: moon, saturn, mars, venus, sun, uranus, jupiter, neptune, pluto"
        print "Current mars azimuth: " + str(mars_current_azimuth())
        print "Example:"
        print argv[0] + " mars"
        print
        exit(0)
    planet_name = argv[1]
    mars_azimuth_correction = float(argv[2])
    rospy.init_node('point_to_stars')

    pts = PointToStars()
    pts.point_at_planet(planet_name, mars_azimuth_correction)
