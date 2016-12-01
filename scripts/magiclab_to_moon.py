#!/usr/bin/env python

import math
import time
from datetime import datetime
# sudo pip install pyephem
import ephem

degrees_per_radian = 180.0 / math.pi

home = ephem.Observer()
# The Magic Lab GPS http://www.maps.ie/coordinates.html
home.lat = '-33.88397'  # +N
home.lon = '151.1969783'  # +E
home.elevation = 80  # meters

# Always get the latest ISS TLE data from:
# http://spaceflight.nasa.gov/realdata/sightings/SSapplications/Post/JavaSSOP/orbit/ISS/SVPOST.html
# iss = ephem.readtle('ISS',
#                     '1 25544U 98067A   16330.51329439  .00016717  00000-0  10270-3 0  9006',
#                     '2 25544  51.6408 333.5634 0006155 255.7021 104.3448 15.53701844 30074'
#                     )

# A few planets, for testing purposes
moon = ephem.Moon()
saturn = ephem.Saturn()
mars = ephem.Mars()
venus = ephem.Venus()
sun = ephem.Sun()
uranus = ephem.Uranus()
jupiter = ephem.Jupiter()
neptune = ephem.Neptune()
pluto = ephem.Pluto()
pname = ['moon', 'saturn', 'mars', 'venus',
         'sun', 'uranus', 'jupiter', 'neptune', 'pluto']
planets = [moon, saturn, mars, venus, sun, uranus, jupiter, neptune, pluto]

while True:
    home.date = datetime.utcnow()
    for planet, name in zip(planets, pname):
        planet.compute(home)
        print(name + ':\n  altitude %4.1f deg, azimuth %5.1f deg' %
              (planet.alt * degrees_per_radian, planet.az * degrees_per_radian))
        print('alt: ' + str(planet.alt) + " az: " + str(planet.az))
        print('type: ' + str(type(planet.alt)))
    print('-------')
    time.sleep(1.0)
