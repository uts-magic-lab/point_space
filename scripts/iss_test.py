#!/usr/bin/env python

import math
import time
from datetime import datetime
# sudo pip install pyephem
import ephem

degrees_per_radian = 180.0 / math.pi

home = ephem.Observer()
home.lon = '-122.63'   # +E
home.lat = '45.56'      # +N
home.elevation = 80  # meters

# Always get the latest ISS TLE data from:
# http://spaceflight.nasa.gov/realdata/sightings/SSapplications/Post/JavaSSOP/orbit/ISS/SVPOST.html
iss = ephem.readtle('ISS',
                    '1 25544U 98067A   16330.51329439  .00016717  00000-0  10270-3 0  9006',
                    '2 25544  51.6408 333.5634 0006155 255.7021 104.3448 15.53701844 30074'
                    )

while True:
    home.date = datetime.utcnow()
    iss.compute(home)
    print('iss: altitude %4.1f deg, azimuth %5.1f deg' %
          (iss.alt * degrees_per_radian, iss.az * degrees_per_radian))
    time.sleep(1.0)
