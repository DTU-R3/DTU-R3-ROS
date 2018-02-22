#!/usr/bin/env python

from pyproj import Proj

projection = Proj(proj="utm", zone="34", ellps='WGS84')

x = 100
y = 5002

lon,lat = projection(x, y, inverse=True)

print lon
print lat
