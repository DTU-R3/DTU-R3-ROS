#!/usr/bin/env python 
 
from pyproj import Proj 
 
projection = Proj(proj="utm", zone="34", ellps='WGS84') 

"""
x = 1 
y = 0  
lon,lat = projection(x, y, inverse=True) 
print lon 
print lat
"""

""" test transformation """
lons = [12.4002731084075, 12.4002711407333, 12.4002672053818, 12.4002632700258, 12.4002534316166]
lats = [55.7303578390348, 55.7303667232865, 55.7303844917896, 55.7304022602925, 55.7304466815488]

i = 0
for l in lons:
  x,y = projection(lons[i],lats[i])
  print "Position"
  print "X: " + str(x)
  print "Y: " + str(y)
  i = i + 1
 
 

