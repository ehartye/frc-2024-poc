"""Robot-coordinate dimensions for the passive breaker and two cable supports, in mm."""
from math import cos, radians, sin

SHIFT_Y = 12.
BASE_HOLES = [(x,y) for x in (-306.,-244.) for y in (-168.,-146.)]
POSTS = (((-290.,-130.),(-312.,-88.5)),((-222.,-155.),(-199.,-137.)))
ROUTES = ((-308.575,-149.03,105.,-1),(-251.425,-176.97,15.,1))
PLATFORMS = ((-340.,-135.,-284.,-84.),(-230.,-183.,-185.,-131.))
DECK_Z = 205.115
SHELF_Z = 248.825


def xy(route,u,v=0.):
    x,y,angle,_=route
    a=radians(angle)
    return x+u*cos(a)-v*sin(a), y+u*sin(a)+v*cos(a)
