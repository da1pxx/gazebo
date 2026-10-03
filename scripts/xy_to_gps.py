#!/usr/bin/env python3
"""Convert world coordinates (x = East, y = North, metres) to WGS84 lat/lon.

Uses the local-tangent (ENU) approximation around a datum, which is exact to
millimetres over the size of the simulation world. The datum must match
<spherical_coordinates> in the world file and `datum` in config/localization.yaml.

Example:
    ros2 run gps_nav_robot xy_to_gps.py -- -3,3.5 3,-3.5 9,0
"""
import argparse
import math

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def xy_to_ll(x, y, lat0, lon0):
    phi = math.radians(lat0)
    w = 1.0 - WGS84_E2 * math.sin(phi) ** 2
    meridian_r = WGS84_A * (1.0 - WGS84_E2) / w ** 1.5
    normal_r = WGS84_A / math.sqrt(w)
    lat = lat0 + math.degrees(y / meridian_r)
    lon = lon0 + math.degrees(x / (normal_r * math.cos(phi)))
    return lat, lon


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('points', nargs='+', help='x,y pairs in metres')
    parser.add_argument('--lat0', type=float, default=55.751244)
    parser.add_argument('--lon0', type=float, default=39.0)
    args = parser.parse_args()

    lats, lons = [], []
    for p in args.points:
        x, y = (float(v) for v in p.split(','))
        lat, lon = xy_to_ll(x, y, args.lat0, args.lon0)
        lats.append(lat)
        lons.append(lon)
        print(f'({x:7.2f}, {y:7.2f}) -> lat {lat:.8f}, lon {lon:.8f}')

    print('\n# paste into config/waypoints.yaml')
    print('    latitudes:  [' + ', '.join(f'{v:.8f}' for v in lats) + ']')
    print('    longitudes: [' + ', '.join(f'{v:.8f}' for v in lons) + ']')


if __name__ == '__main__':
    main()
