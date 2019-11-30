import math
import numpy as np
import pyproj


def getXY(lat, lon, h=0):
    lam = math.pi / 180.0 * lat
    phi = math.pi / 180.0 * lon
    s = math.sin(lam)
    N = a / math.sqrt(1 - e_sq * s * s)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    return [(h + N) * cos_lam * cos_phi, (h + N) * cos_lam * sin_phi]


def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj="geocent", ellps="WGS84", datum="WGS84")
    lla = pyproj.Proj(proj="latlong", ellps="WGS84", datum="WGS84")
    x, y, _ = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)

    return [x, y]


if __name__ == "__main__":
    a = 6378137.0
    f_inv = 298.2572215381
    f = 1.0 / f_inv
    b = a * (1 - f)
    e_sq = (1.0 - b ** 2) / (a ** 2)

    sx, sy = 41.016147, 29.171307
    ex, ey = 41.016043, 29.171708

    x = getXY(sx, sy, 146.0)
    y = getXY(ex, ey, 146.0)

    print(x, y)
    print(np.subtract(x, y))
    print("%.2f meters" % (math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)))

    x = gps_to_ecef_pyproj(sx, sy, 146.0)
    y = gps_to_ecef_pyproj(ex, ey, 146.0)

    print(x, y)
    print(np.subtract(x, y))
    print("%.2f meters" % (math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)))
