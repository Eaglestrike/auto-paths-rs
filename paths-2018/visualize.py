#!/usr/bin/env python3

import sys
import matplotlib.pyplot as plt
from math import sqrt


def to_bool(s):
    return s.lower().strip() in ['true', 't', 'tr', 'tru', 'y', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']


def box(xy, norm_len, axial_len):
    norm2 = norm_len / 2
    axial2 = axial_len / 2
    out = ([], [], [], [])
    for i in range(len(xy)-1):
        v = (xy[i+1][0] - xy[i][0], xy[i+1][1] - xy[i][1])
        norm = sqrt(v[0]**2 + v[1]**2)

        # like in the TNB frame
        t = (v[0] / norm, v[1] / norm)
        n = (-t[1], t[0])  # (-dy, dx)

        out[0].append((xy[i][0] + t[0]*axial2 + n[0]*norm2,
                       xy[i][1] + t[1]*axial2 + n[1]*norm2))
        out[1].append((xy[i][0] + t[0]*axial2 - n[0]*norm2,
                       xy[i][1] + t[1]*axial2 - n[1]*norm2))
        out[2].append((xy[i][0] - t[0]*axial2 + n[0]*norm2,
                       xy[i][1] - t[1]*axial2 + n[1]*norm2))
        out[3].append((xy[i][0] - t[0]*axial2 - n[0]*norm2,
                       xy[i][1] - t[1]*axial2 - n[1]*norm2))
    return out


def main():
    file = open(sys.argv[1], 'r')
    lines = file.readlines()
    lines = lines[1:-1]
    data = [line.split(",") for line in lines]
    xy_pts = [(float(datum[0]), float(datum[1]))
              for datum in data if not to_bool(datum[3])]
    xy_interp = [(float(datum[0]), float(datum[1]))
                 for datum in data if to_bool(datum[3])]

    ROBOT_WIDTH = 2.83333333
    ROBOT_LENGTH = 3.125
    bounds = box(xy_pts, ROBOT_WIDTH, ROBOT_LENGTH)

    x_dat = [pt[0] for pt in xy_pts]
    y_dat = [pt[1] for pt in xy_pts]
    x_interp = [pt[0] for pt in xy_interp]
    y_interp = [pt[1] for pt in xy_interp]

    IMAGE_HEIGHT_FT = 27.0
    IMAGE_WIDTH_FT = 54.0
    image = plt.imread("fieldCropped.png")
    fig, ax = plt.subplots()
    ax.imshow(image, extent=[0, IMAGE_WIDTH_FT,
                             IMAGE_HEIGHT_FT/2.0, -IMAGE_HEIGHT_FT/2.0])

    # the path
    ax.plot(y_dat, x_dat, '.', linewidth=1, color='firebrick', zorder=10)
    ax.plot(y_interp, x_interp, '.', linewidth=1, color='green', zorder=5)

    # safety lines
    ax.plot([i / 5 for i in range(200)], [-IMAGE_HEIGHT_FT /
                                          2.0 + ROBOT_WIDTH/2 for i in range(200)], zorder=0)
    ax.plot([i / 5 for i in range(200)], [IMAGE_HEIGHT_FT /
                                          2.0 - ROBOT_WIDTH/2 for i in range(200)], zorder=0)

    # front corners
    for corner in bounds[:2]:
        x, y = zip(*corner)
        ax.plot(y, x, '.', linewidth=1, color="purple", zorder=2)
    # back corners
    for corner in bounds[2:]:
        x, y = zip(*corner)
        ax.plot(y, x, '.', linewidth=1, color="orange", zorder=1)
    plt.show()


if __name__ == "__main__":
    main()
