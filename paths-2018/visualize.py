#!/usr/bin/env python3

import sys
import matplotlib.pyplot as plt

def to_bool(s):
    return s.lower().strip() in ['true', 't', 'tr','tru','y', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']

def main():
    file = open(sys.argv[1], 'r')
    lines = file.readlines()
    lines = lines[1:-1]
    data = [line.split(",") for line in lines]
    xy_pts = [(float(datum[0]), float(datum[1])) for datum in data if not to_bool(datum[3])]
    xy_interp = [(float(datum[0]), float(datum[1])) for datum in data if to_bool(datum[3])]

    x_dat = [pt[0] for pt in xy_pts]
    y_dat = [pt[1] for pt in xy_pts]
    x_interp = [pt[0] for pt in xy_interp]
    y_interp = [pt[1] for pt in xy_interp]


    ROBOT_WIDTH = 2.83333333

    IMAGE_HEIGHT_FT = 27.0
    IMAGE_WIDTH_FT = 54.0
    image = plt.imread("fieldCropped.png")
    fig, ax = plt.subplots()
    ax.imshow(image, extent=[0, IMAGE_WIDTH_FT, IMAGE_HEIGHT_FT/2.0, -IMAGE_HEIGHT_FT/2.0])

    # the path
    ax.plot(y_dat, x_dat, '.', linewidth=1, color='firebrick')
    ax.plot(y_interp, x_interp, '.', linewidth=1, color='green')

    # safety lines
    ax.plot([i / 5 for i in range(200)], [-IMAGE_HEIGHT_FT/2.0 + ROBOT_WIDTH/2 for i in range(200)])
    ax.plot([i / 5 for i in range(200)], [IMAGE_HEIGHT_FT/2.0 - ROBOT_WIDTH/2 for i in range(200)])
    plt.show()


if __name__ == "__main__":
    main()
