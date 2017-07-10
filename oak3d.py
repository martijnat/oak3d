#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2017  Martijn Terpstra

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from math import sin, cos, pi, sqrt
from random import random
from shutil import get_terminal_size
from sys import stdout, stderr, argv
from time import sleep
from tqdm import tqdm

columns, rows = get_terminal_size((80, 20))

samples_per_pixel = 2
rows = rows * 2 * samples_per_pixel
columns = columns * samples_per_pixel

half_block = "▄"
# escape codes, send to stdout to do stuff
esc_draw_rgb_bg = "\x1b[48;2;%i;%i;%im"
esc_draw_rgb_fg = "\x1b[38;2;%i;%i;%im"
esc_position_cursor = "\033[%i;%iH"
esc_hide_cursor = "\033[?25l"
esc_reset_cursor = "\033[?25h"

max_draw_dist = 99999

color_fog = (.05, .1, .2)
color_base = (1.0,1.0,1.0)
color_var = (0.0,0.0,0.0)
color_default = lambda x, y, z: [b+(v*random()*2)-v for b,v in zip(color_base,color_var)]
color_sun = (1.5, 1.1, 1)
color_countersun = (.1, .1, .1)

gamma_correction = 1/2.2
ambient_light = (0, 0, 0)

angle_sun = (1, -1, -1)
angle_countersun1 = (-1, 0, 0)
angle_countersun2 = (0, 1, 0)
angle_countersun3 = (0, 0, 1)


def main():
    try:
        if len(argv) > 1:
            model = load_obj(argv[1])
        else:
            stderr.write("Usage: %s /path/to/file.obj\n" % argv[0])
            quit(1)

        x, y, z, d = get_camera_values(model)

        global max_draw_dist
        max_draw_dist = d * 3
        # d = 100

        stdout.write(esc_hide_cursor)

        steps = 256
        prerendered_screens = []
        # render the first frames manual
        for step in range(steps):
            u = 0.1 * pi * sin(2 * pi * step / steps)
            v = 2 * pi * step / steps
            w = 0
            camera = Camera(x - d * sin(-v),
                            y - d * sin(-u),
                            z + d * cos(-v),
                            u, v + pi, w)
            screen = new_screen(rows, columns)
            zbuffer = new_zbuffer(rows, columns)
            if step == 0:
                for triangle in tqdm(model, desc="Drawing First frame.."):
                    draw_triangle_relative(
                        rows, columns, screen, zbuffer, triangle, camera)
            else:
                for triangle in model:
                    draw_triangle_relative(
                        rows, columns, screen, zbuffer, triangle, camera)

            print_string = print_screen(rows, columns, screen, stdout)
            prerendered_screens.append(print_string)

        if len(argv) > 2:
            output_file = open(argv[2], 'w')
            output_file.write("#!/bin/sh\n")
            output_file.write("# Script generated with oak3d\n\n\n")
            output_file.write("echo -en \"\033[1J\"")
            output_file.write("echo -en \"\\033[?25l\"")
            for screen in prerendered_screens:
                # output_file.write("\ncat << 'EOF'\n")
                output_file.write("echo -en \"")
                output_file.write(screen)
                output_file.write("\"\n")
                # output_file.write("\nEOF\n")
                output_file.write("sleep 0.01\n")
            output_file.write("\n\necho -en \"\033[?25h\"")
            output_file.write("\n\necho -en \"\\033[?25l\"")
            stdout.write(esc_reset_cursor)
        else:
            # then repeat from memory
            while True:
                for screen in prerendered_screens:
                    stdout.write(screen)
                    sleep(0.01)

    except KeyboardInterrupt:
        stdout.write(esc_draw_rgb_bg%(0,0,0))
        stdout.write(esc_draw_rgb_fg%(255,255,255))
    finally:
        stdout.write(esc_reset_cursor)


def sample(screen, y, x, samples_per_pixel):
    rsum, gsum, bsum = 0.0, 0.0, 0.0
    for dy in range(samples_per_pixel):
        for dx in range(samples_per_pixel):
            r, g, b = screen[y + dy][x + dx]
            rsum += r / samples_per_pixel
            gsum += g / samples_per_pixel
            bsum += b / samples_per_pixel
    return rsum / samples_per_pixel, gsum / samples_per_pixel, bsum / samples_per_pixel


def print_screen(rows, columns, screen, stdout):
    print_string = esc_position_cursor % (0, 0)
    for y in range(0, rows, 2 * samples_per_pixel):
        for x in range(0, columns, samples_per_pixel):
            r1, g1, b1 = map_color_to_rgb(
                sample(screen, y, x, samples_per_pixel))
            r2, g2, b2 = map_color_to_rgb(
                sample(screen, y + samples_per_pixel, x, samples_per_pixel))
            print_string += esc_draw_rgb_bg % (r1, g1, b1) + \
                esc_draw_rgb_fg % (r2, g2, b2) + half_block
        if y < rows - 2 * samples_per_pixel:
            print_string += "\n"

    # print everything at the end to prevent screen tearing
    stdout.write(print_string)
    stdout.flush()
    return print_string


def draw_triangle_relative(height, width, screen, zbuffer, triangle, camera):
    # get three point of triangle
    p1 = map_point_to_screen(point_relative_to_camera(
        triangle.p1, camera), height, width)
    p2 = map_point_to_screen(point_relative_to_camera(
        triangle.p2, camera), height, width)
    p3 = map_point_to_screen(point_relative_to_camera(
        triangle.p3, camera), height, width)
    # add fog
    p1, p2, p3 = [add_fog(p) for p in [p1, p2, p3]]

    draw_triangle(height, width, screen, zbuffer, p1, p2, p3)


def add_lights(point, lights):
    r, g, b = ambient_light
    for light_color, light_normal in lights:
        brightness = -dot_product(point.normal, light_normal)
        brightness = max(0, min(1, brightness))
        _r, _g, _b = [sqrt(brightness) * c1 * c2 for c1,
                      c2 in zip(point.color, light_color)]
        r += _r
        g += _g
        b += _b
    return Point(point.x, point.y, point.z, (r, g, b), point.normal)


def add_fog(p):
    return Point(p.x, p.y, p.z,
                 blend_color(p.color, color_fog, 1 - (p.z / max_draw_dist)),
                 p.normal)


def draw_triangle(height, width, screen, zbuffer, p1, p2, p3):
    class Scanbuffer():
        "Class for drawing triangles effeciently"

        def __init__(self):
            self.minX = [0 for _ in range(height * 2)]
            self.maxX = [0 for _ in range(height * 2)]
            self.minZ = [0 for _ in range(height * 2)]
            self.maxZ = [0 for _ in range(height * 2)]
            self.minC = [0 for _ in range(height * 2)]
            self.maxC = [0 for _ in range(height * 2)]
        def draw_part(self, y_min, y_max):
            for y in range(max(-height, int(y_min)), min(height, int(y_max)) + 1):
                try:
                    draw_line_horizontal(height, width, screen, zbuffer, y,
                                         self.minX[y], self.maxX[y],
                                         self.minZ[y], self.maxZ[y],
                                         self.minC[y], self.maxC[y])
                except:
                    pass
        def write_line(self, p_low, p_high, handedness):
            xdist = p_high.x - p_low.x
            ydist = p_high.y - p_low.y
            if ydist <= 0:
                return
            xstep = xdist / ydist
            xcurrent = p_low.x
            zcurrent = p_low.z
            zstep = (p_high.z - p_low.z) / ydist
            r, g, b = p_low.color
            r2, g2, b2 = p_high.color
            dr, dg, db = (r2 - r) / ydist, (g2 - g) / ydist, (b2 - b) / ydist
            for y in range(int(p_low.y), int(p_high.y)):
                ratio = ((y - p_low.y) / (p_high.y - p_low.y))
                if y >= height or y < 0:
                    break

                if handedness:
                    self.minX[y] = int(xcurrent)
                    self.minC[y] = r, g, b
                    self.minZ[y] = zcurrent
                else:
                    self.maxX[y] = int(xcurrent)
                    self.maxC[y] = r, g, b
                    self.maxZ[y] = zcurrent
                xcurrent += xstep
                zcurrent += zstep
                r += dr
                g += dg
                b += db

    if ((p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * ((p1.y - p3.y))) <= 0:
        return                  # back face culling

    # simple bubble sort to order points from low to high
    if p1.y > p2.y:
        p1, p2 = p2, p1
    if p2.y > p3.y:
        p2, p3 = p3, p2
    if p1.y > p2.y:
        p1, p2 = p2, p1
    # scanbuffer allows fast triangle drawing
    if p1.y < p3.y:
        sbuffer = Scanbuffer()
        sbuffer.write_line(p1, p2, False)
        sbuffer.write_line(p2, p3, False)
        sbuffer.write_line(p1, p3, True)
        sbuffer.draw_part(p1.y, p3.y)


def blend_color(color1, color2, ratio):
    r1, g1, b1 = color1
    r2, g2, b2 = color2
    return ((r1 * ratio + r2 * (1 - ratio)),
            (g1 * ratio + g2 * (1 - ratio)),
            (b1 * ratio + b2 * (1 - ratio)))


def draw_line_horizontal(height, width, screen, zbuffer, y, x1, x2, z1, z2, c1, c2):
    "For every point visible on the line, draw a pixel"
    if x1 > x2:
        x1, x2 = x2, x1
        c1, c2 = c2, c1
        z1, z2 = z2, z1
    else:
        pass

    # check if line is inside screen
    if x1 > width or x2 < 0 or y < 0 or y > height:
        return

    for x in range(x1, x2):
        ratio = (float(x) - x1) / (float(x2) - x1)
        color = blend_color(c2, c1, ratio)
        z = z1 * (1 - ratio) + z2 * (ratio)
        add_pixel_to_screen(height, width, screen, zbuffer,
                            width - x, y, z, color)


def new_screen(height, width):
    return [[color_fog for x in range(width)] for y in range(height)]


def new_zbuffer(height, width):
    return [[max_draw_dist for x in range(width)] for y in range(height)]


class Point():

    def __init__(self, x, y, z, color, normal):
        self.x, self.y, self.z, self.color, self.normal = x, y, z, color, normal


class Triangle():

    def __init__(self, p1, p2, p3):
        self.p1, self.p2, self.p3 = p1, p2, p3


class Camera():

    def __init__(self, x=0, y=0, z=0, u=0, v=0, w=0):
        self.x, self.y, self.z = x, y, z              # position
        self.u, self.v, self.w = u, v, w              # angle


def point_relative_to_camera(point, camera):
    "Gives newcoordinate for a point relative to a cameras position and angle"
    # tranlate to camera
    x = point.x - camera.x
    y = point.y - camera.y
    z = point.z - camera.z

    x, y, z = rotate_3d(x, y, z, camera.u, camera.v, camera.w)

    n1, n2, n3 = point.normal
    new_normal = rotate_3d(n1, n2, n3, camera.u, camera.v, camera.w)

    return Point(x, y, z, point.color, new_normal)


def map_color_to_rgb(color):
    return map((lambda c: int((min(1, max(0, c))**(gamma_correction)) * 255)), color)


def rotate_3d(x, y, z, u, v, w):
    # shorthands so the projection formula is easier to read
    sx, cx, sy, cy, sz, cz = (sin(u), cos(u), sin(v), cos(v), sin(w), cos(w))
    # Rotation around camera
    x, y, z = (cy * (sz * y + cz * x) - sy * z,
               sx * (cy * z + sy * (sz * y + cz * x)) + cx * (cz * y - sz * x),
               cx * (cy * z + sy * (sz * y + cz * x)) - sx * (cz * y - sz * x))

    return x, y, z


def normalize_vector(v):
    a, b, c = v
    size = sqrt(a * a + b * b + c * c)
    if size > 0:
        return a / size, b / size, c / size
    return (0, 0, 0)


def random_vector():
    return normalize_vector((0.5 - random(), 0.5 - random(), 0.5 - random()))


def dot_product(v1, v2):
    return sum([a * b for (a, b) in zip(v1, v2)])


def add_pixel_to_screen(height, width, screen, zbuffer, x, y, z, color):
    if x < 0 or x >= width or y < 0 or y >= height:
        return
    if z > zbuffer[int(y)][int(x)] or z < 0:
        return
    screen[int(y)][int(x)] = color
    zbuffer[int(y)][int(x)] = z


def map_point_to_screen(point, height, width, zoom=2.5, ratio=0.4):
    x, y, z, color = point.x, point.y, point.z, point.color
    new_z = max(0.01, z)
    new_x = (zoom * ratio * x / (1 + new_z) + 1) * width * 0.5
    new_y = (zoom * -y / (1 + new_z) + 1) * height * 0.5
    return Point(new_x, new_y, z, color, point.normal)


def load_obj(filename):
    "Parse an .obj file and return an array of Triangles"
    global draw_dist_min, draw_dist_max, zoomfactor
    vertices, normals, faces = [], [], []
    # each line represents 1 thing, we care only about
    # vertices(points) and faces(triangles)
    for linenumber, line in tqdm(enumerate(open(filename).readlines()), desc="Parsing Model..."):
        c = line[0]
        if c == "v":            # vertices information
            if line[1] in "t":  # We ignore textures
                pass
            elif line[1] == "n":  # normals
                coords = list(map(float, line[2:-1].split()))
                normals.append((coords[0], coords[1], coords[2]))
            else:
                coords = list(map(float, line[1:-1].split()))
                vertices.append(Point(coords[0], coords[1], coords[2], color_default(
                    coords[0], coords[1], coords[2]), random_vector()))
        elif c == "f":          # face information
            normali = None
            if "/" in line:  # check for a/b/c syntax
                if "//" in line:  # check for a//b b//c c//d sumtax
                    indexes = [list(map(lambda x:int(x.split("//")[0]),
                                        miniline.split(" ")))[0] - 1
                               for miniline in line[2:-1].split()]
                    normali = [list(map(lambda x:int(x.split("//")[1]),
                                        miniline.split(" ")))[0] - 1
                               for miniline in line[2:-1].split()]
                else:
                    indexes = [list(map(int, miniline.split("/")))[0] - 1
                               for miniline in line[2:-1].split()]
                    normali = [list(map(lambda x:int(x.split("/")[-1]),
                                        miniline.split(" ")))[0] - 1
                               for miniline in line[2:-1].split()]
            else:
                indexes = list(map(lambda x: (int(x) - 1), line[1:-1].split()))

            for i in range(0, len(indexes) - 2):
                try:
                    p1, p2, p3 = vertices[indexes[0]], vertices[
                        indexes[i + 1]], vertices[indexes[i + 2]]
                    n1, n2, n3 = normals[normali[0]], normals[
                        normali[i + 1]], normals[normali[i + 2]]
                    face = Triangle(Point(p1.x, p1.y, p1.z, p1.color, n1),
                                    Point(p2.x, p2.y, p2.z, p2.color, n2),
                                    Point(p3.x, p3.y, p3.z, p3.color, n3))
                    faces.append(face)
                except:
                    p1, p2, p3 = vertices[indexes[0]], vertices[
                        indexes[i + 1]], vertices[indexes[i + 2]]
                    v = normal_from_triangle(p1, p2, p3)
                    face = Triangle(Point(p1.x, p1.y, p1.z, p1.color, v),
                                    Point(p2.x, p2.y, p2.z, p2.color, v),
                                    Point(p3.x, p3.y, p3.z, p3.color, v))
                    faces.append(face)
        else:
            pass                # ignore all other information

    # add lighting
    shaded_faces = []
    for face in tqdm(faces, desc="Applying lighting"):
        p1, p2, p3 = face.p1, face.p2, face.p3
        p1, p2, p3 = map((lambda p: add_lights(p, [(color_sun, normalize_vector(angle_sun)),
                                                   (color_countersun, normalize_vector(angle_countersun1)),
                                                   (color_countersun, normalize_vector(angle_countersun2)),
                                                   (color_countersun, normalize_vector(angle_countersun3))])), [p1, p2, p3])
        new_face = Triangle(p1, p2, p3)
        shaded_faces.append(new_face)

    if len(vertices) <= 0:
        stderr.write("Model contains no vertices\n")
        quit(1)

    return shaded_faces


def normal_from_triangle(p1, p2, p3):
    U = Point(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z, None, None)
    V = Point(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z, None, None)

    Nx = U.y * V.z - U.z * V.y
    Ny = U.z * V.x - U.x * V.z
    Nz = U.x * V.y - U.y * V.x

    return normalize_vector((Nx, Ny, Nz))


def get_camera_values(model):
    vertices = [v for t in model for v in [t.p1, t.p2, t.p3]]
    min_x = min(map(lambda v: v.x, vertices))
    min_y = min(map(lambda v: v.y, vertices))
    min_z = min(map(lambda v: v.z, vertices))
    max_x = max(map(lambda v: v.x, vertices))
    max_y = max(map(lambda v: v.y, vertices))
    max_z = max(map(lambda v: v.z, vertices))

    center_x = (max_x + min_x) / 2
    center_y = (max_y + min_y) / 2
    center_z = (max_z + min_z) / 2

    return center_x, center_y, center_z, abs(max_y - min_y)

if __name__ == "__main__":
    main()
