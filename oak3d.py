#!/usr/bin/env python3

from math import sin,cos,pi,atan,atan2,sqrt
from random import random
import os
import shutil
import sys
import time
import argparse

columns, rows = shutil.get_terminal_size((80, 20))

output = open(sys.argv[1],'w') if len(sys.argv) > 1 else sys.stdout

# escape codes, send to stdout to do stuff
esc_draw_rgb        = "\x1b[48;2;%i;%i;%im "
esc_position_cursor = "\033[%i;%iH"
esc_clear_screen    = "\033[1J"
esc_hide_cursor     = "\033[?25l"
esc_reset_cursor    = "\033[?25h"

max_draw_dist = 9999


from opensimplex import OpenSimplex
noise_generator = OpenSimplex(seed=int(10000*random()))
def my_noise(x,y,noise,depth=7):
    return sum([noise(x/(2**(depth-d)),y/(2**(depth-d))) / (2**(d+1))
                for d in range(depth)])


def main():
    try:
        floor_size = 20

        floor_height = [[my_noise(w*20,h*20,noise_generator.noise2d)
                         for w in range(floor_size)]
                        for h in range(floor_size)]
        floor_points = [[None for w in range(floor_size)] for h in range(floor_size)]
        floor_triangles = []

        for h in range(floor_size):
            for w in range(floor_size):
                x = w - floor_size/2
                z = h - floor_size/2
                y = 4*floor_height[h][w]

                p = Point(x,y,z,
                          (random(),
                           random(),
                           random()))
                floor_points[h][w] = p

        for h in range(floor_size-1):
            for w in range(floor_size-1):
                p0 = floor_points[h][w]
                p1 = floor_points[h+1][w]
                p2 = floor_points[h][w+1]
                p3 = floor_points[h+1][w+1]
                floor_triangles.append(Triangle(p0,p1,p2))
                floor_triangles.append(Triangle(p2,p1,p3))


        output.write(esc_hide_cursor)
        view_steps = 1000
        for t in range(view_steps):
            camera = Camera(0,2,0,
                            0,(t*2*pi) / view_steps,0)
            screen  = new_screen(rows,columns)
            zbuffer = new_zbuffer(rows,columns)
            for triangle in floor_triangles:
                draw_triangle_relative(rows,columns,screen,zbuffer,triangle,camera)
            print_screen(rows,columns,screen,output)
    finally:
        output.write(esc_reset_cursor)

def print_screen(rows,columns,screen,output):
    output.write(esc_position_cursor%(0,0))
    for y in range(rows):
        for x in range(columns):
            r,g,b = map_color_to_rgb(screen[y][x])
            output.write(esc_draw_rgb%(r,g,b))
        if y < rows-1:
            output.write("\n")
    output.write(esc_position_cursor%(0,0))

def draw_triangle_relative(height,width,screen,zbuffer,triangle,camera):
    # get three point of triangle
    p1 = map_point_to_screen(point_relative_to_camera(triangle.p1,camera),height,width)
    p2 = map_point_to_screen(point_relative_to_camera(triangle.p2,camera),height,width)
    p3 = map_point_to_screen(point_relative_to_camera(triangle.p3,camera),height,width)



    draw_triangle(height,width,screen,zbuffer,p1,p2,p3)
    # wireframe for reference

def draw_triangle(height,width,screen,zbuffer,p1,p2,p3):
    class Scanbuffer():
        "Class for drawing triangles effeciently"
        def __init__(self):
            self.minX=[0 for _ in range(height*2)]
            self.maxX=[0 for _ in range(height*2)]
            self.minZ=[0 for _ in range(height*2)]
            self.maxZ=[0 for _ in range(height*2)]
            self.minC=[0 for _ in range(height*2)]
            self.maxC=[0 for _ in range(height*2)]
        def draw_part(self,y_min,y_max):
            for y in range(max(-height,int(y_min)),min(height,int(y_max))):
                try:
                    draw_line_horizontal(height,width,screen,zbuffer,y,
                                         self.minX[y],self.maxX[y],
                                         self.minZ[y],self.maxZ[y],
                                         self.minC[y],self.maxC[y])
                except:
                    pass
        def write_line(self,p_low,p_high,handedness):
            xdist = p_high.x - p_low.x
            ydist = p_high.y - p_low.y
            if ydist<=0:
                return
            xstep = xdist / ydist
            xcurrent = p_low.x
            zcurrent = p_low.z
            zstep    = (p_high.z - p_low.z) / ydist
            r,g,b    = p_low.color
            r2,g2,b2 = p_high.color
            dr,dg,db = (r2-r)/ydist,(g2-g)/ydist,(b2-b)/ydist
            for y in range(int(p_low.y),int(p_high.y)):
                ratio = ((y-p_low.y)/(p_high.y - p_low.y))
                if y>= height or y<0:
                    break

                if handedness:
                    self.minX[y] = int(xcurrent)
                    self.minC[y] = r,g,b
                    self.minZ[y] = zcurrent
                else:
                    self.maxX[y] = int(xcurrent)
                    self.maxC[y] = r,g,b
                    self.maxZ[y] = zcurrent
                xcurrent += xstep
                zcurrent += zstep
                r += dr
                g += dg
                b += db

    if ((p1.x - p3.x)*(p2.y - p3.y) - (p2.x - p3.x)*((p1.y - p3.y))) <= 0:
        return                  # back face culling

    # simple bubble sort to order points from low to high
    if p1.y > p2.y:
        p1,p2 = p2,p1
    if p2.y > p3.y:
        p2,p3 = p3,p2
    if p1.y > p2.y:
        p1,p2 = p2,p1
    # scanbuffer allows fast triangle drawing
    sbuffer = Scanbuffer()
    sbuffer.write_line(p1, p2, False)
    sbuffer.write_line(p2, p3, False)
    sbuffer.write_line(p1, p3, True)
    sbuffer.draw_part(p1.y,p3.y)


def blend_color(color1,color2,ratio):
    r1,g1,b1 = color1
    r2,g2,b2 = color2
    return ((r1*ratio + r2*(1-ratio)),
            (g1*ratio + g2*(1-ratio)),
            (b1*ratio + b2*(1-ratio)))


def draw_line(height,width,screen,zbuffer,p1,p2):
    "For every point visible on the line, draw a pixel"

    steps = max(abs(p1.x-p2.x),abs(p1.y-p2.y))
    if steps>0:
        for s in range(int(steps+1)):
            r1,r2 = s/steps, (1- s/steps)
            x,y,z,color = r1*p1.x + r2*p2.x,\
                          r1*p1.y + r2*p2.y,\
                          r1*p1.z + r2*p2.z,\
                          blend_color(p1.color,p2.color,r1)
            add_pixel_to_screen(height,width,screen,zbuffer,x,y,z,color)
    else:
        return

def draw_line_horizontal(height,width,screen,zbuffer,y,x1,x2,z1,z2,c1,c2):
    "For every point visible on the line, draw a pixel"
    if x1>x2:
        x1,x2 = x2,x1
        c1,c2 = c2,c1
        z1,z2 = z2,z1
    else:
        pass
    for x in range(x1,x2):
        ratio = (float(x) - x1) / (float(x2) - x1)
        color = blend_color(c2,c1,ratio)
        z     = z1*(1-ratio) + z2*(ratio)
        add_pixel_to_screen(height,width,screen,zbuffer,x,y,z,color)

def new_screen(height,width):
    return [[(0,0,0) for x in range(width)] for y in range(height)]

def new_zbuffer(height,width):
    return [[max_draw_dist for x in range(width)] for y in range(height)]

class Point():
    def __init__(self,x,y,z,color=(1,1,1),normal=(0,0,1)):
        self.x,self.y,self.z,self.color,self.normal = x,y,z,color,normal

class Triangle():
    def __init__(self,p1,p2,p3):
        self.p1,self.p2,self.p3=p1,p2,p3

class Camera():
    def __init__(self,x=0,y=0,z=0,u=0,v=0,w=0):
        global zoomfactor
        self.x,self.y,self.z = x,y,z              # position
        self.u,self.v,self.w = u,v,w              # angle

def point_relative_to_camera(point,camera):
    "Gives newcoordinate for a point relative to a cameras position and angle"
    # first we tranlate to camera
    x = point.x - camera.x
    y = point.y - camera.y
    z = point.z - camera.z
    # shorthands so the projection formula is easier to read
    sx,cx,sy,cy,sz,cz = (sin(camera.u),
                         cos(camera.u),
                         sin(camera.v),
                         cos(camera.v),
                         sin(camera.w),
                         cos(camera.w))
    # Rotation around camera
    x, y, z = (cy* (sz*y + cz*x) - sy*z,
               sx* (cy*z + sy*(sz*y + cz*x)) + cx*(cz*y-sz*x),
               cx* (cy*z + sy*(sz*y + cz*x)) - sx*(cz*y-sz*x))

    return Point(x,y,z,point.color)

def map_color_to_rgb(color):
    return map((lambda c:int(min(1,max(0,c))*255)),color)

def add_point_to_screen(height,width,screen,zbuffer,point):
    point = map_point_to_screen(point,height,width)
    add_pixel_to_screen(height,width,screen,zbuffer,x,y,z,color)

def add_pixel_to_screen(height,width,screen,zbuffer,x,y,z,color):
    if x<0 or x>= width or y<0 or y>=height:
        return
    if z > zbuffer[int(y)][int(x)] or z<0:
        return
    screen[int(y)][int(x)] = color
    zbuffer[int(y)][int(x)] = z




def map_point_to_screen(point,height,width,zoom=1,ratio=0.55):
    x,y,z,color = point.x,point.y,point.z,point.color
    new_z       = max(-.99,z)
    new_x       = (zoom*ratio*x/(1+new_z)+1) * width  * 0.5
    new_y       = (zoom*-y/(1+new_z)+1) * height * 0.5
    return Point(new_x,new_y,z,color)


if __name__ == "__main__":
    main()
