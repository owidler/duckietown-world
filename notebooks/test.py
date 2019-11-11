from pudb import set_trace; set_trace()

# disabling contracts for speed
import contracts
contracts.disable_all()

import duckietown_world as dw
dw.logger.setLevel(50)
from duckietown_world.svg_drawing.ipython_utils import ipython_draw_svg, ipython_draw_html
import geometry as geo
import numpy as np

def interpolate(q0, q1, alpha):
    v = geo.SE2.algebra_from_group(geo.SE2.multiply(geo.SE2.inverse(q0), q1))
    vi = v * alpha
    q = np.dot(q0, geo.SE2.group_from_algebra(vi))
    return q

class Person(dw.PlacedObject):

    def __init__(self, radius, *args, **kwargs):
        self.radius = radius
        dw.PlacedObject.__init__(self, *args, **kwargs)

    def draw_svg(self, drawing, g):
        # drawing is done using the library svgwrite
        c = drawing.circle(center=(0, 0), r=self.radius, fill='pink')
        g.add(c)
        # draws x,y axes
        dw.draw_axes(drawing, g)

    def extent_points(self):
        # set of points describing the boundary
        L = self.radius
        return [(-L, -L), (+L, +L)]

q0 = geo.SE2_from_translation_angle([0, 0], 0)
q1 = geo.SE2_from_translation_angle([2, -2], np.deg2rad(-90))

# create a sequence of poses
n = 10
seqs = []
steps = np.linspace(0, 1, num=n)
for alpha in steps:
    q = interpolate(q0, q1, alpha)
    seqs.append(q)

root = dw.PlacedObject()

timestamps = range(len(seqs)) # [0, 1, 2, ...]

# SE2Transform is the wrapper for SE2 used by Duckietown World
transforms = [dw.SE2Transform.from_SE2(_) for _ in seqs]
seq_me = dw.SampledSequence(timestamps, transforms)

print(seq_me.timestamps)

print(seq_me.values[0])
