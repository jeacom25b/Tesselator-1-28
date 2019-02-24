'''
Copyright (C) 2018 Jean Da Costa machado.
Jean3dimensional@gmail.com

Created by Jean Da Costa machado

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import bpy
import bmesh
import numpy as np
import math
from random import random, choice
from itertools import product
from mathutils import Vector, Matrix, Color
from mathutils.geometry import barycentric_transform
from mathutils.bvhtree import BVHTree
from .multifile import register_class
from .draw_3d import *


def get_gp_frame(context):
    frame = None
    gp = context.scene.grease_pencil
    if gp:
        if gp.layers:
            if gp.layers.active:
                if gp.layers.active.active_frame:
                    frame = gp.layers.active.active_frame
                    print(frame)
    return frame


def average_curvature(vert):
    curv = 0
    tot = 0
    for edge in vert.link_edges:
        other = edge.other_vert(vert)
        d = other.co - vert.co
        nd = other.normal - vert.normal
        curv += nd.dot(d) / d.length_squared
        tot += 1
    if not tot:
        return 0
    else:
        return curv / tot


def curvature_direction(vert):
    if vert.is_boundary:
        for edge in vert.link_edges:
            if edge.is_boundary:
                other = edge.other_vert(vert)
                d = vert.co - other.co
                return d.normalized()
    try:
        other = min((edge.other_vert(vert) for edge in vert.link_edges), key=lambda v: v.normal.dot(vert.normal))
        vec = other.normal.cross(vert.normal).normalized()
        if vec.length_squared == 0:
            raise ValueError()
        return vec
    except ValueError:
        return random_tangent_vector(vert.normal)


def average_curvature(vert):
    return sum((abs(edge.other_vert(vert).normal.dot(vert.normal)) for edge in vert.link_edges)) / len(vert.link_edges)


def random_tangent_vector(normal):
    return normal.cross(np.random.sample(3) - 0.5).normalized()


def vert_normal(vert):
    return vert.normal


def normalize_vectors_array(arr):
    magnitudes = np.sqrt((arr ** 2).sum(axis=1))
    return arr / magnitudes[:, np.newaxis]


def best_matching_vector(tests, reference):
    return max(tests, key=lambda v: v.dot(reference))


def best_matching_vector_unsigned(tests, reference):
    return max(tests, key=lambda v: abs(v.dot(reference)))


def best_vector_combination(vecs_a, vecs_b):
    a, b = max(product(vecs_a, vecs_b), key=lambda a: a[0].dot(a[1]))
    return a, b


def symmetry_space(vec, normal):
    vec1 = Vector(vec).cross(normal)
    vec = Vector(vec)
    return vec, vec1, -vec, -vec1


def hex_symmetry_space(vec, normal):
    x = Vector(vec)
    y = Vector(vec).cross(normal)
    e = x * 0.5 + y * 0.866025
    f = x * -0.5 + y * 0.866025
    return x, e, f, -x, -e, -f


class Field:
    def __init__(self, obj, max_adjacent=20):

        self.matrix = obj.matrix_world.copy()
        self.draw = DrawCallback()
        self.draw.matrix = self.matrix
        self.bm = bmesh.new()
        self.bm.from_mesh(obj.data)
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)
        self.bm.verts.ensure_lookup_table()
        self.bm.edges.ensure_lookup_table()
        self.bm.faces.ensure_lookup_table()

        self.hex_mode = False

        self.bvh = BVHTree.FromBMesh(self.bm)
        self.n = len(self.bm.verts)
        self.max_adjacent = max_adjacent
        self.singularities = []

        self.locations = np.array([vert.co for vert in self.bm.verts], dtype=np.float32)
        self.normals = np.zeros((self.n, 3), dtype=np.float32)
        self.adjacent_counts = np.zeros((self.n,), dtype=np.float32)
        self.field = np.zeros((self.n, 3), dtype=np.float64)
        self.scale = np.zeros((self.n,), dtype=np.float64)
        self.curvature = np.zeros((self.n,), dtype=np.float64)
        self.weights = np.ones((self.n,), dtype=np.float64)

        self.connectivity = np.zeros((self.n, max_adjacent), dtype=np.int64)
        mask_layer = self.bm.verts.layers.paint_mask.verify()
        for vert in self.bm.verts:
            i = vert.index
            self.field[i] = curvature_direction(vert)
            self.normals[i] = vert_normal(vert)
            self.scale[i] = vert[mask_layer]
            self.curvature[i] = average_curvature(vert)
            self.adjacent_counts[i] = min(len(vert.link_edges), max_adjacent)
            if vert.is_boundary:
                self.weights[vert.index] = 0
            for j, e in enumerate(vert.link_edges):
                if j >= max_adjacent:
                    continue
                self.connectivity[i, j] = e.other_vert(vert).index

    def initialize_from_gp(self, context):
        mat = self.matrix.inverted()
        frame = get_gp_frame(context)
        seen_verts = set()
        if frame:
            for stroke in frame.strokes:
                le = len(stroke.points)
                for i in range(le - 2):
                    p0 = mat @ stroke.points[i].co
                    p1 = mat @ stroke.points[i + 1].co
                    p2 = mat @ stroke.points[i + 2].co
                    d = p0 - p1
                    d += p1 - p2

                    location, normal, index, dist = self.bvh.find_nearest(p1)
                    face = self.bm.faces[index]
                    vert = min(face.verts, key=lambda v: (v.co - p1).length_squared)
                    self.field[vert.index] = d.normalized()
                    self.weights[vert.index] = 0
                    seen_verts.add(vert)

        current_front = set()
        for vert in seen_verts:
            for edge in vert.link_edges:
                other = edge.other_vert(vert)
                if other not in seen_verts:
                    current_front.add(vert)

        while current_front:
            new_front = set()
            for vert in current_front:
                d = Vector()
                tot = 0
                for edge in vert.link_edges:
                    other = edge.other_vert(vert)
                    if other in seen_verts:
                        if not tot:
                            d = Vector(self.field[other.index])
                        else:
                            d += best_matching_vector(
                                symmetry_space(self.field[other.index], other.normal),
                                d
                            )
                        tot += 1
                    else:
                        new_front.add(other)
                        self.weights[other.index] = self.weights[vert.index] + 1
                    if tot:
                        self.field[vert.index] = d.normalized().cross(vert.normal)
            seen_verts |= current_front
            new_front -= seen_verts
            current_front = new_front
        self.weights /= self.weights.max()

    def walk_edges(self, depth=0):
        cols = np.arange(self.n)
        ids = np.random.randint(0, self.max_adjacent, (self.n,)) % self.adjacent_counts
        ids = ids.astype(np.int_)
        adjacent_edges = self.connectivity[cols, ids]
        for _ in range(depth):
            ids = np.random.randint(0, self.max_adjacent, (self.n,)) % self.adjacent_counts[adjacent_edges]
            ids = ids.astype(np.int_)
            adjacent_edges = self.connectivity[adjacent_edges, ids]
        return adjacent_edges

    def smooth(self, iterations=100, depth=3, hex_mode=False):

        def find_best_combinations(a, b):
            w = self.weights[:, np.newaxis]
            scores = []
            vectors = []
            for a, b in product(a, (b * w)):
                m = (a * b).sum(axis=1)
                scores.append(m)
                vectors.append((a + b))
            scores = np.stack(scores, axis=0)
            vectors = np.stack(vectors, axis=0)
            idx = scores.argmax(axis=0)
            cols = np.arange(self.n)
            rval = vectors[idx, cols]
            nans = np.isnan(rval)
            rval[nans] = 0
            return rval * (1 / (w + 1))

        if not self.hex_mode:
            for i in range(iterations):
                print(i)
                a = self.field
                b = np.cross(self.field, self.normals)
                adjacent_edges = self.walk_edges(depth)

                c = self.field[adjacent_edges]
                d = np.cross(c, self.normals[adjacent_edges])

                best = find_best_combinations((a, b, -a, -b), (c, d))
                best = best - self.normals * (best * self.normals).sum(axis=1)[:, np.newaxis]
                self.field = best
        else:
            for i in range(iterations):
                print(i)
                x = self.field
                y = np.cross(self.field, self.normals)
                a = x
                b = x * 0.5 + y * 0.866025
                c = x * -0.5 + y * 0.866025

                adjacent_edges = self.walk_edges(depth)

                x = self.field[adjacent_edges]
                y = np.cross(x, self.normals[adjacent_edges])
                d = x
                e = x * 0.5 + y * 0.866025
                f = x * -0.5 + y * 0.866025

                best = find_best_combinations((a, b, c, -a, -b, -c), (d, e, f))
                best = best - self.normals * (best * self.normals).sum(axis=1)[:, np.newaxis]
                self.field = best

        self.field = normalize_vectors_array(self.field)

    def autoscale(self):
        symmetry = hex_symmetry_space if self.hex_mode else symmetry_space

        for vert in self.bm.verts:
            u = Vector(self.field[vert.index])
            v = u.cross(vert.normal)
            ang = 0
            last_vec = u
            for loop in vert.link_loops:
                vert1 = loop.link_loop_next.vert
                vert2 = loop.link_loop_next.link_loop_next.vert
                if not last_vec:
                    vert1_vec = Vector(self.field[vert1.index])
                else:
                    vert1_vec = last_vec

                vert2_vec = best_matching_vector(symmetry(self.field[vert2.index], vert2.normal), vert1_vec)

                vert1_vec = Vector((vert1_vec.dot(u), vert1_vec.dot(v)))
                vert2_vec = Vector((vert2_vec.dot(u), vert2_vec.dot(v)))

                ang += vert1_vec.angle_signed(vert2_vec)
            self.scale[vert.index] = ang
        for i in range(20):
            self.scale += self.scale[self.walk_edges(0)]
            self.scale /= 2
        self.scale -= self.scale.min()
        self.scale /= self.scale.max()

    def mirror(self, axis=0):
        mirror_vec = Vector()
        mirror_vec[axis] = -1
        for vert in self.bm.verts:
            if vert.co[axis] < 0:
                mirror_co = vert.co.copy()
                mirror_co[axis] *= -1
                location, normal, vec, s, c = self.sample_point(mirror_co)
                self.field[vert.index] = vec - vec.dot(mirror_vec) * 2 * mirror_vec

    def detect_singularities(self):
        symmetry = hex_symmetry_space if self.hex_mode else symmetry_space
        cache = {}

        def symmetry_cached(vert):
            if vert in cache:
                return cache[vert]
            else:
                s = symmetry(self.field[vert.index], vert.normal)
                cache[vert] = s
                return s

        singularities = []

        if not self.hex_mode:
            for face in self.bm.faces:
                v0 = face.verts[0]
                v1 = face.verts[1]
                v2 = face.verts[2]
                vec0 = self.field[v0.index]
                vec1 = best_matching_vector(symmetry_cached(v1), vec0)
                v2_symmetry = symmetry_cached(v2)
                match0 = best_matching_vector(v2_symmetry, vec0)
                match1 = best_matching_vector(v2_symmetry, vec1)
                if match0.dot(match1) < 0.5:
                    singularities.append(face.calc_center_median())
        else:
            for vert in self.bm.verts:
                ang = 0
                u = random_tangent_vector(vert.normal)
                v = u.cross(vert.normal)
                last_vec = None
                for loop in vert.link_loops:
                    vert1 = loop.link_loop_next.vert
                    vert2 = loop.link_loop_next.link_loop_next.vert
                    if not last_vec:
                        vert1_vec = symmetry_cached(vert1)[0]
                    else:
                        vert1_vec = last_vec
                    vert2_vec = best_matching_vector(symmetry_cached(vert2), vert1_vec)
                    last_vec = vert2_vec
                    vert1_vec = Vector((vert1_vec.dot(u), vert1_vec.dot(v)))
                    vert2_vec = Vector((vert2_vec.dot(u), vert2_vec.dot(v)))
                    ang += vert1_vec.angle_signed(vert2_vec)
                if ang > 0.9:
                    singularities.append(vert.co)

        self.singularities = singularities

    def sample_point(self, point, ref_dir=None):
        location, normal, index, distance = self.bvh.find_nearest(point)
        if location:
            face = self.bm.faces[index]
            face_verts_co = [vert.co for vert in face.verts]
            if not ref_dir:
                ref_dir = self.field[face.verts[0].index]

            field = [
                best_matching_vector(
                    symmetry_space(
                        self.field[vert.index], vert.normal) if not self.hex_mode
                    else hex_symmetry_space(self.field[vert.index], vert.normal),
                    reference=ref_dir
                )
                for vert in face.verts
            ]

            dir = barycentric_transform(point, *face_verts_co, *field)
            scale_curv = [Vector((self.scale[vert.index], self.curvature[vert.index], 0)) for vert in face.verts]
            scale_curv = barycentric_transform(point, *face_verts_co, *scale_curv)
            scale = scale_curv[0]
            curv = scale_curv[1]
            dir -= normal * normal.dot(dir)
            dir.normalize()
            return location, normal, dir, scale, curv
        else:
            return None, None, None, None

    def preview(self):
        draw = self.draw
        draw.blend_mode = MULTIPLY_BLEND
        draw.line_width = 1.5
        draw.point_size = 20
        draw.clear_data()

        blue = Vector((0.7, 0.7, 1, 1))
        red = Vector((1, 0, 0, 1))
        white = Vector((1, 1, 1, 1))
        for vert in self.bm.verts:
            fac = self.scale[vert.index]
            loc = vert.co
            color = np.array((fac ** 2, ((1 - fac) * 4 * fac), (1 - fac) ** 2, 1))
            color += 2
            color /= 3
            size = sum(edge.calc_length() for edge in vert.link_edges) / len(vert.link_edges)
            u = Vector(self.field[vert.index]) * size
            vecs = symmetry_space(u, vert.normal) if not self.hex_mode else hex_symmetry_space(u, vert.normal)
            for v in vecs:
                draw.add_line(loc, loc + v, color1=color, color2=white)

        for singularity in self.singularities:
            draw.add_point(singularity, red)

        draw.update_batch()

    def preview_fast(self):
        draw = self.draw
        draw.blend_mode = MULTIPLY_BLEND
        draw.line_width = 1
        d = self.locations - self.locations[self.walk_edges(0)]
        edge_lengths = np.sqrt(np.sum(d ** 2, axis=1))[:, np.newaxis]
        draw.line_coords = np.empty((self.n * 2, 3), dtype=np.float32)
        draw.line_coords[0::2] = self.locations
        draw.line_coords[1::2] = self.locations + self.field * edge_lengths * 0.5
        white = np.array([[1, 1, 1, 1]])
        blue = np.array([[0, 0, 1, 1]])
        draw.line_colors = np.empty((self.n * 2, 4), dtype=np.float32)
        draw.line_colors[0::2] = np.repeat(blue, [self.n], axis=0)
        draw.line_colors[1::2] = np.repeat(white, [self.n], axis=0)
        self.draw.update_batch()

    @register_class
    class Test(bpy.types.Operator):
        bl_idname = "tesselator2.testfield"
        bl_label = "Test Field"
        bl_description = ""
        bl_options = {"REGISTER", "UNDO"}
        _timer = None

        @classmethod
        def poll(cls, context):
            return context.active_object and context.active_object.type == "MESH"

        def execute(self, context):
            wm = context.window_manager
            wm.modal_handler_add(self)
            self.field = Field(context.active_object)
            self.field.hex_mode = True
            self.field.smooth(20, 20, )
            self.field.smooth(100, 0, 0)
            self.field.autoscale()
            # self.field.initialize_from_gp(context)
            self.field.weights *= 5
            self.field.weights = np.minimum(self.field.weights, 1)
            # self.field.smooth(50, 0)
            self.field.detect_singularities()
            self.field.preview()
            self.field.draw.setup_handler()
            return {"RUNNING_MODAL"}

        def modal(self, context, event):
            if event.type == "ESC":
                self.field.draw.remove_handler()
                context.area.tag_redraw()
                return {"FINISHED"}

            return {"PASS_THROUGH"}
