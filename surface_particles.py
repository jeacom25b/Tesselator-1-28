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
import math
from itertools import product
from . import vector_fields
from .draw_3d import *
from .multifile import register_class
from mathutils import Vector
from mathutils.kdtree import KDTree
from mathutils.bvhtree import BVHTree
import bmesh
import numpy as np
from .interface import DebugText
import random
import traceback


def subdivide_split_triangles(bm):
    bmesh.ops.subdivide_edges(bm, edges=bm.edges, cuts=1, use_grid_fill=True, smooth=True)
    collapse = []
    triangulate = set()
    seen_verts = set()
    for vert in bm.verts:
        if len(vert.link_faces) == 5:
            face_signature = tuple(sorted(len(face.verts) for face in vert.link_faces))
            if face_signature == (3, 4, 4, 4, 4):
                for face in vert.link_faces:
                    if len(face.verts) == 3:
                        for edge in face.edges:
                            verts = set(edge.verts)
                            if vert not in verts and not verts & seen_verts:
                                seen_verts |= verts
                                collapse.append(edge)
                            triangulate |= set(face for v in verts for face in v.link_faces)
    bmesh.ops.triangulate(bm, faces=list(triangulate), quad_method="SHORT_EDGE")
    bmesh.ops.collapse(bm, edges=collapse)
    bmesh.ops.join_triangles(bm, faces=bm.faces, angle_face_threshold=3.16, angle_shape_threshold=3.16, cmp_seam=True)


def relax_topology(bm):
    for vert in bm.verts:
        if vert.is_boundary:
            continue
        avg = Vector()
        n = len(vert.link_edges)
        for edge in vert.link_edges:
            if edge.seam:
                n = 0
                break
            other = edge.other_vert(vert)
            avg += other.co
        if n in (3, 5, 0):
            continue
        avg /= n
        avg -= vert.co
        avg -= vert.normal * vert.normal.dot(avg)
        vert.co += avg * 0.5


def straigthen_quad_topology(bm):
    for vert in bm.verts:
        if vert.is_boundary:
            continue
        if len(vert.link_edges) == 3:
            valid = True
            for edge in vert.link_edges:
                if edge.seam:
                    valid = False
                    break
            if valid:
                pairs = [(e_a.other_vert(vert).co, e_b.other_vert(vert).co)
                         for e_a in vert.link_edges for e_b in vert.link_edges if e_a is not e_b]
                best_pair = min(pairs, key=lambda pair: (pair[0] - pair[1]).length_squared)
                vert.co = sum(best_pair, vert.co * 0.2) / 2.2


def bvh_snap(bvh, verts):
    for vert in verts:
        if vert.is_boundary:
            continue
        cont = False
        for edge in vert.link_edges:
            if edge.seam:
                cont = True
                break
        if cont:
            continue
        final_co = None
        start = vert.co
        ray = vert.normal
        location1, normal, index, distance1 = bvh.ray_cast(start, ray)
        location2, normal, index, distance2 = bvh.ray_cast(start, -ray)
        location3, normal, index, distance3 = bvh.find_nearest(vert.co)
        if location1 and location2:
            final_co = location2 if distance2 < distance1 else location1

        elif location1:
            final_co = location1
            if location3:
                if distance3 * 1.2 < distance1:
                    final_co = location3
        elif location2:
            final_co = location2
            if location3:
                if distance3 * 1.2 < distance2:
                    final_co = location3
        else:
            if location3:
                final_co = location3

        if final_co:
            vert.co = final_co


def lerp(v, a, b):
    return (1 - v) * a + v * b


class SpatialHash:
    def __init__(self, cell_size=0.1):
        self.buckets = {}
        self.items = {}
        self.size = cell_size

    def get_key(self, location):
        return (
            round(location[0] / self.size),
            round(location[1] / self.size),
            round(location[2] / self.size)
        )

    def insert(self, item, key=None):
        if not key:
            key = self.get_key(item.co)
        if key in self.buckets:
            self.buckets[key].add(item)
        else:
            self.buckets[key] = {item, }
        self.items[item] = self.buckets[key]

    def remove(self, item):
        self.items[item].remove(item)
        del self.items[item]

    def update(self, item):
        self.remove(item)
        self.insert(item)

    def test_sphere(self, co, radius, exclude=()):
        radius_sqr = radius ** 2
        radius = radius / self.size
        location = co / self.size
        min_x = math.floor(location[0] - radius)
        max_x = math.ceil(location[0] + radius)
        min_y = math.floor(location[1] - radius)
        max_y = math.ceil(location[1] + radius)
        min_z = math.floor(location[2] - radius)
        max_z = math.ceil(location[2] + radius)
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                for z in range(min_z, max_z + 1):
                    key = (x, y, z)
                    if key in self.buckets:
                        for item in self.buckets[key]:
                            if (item.co - co).length_squared <= radius_sqr:
                                if item in exclude:
                                    continue
                                yield item

    def nearest_in_sphere(self, co, radius, exclude=()):
        try:
            yield min(self.test_sphere(co, radius, exclude=exclude), key=lambda i: (i.co - co).length_squared)
        except ValueError:
            pass


class Particle:
    def __init__(self, location, normal, bvh_tree=None):
        self.color = Vector((1, 0, 0, 1))
        self.radius = 0
        self.co = location
        self.normal = normal
        self.bvh = bvh_tree
        self.dir = Vector((1, 0, 0))
        self.field = None
        self.parent = None
        self.tag = "PARTICLE"
        self.tag_number = 0
        self.accumulation = location
        self.accumulation_counts = 1

    def add_location_sample(self, co, w=0.3):
        self.accumulation += co * w
        self.accumulation_counts += w
        self.co = self.accumulation / self.accumulation_counts


class SurfaceParticleSystem:
    def __init__(self, obj, model_size=1, resolution=60, mask_resolution=100):
        self.triangle_mode = False
        self.particles = set()
        self.field = vector_fields.Field(obj)

        self.draw = DrawCallback()
        self.draw.matrix = obj.matrix_world

        self.particle_size = model_size / resolution
        self.particle_size_mask = model_size / mask_resolution

        self.field_sampling_method = "RUNGE_KUTTA"

        self.grid = SpatialHash(self.particle_size * 2)

    def curvature_spawn_particles(self, n=10):
        d_sqr = self.particle_size ** 2
        verts = sorted(self.field.bm.verts, key=vector_fields.average_curvature)
        for i in range(n):
            vert = verts[i]
            not_valid = False
            for particle in self.particles:
                if (vert.co - particle.co).length_squared < d_sqr:
                    not_valid = True
                    break
            if not not_valid:
                self.new_particle(vert.co)

    def gp_spawn_particles(self, context):
        r = max(self.particle_size, self.particle_size_mask)
        mat = self.field.matrix.inverted()
        frame = vector_fields.get_gp_frame(context)
        if frame:
            for stroke in frame.strokes:
                for point in stroke.points:
                    co = mat @ point.co
                    valid = True
                    for particle in self.grid.test_sphere(co, r):
                        d = co - particle.co
                        if d.length < particle.radius:
                            valid = False
                    if valid:
                        p = self.new_particle(co)
                        p.tag = "GREASE"
                        p.color = Vector((0, 1, 0, 1))

    def singularity_spawn_particles(self):
        r = max(self.particle_size, self.particle_size_mask)
        for singularity in self.field.singularities:
            valid = True
            for particle in self.grid.test_sphere(singularity, r):
                d = particle.co - singularity
                if d.length < particle.radius * 3:
                    break
            if valid:
                self.new_particle(singularity)

    def sharp_edge_spawn_particles(self, source_bm, sharp_angle=0.523599):

        def sharp_particle_from_vert(vert):
            p = self.new_particle(vert.co)
            p.tag = "SHARP"
            p.normal = vert.normal
            p.dir = p.dir - p.normal * p.dir.dot(p.normal)
            p.color = Vector((0, 1, 0, 1))

        new_bm = bmesh.new()
        for edge in source_bm.edges:
            if edge.calc_face_angle(0) > sharp_angle or edge.is_boundary:
                verts = [new_bm.verts.new(vert.co) for vert in edge.verts]
                new_bm.edges.new(verts)
        bmesh.ops.remove_doubles(new_bm,
                                 verts=new_bm.verts,
                                 dist=min(self.particle_size, self.particle_size_mask) * 0.001)

        n = 10
        while True:
            subdivide = []
            for edge in new_bm.edges:
                center = (edge.verts[0].co + edge.verts[1].co) / 2
                location, normal, dir, s, c = self.field.sample_point(center)
                size = lerp(s, self.particle_size, self.particle_size_mask)
                if edge.calc_length() > size * 0.1:
                    subdivide.append(edge)
            if not subdivide or n <= 0:
                break
            n -= 1
            bmesh.ops.subdivide_edges(new_bm, edges=subdivide, cuts=1)

        if "out" in bpy.context.scene.objects:
            new_bm.to_mesh(bpy.context.scene.objects["out"].data)

        for vert in new_bm.verts:
            if vert.calc_edge_angle(0) > sharp_angle or len(vert.link_edges) > 2:
                sharp_particle_from_vert(vert)

        dir = Vector(np.random.sample((3,))).normalized()

        for vert in sorted(new_bm.verts, key=lambda v: v.co.dot(dir)):
            location, normal, dir, s, c = self.field.sample_point(vert.co)
            size = lerp(s, self.particle_size, self.particle_size_mask)
            valid = True
            for neighbor in self.grid.test_sphere(location, radius=size):
                valid = False
                break

            if valid:
                sharp_particle_from_vert(vert)

    def spread_particles(self, relaxation=3, factor=0.5):
        grid = self.grid
        current_front = list(self.particles)
        while len(current_front) > 0:
            yield
            new_front = []
            for particle in current_front:
                if particle.tag not in {"SHARP", "GREASE"}:
                    remove = False
                    for intruder in grid.test_sphere(particle.co, particle.radius * 0.7, exclude=(particle,)):
                        if intruder.tag in {"SHARP", "GREASE"}:
                            remove = True
                            break
                        elif (particle.co - intruder.co).length_squared < (particle.radius + intruder.radius) ** 2 / 10:
                            remove = True
                            break
                    if remove:
                        self.remove_particle(particle)
                        continue

                if self.triangle_mode:
                    vecs = vector_fields.hex_symmetry_space(particle.dir, particle.normal)
                    vecs = (vecs[0], vecs[1], vecs[4])
                else:
                    vecs = vector_fields.symmetry_space(particle.dir, particle.normal)
                    vecs = (vecs[0], vecs[1], vecs[3])

                for dir in vecs:
                    try:
                        if self.field_sampling_method == "EULER":
                            location, normal, dir, s, c = self.field.sample_point(particle.co + dir * particle.radius,
                                                                                  dir)

                        elif self.field_sampling_method == "MIDPOINT":
                            location, normal, dir, s, c = self.field.sample_point(
                                particle.co + dir * particle.radius * 0.3, dir)
                            n = normal * particle.radius * 0.1 * (1 if c > 0 else -1)

                            location, normal, dir2, s, c = self.field.sample_point(
                                n + particle.co + dir * particle.radius, dir)

                        elif self.field_sampling_method == "RUNGE_KUTTA":
                            location, normal, dir1, s, c = self.field.sample_point(
                                particle.co + dir * particle.radius * 0.3, dir)
                            n = normal * particle.radius * 0.1 * (1 if c > 0 else -1)

                            location, normal, dir2, s, c = self.field.sample_point(
                                n + particle.co + dir1 * particle.radius * 0.5, dir1)

                            location, normal, dir3, s, c = self.field.sample_point(
                                n + particle.co + dir2 * particle.radius, dir2)

                            dir = (dir + 2 * dir1 + 2 * dir2 + dir)
                            n = normal * particle.radius * 0.1 * (1 if c > 0 else -1)
                            location, normal, dir, s, c = self.field.sample_point(
                                n + particle.co + dir2 * particle.radius, dir)
                    except ValueError:
                        continue

                    valid = True
                    for neighbor in grid.test_sphere(location, particle.radius * 0.7, exclude=(particle,)):
                        if not neighbor.tag in {"SHARP", "GREASE"} and not neighbor is particle.parent:
                            # neighbor.co += location * 0.3
                            # neighbor.co /= 1.3
                            neighbor.add_location_sample(location, w=factor)
                            grid.update(neighbor)
                        valid = False
                        break

                    if valid:
                        p = self.new_particle(location, dir)
                        radius_diff = p.radius - particle.radius
                        if abs(radius_diff) > 0.5 * particle.radius:
                            p.radius = particle.radius * 1.5 if radius_diff > 0 else particle.radius * 0.5
                        p.parent = particle
                        grid.insert(p)
                        new_front.append(p)

                location, normal, dir, _, _ = self.field.sample_point(particle.co)
                particle.co = location
                particle.normal = normal
                particle.dir = dir
                grid.update(particle)
                if particle.tag_number < relaxation:
                    new_front.append(particle)
                    particle.tag_number += 1

            current_front = new_front

        # particles = list(self.particles)
        # for particle in particles:
        #     if particle.tag not in {"SHARP", "REMOVED"}:
        #         remove = False
        #         for intruder in grid.test_sphere(particle.co, particle.radius * 0.7, exclude=(particle,)):
        #             remove = True
        #             break
        #         if remove:
        #             self.remove_particle(particle)
        #             particle.tag = "REMOVED"

    def repeal_particles(self, iterations=20, factor=0.01):
        particles = list(self.particles)
        tree = KDTree(len(particles))
        for index, particle in enumerate(particles):
            tree.insert(particle.co, index)
        tree.balance()

        for i in range(iterations):
            new_tree = KDTree(len(self.particles))
            for index, particle in enumerate(particles):
                if particle.tag in {"SHARP", "GREASE"}:
                    continue

                d = Vector()

                for loc, other_index, dist in tree.find_n(particle.co, 5):
                    if dist == 0:
                        continue
                    other = particles[other_index]
                    vec = particle.co - other.co

                    d += (vec.normalized() / (dist * dist))

                    if not self.triangle_mode:
                        vecs = vector_fields.symmetry_space(particle.dir, particle.normal)
                        vec = vector_fields.best_matching_vector(vecs, vec)
                        vec *= (particle.radius + other.radius) / 2
                        # loc, _, _, _, _ = self.field.sample_point(particle.co + vec)
                        # vec = loc - particle.co

                        vec += other.co
                        vec -= particle.co
                        d += vec

                d.normalize()
                location, normal, dir, s, c = self.field.sample_point(particle.co + (d * factor * particle.radius))
                if location:
                    particle.co = location
                    particle.normal = normal
                    self.grid.update(particle)
                    particle.dir = dir

                new_tree.insert(particle.co, index)
            new_tree.balance()
            tree = new_tree

            yield i

    def mirror_particles(self, axis):
        particles = list(self.particles)
        for particle in particles:
            r = particle.radius * 0.5

            if -r * 0.7 <= particle.co[axis] <= r:
                particle.co[axis] = 0

            elif particle.co[axis] < 0:
                self.remove_particle(particle)

            else:
                mirror_co = particle.co.copy()
                mirror_co[axis] *= -1
                self.new_particle(mirror_co)

    def new_particle(self, location, dir=None):
        location, normal, dir, s, c = self.field.sample_point(location, dir)
        particle = Particle(location, normal, self.field.bvh)
        particle.dir = dir
        particle.radius = lerp(s, self.particle_size, self.particle_size_mask)
        self.particles.add(particle)
        self.grid.insert(particle)
        return particle

    def remove_particle(self, particle):
        self.particles.remove(particle)
        self.grid.remove(particle)
        particle.tag = "REMOVED"

    def draw_particles(self, relaxation_steps=3):
        self.draw.clear_data()
        self.draw.point_size = 8
        for particle in self.particles:
            self.draw.add_point(particle.co,
                                particle.color * (particle.tag_number / relaxation_steps))
        self.draw.update_batch()

    def create_mesh(self, bm, sharp_angle=0.52):

        bmesh.ops.triangulate(bm, faces=bm.faces)
        source_bvh = BVHTree.FromBMesh(bm)

        mask_layer = bm.verts.layers.paint_mask.verify()
        n = 5
        while True:
            subdivide_edges = []
            for edge in bm.edges:
                le = edge.calc_length()

                s = (edge.verts[0][mask_layer] + edge.verts[1][mask_layer]) / 2
                target_le = lerp(s, self.particle_size, self.particle_size_mask)
                if target_le * 0.5 <= le:
                    subdivide_edges.append(edge)
            print(len(subdivide_edges))
            print("subdivide", n)
            n -= 1
            if not subdivide_edges or n < 0:
                break
            print("subdivide")
            bmesh.ops.subdivide_edges(bm, edges=subdivide_edges, cuts=1, use_grid_fill=True, use_only_quads=True)
            bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="SHORT_EDGE")
            bmesh.ops.beautify_fill(bm, edges=bm.edges, faces=bm.faces, method="AREA")
        print("done")

        # ==========================================================================================

        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        n = len(bm.verts)

        bvh = BVHTree.FromBMesh(bm)

        sharp = 20
        smooth = 10

        particles = np.array([particle.co for particle in self.particles], dtype=np.float64, ndmin=2)
        weights = np.array([smooth if particle.tag == "SHARP" else sharp for particle in self.particles], dtype=np.int8)
        locations = np.array([vert.co for vert in bm.verts], dtype=np.float64, ndmin=2)
        particles_mapping = np.full((n,), -1, dtype=np.int64)

        current_front = set()
        for i in range(len(self.particles)):
            co = particles[i]
            location, normal, index, dist = bvh.find_nearest(co)

            if location:
                vert = min(bm.faces[index].verts,
                           key=lambda v: (v.co - Vector(co)).length_squared * (
                               2 if particles_mapping[v.index] == -1 else 1))
                vert.tag = True
                particles_mapping[vert.index] = i
                current_front.add(vert)

        while current_front:
            new_front = set()
            for vert in current_front:
                for edge in vert.link_edges:
                    other = edge.other_vert(vert)
                    if not other.tag:
                        new_front.add(other)
                        particles_mapping[other.index] = particles_mapping[vert.index]
                        other.tag = True
            current_front = new_front

        edges_limit = 10
        edges = np.empty((n, edges_limit), dtype=np.int64)
        edges_count = np.empty((n,), dtype=np.int64)

        for vert in bm.verts:
            edges_count[vert.index] = min(edges_limit, len(vert.link_edges))
            for i, edge in enumerate(vert.link_edges):
                if i >= edges_limit:
                    break
                other = edge.other_vert(vert)
                edges[vert.index][i] = other.index

        ids = np.arange(n)
        for i in range(30):
            cols = np.random.randint(0, edges_limit) % edges_count
            edge_indexes = edges[ids, cols]
            edge_mappings = particles_mapping[edge_indexes]
            distance = ((particles[particles_mapping] - locations) ** 2).sum(axis=1) * weights[particles_mapping]
            edge_distance = ((particles[edge_mappings] - locations) ** 2).sum(axis=1) * weights[edge_mappings]
            particles_mapping = np.where(edge_distance > distance, particles_mapping, edge_mappings)

        # ==========================================================================================

        new_bm = bmesh.new()

        # ==========================================================================================

        verts = [new_bm.verts.new(co) for co in particles]
        for index, particle in enumerate(self.particles):
            if particle.tag == "SHARP":
                verts[index].tag = True

        for face in bm.faces:
            particles_indexes = set(particles_mapping[vert.index] for vert in face.verts)
            if len(particles_indexes) == 3:
                try:
                    new_bm.faces.new((verts[i] for i in particles_indexes))
                except ValueError:
                    pass
        bmesh.ops.recalc_face_normals(new_bm, faces=new_bm.faces)

        # ==========================================================================================

        for i in range(50):
            stop = True
            for vert in new_bm.verts:
                le = len(vert.link_edges)
                if le < 3:
                    new_bm.verts.remove(vert)
                    stop = False

            for edge in new_bm.edges:
                if len(edge.link_faces) < 2:
                    new_bm.edges.remove(edge)
                    stop = False
            bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=min(self.particle_size, self.particle_size_mask) * 0.1)
            bmesh.ops.holes_fill(new_bm, edges=new_bm.edges)
            bmesh.ops.triangulate(new_bm, faces=new_bm.faces, quad_method="SHORT_EDGE")
            if stop:
                break

        bvh_snap(source_bvh, bm.verts)

        bmesh.ops.holes_fill(new_bm, edges=new_bm.edges)
        bmesh.ops.triangulate(new_bm, faces=new_bm.faces)
        bmesh.ops.recalc_face_normals(new_bm, faces=new_bm.faces)

        # ==========================================================================================

        if sharp_angle < math.pi:
            crease = new_bm.edges.layers.crease.verify()
            for edge in new_bm.edges:
                if edge.calc_face_angle(0) > sharp_angle:
                    edge[crease] = 1.0
                    edge.seam = True

        # ==========================================================================================

        if not self.triangle_mode:
            for i in range(2):
                stop = True
                bmesh.ops.join_triangles(new_bm, faces=new_bm.faces,
                                         angle_face_threshold=3.14,
                                         angle_shape_threshold=3.14,
                                         cmp_seam=True)
                relax_topology(new_bm)
                bvh_snap(source_bvh, new_bm.verts)

                # dissolve = []
                # for vert in new_bm.verts:
                #     le = len(vert.link_edges)
                #     one_ring_signature = tuple(sorted(len(face.verts) for face in vert.link_faces))
                #
                #     if le == 3 and one_ring_signature in ((3, 4, 4), (3, 3, 4)):
                #         dissolve.append(vert)
                #         stop = False
                # bmesh.ops.dissolve_verts(new_bm, verts=dissolve)

                # subdivide_edges = []
                # for vert in new_bm.verts:
                #     le = len(vert.link_edges)
                #     one_ring_signature = tuple(sorted(len(face.verts) for face in vert.link_faces))
                #
                #     if le > 5 and one_ring_signature.count(3) > 1:
                #         longest_edge = max(vert.link_edges,
                #                            key=lambda e: (e.verts[0].co - e.verts[1].co).length_squared)
                #         subdivide_edges.append(longest_edge)
                #         stop = False

                # bmesh.ops.subdivide_edges(new_bm, edges=list(set(subdivide_edges)))
                bmesh.ops.triangulate(new_bm, faces=new_bm.faces, quad_method="SHORT_EDGE")

                if stop:
                    break

            bmesh.ops.join_triangles(new_bm, faces=new_bm.faces, angle_face_threshold=sharp_angle,
                                     angle_shape_threshold=3.14,
                                     cmp_seam=True)

            # ==========================================================================================

            # merge_hints = {
            #     (5, 3, 5, 3), (3, 5, 3, 5),
            #     (4, 3, 4, 3), (3, 4, 3, 4),
            #     (5, 3, 4, 3), (3, 5, 3, 4), (4, 3, 5, 3), (3, 4, 3, 5)
            # }
            # merge_map = {}
            # seen_verts = set()
            # for face in new_bm.faces:
            #     face_signature = tuple((len(vert.link_edges) for vert in face.verts))
            #     if face_signature in merge_hints:
            #         loop = [loop for loop in face.loops if len(loop.vert.link_edges) == 3][0]
            #         vert0 = loop.vert
            #         vert1 = loop.link_loop_next.link_loop_next.vert
            #         co = (vert0.co + vert1.co) / 2
            #         vert0.co, vert1.co = co, co
            #         verts = {vert0, vert1}
            #         if not verts & seen_verts:
            #             seen_verts |= verts
            #             merge_map[vert0] = vert1
            # bmesh.ops.weld_verts(new_bm, targetmap=merge_map)

            # ==========================================================================================

            # faces = []
            # for face in new_bm.faces:
            #     tri_count = 0
            #     for edge in face.edges:
            #         for possible_tri in edge.link_faces:
            #             if possible_tri is not face and len(possible_tri.verts) == 3:
            #                 tri_count += 1
            #     if tri_count == 3:
            #         faces.append(face)
            # bmesh.ops.poke(new_bm, faces=faces)
            # bmesh.ops.join_triangles(new_bm, faces=new_bm.faces, angle_face_threshold=sharp_angle,
            #                          angle_shape_threshold=3.14,
            #                          cmp_seam=True)

            # ==========================================================================================

            # collapse = []
            # seen_verts = set()
            # for edge in new_bm.edges:
            #     tri_count = 0
            #     for face in edge.link_faces:
            #         if len(face.verts) == 3:
            #             tri_count += 1
            #     if tri_count == 2:
            #         verts = set(edge.verts)
            #         if not verts & seen_verts:
            #             collapse.append(edge)
            #             seen_verts |= verts
            # bmesh.ops.collapse(new_bm, edges=collapse)

        # ==========================================================================================

        relax_topology(new_bm)
        bvh_snap(source_bvh, new_bm.verts)
        return new_bm, source_bvh


@register_class
class ParticleRemesh(bpy.types.Operator):
    bl_idname = "tesselator.remesh"
    bl_label = "Particle Remesh"
    bl_description = "Rebuilds the mesh by simulating a particle system."
    bl_options = {"REGISTER", "UNDO"}
    _timer = None

    particle_relaxation: bpy.props.FloatProperty(
        name="Relaxation Factor",
        description="Relaxates points after placement",
        min=0.0001,
        max=2,
        default=0.5
    )

    relaxation_steps: bpy.props.IntProperty(
        name="Relaxation Steps",
        description="Amount of smoothing steps applied to the particles.",
        min=1,
        default=3
    )

    resolution: bpy.props.FloatProperty(
        name="Resolution",
        description="Distance between particles relative to object size",
        min=0.0001,
        default=45
    )

    mask_resolution: bpy.props.FloatProperty(
        name="Mask Resolution",
        description="Distance between particles relative to object size in masked areas\n"
                    "Notice, Particle size will also interpolate between smooth mask values",
        min=0.0001,
        default=100
    )

    mirror_axes: bpy.props.BoolVectorProperty(
        name="Mirror Axes",
        description="Mirror field and particles at specified axes.",
        size=3,
        default=(False, False, False)
    )

    subdivisions: bpy.props.IntProperty(
        name="Subdivisions",
        description="Amount of subudivisions applied to the final mesh",
        min=0,
        default=1
    )

    gp_influence: bpy.props.FloatProperty(
        name="Annotation Influence",
        description="How much annotations affect the resulting direction",
        min=0,
        max=1,
        default=0.2
    )

    field_resolution: bpy.props.IntProperty(
        name="Field Resolution",
        description="Maximum amount of verts the guiding field should have.",
        min=100,
        default=8000
    )

    field_sampling_method: bpy.props.EnumProperty(
        name="Field Sampling Method",
        description="Precision level of sampling",
        items=[
            ("EULER", "Euler", "Super fast and simple"),
            ("MIDPOINT", "Midpoint", "Fast and precise"),
            ("RUNGE_KUTTA", "Runge-Kutta Order 4", "Slow but super precise")
        ],
        default="MIDPOINT"
    )

    polygon_mode: bpy.props.EnumProperty(
        name="Polygon Mode",
        description="What kind of polygon to tesselate",
        items=[("TRI", "Triangles", "Pure Triangles"),
               ("QUADS", "Quads", "Pure Quadrilaterals"),
               ("TRI_AND_QUADS", "Triangles and Quads", "Remesh with quadrilaterals bud add triangle bifurcations")],
        default="QUADS"
    )

    field_smoothing_iterations: bpy.props.IntVectorProperty(
        name="Repeat",
        description="Amount of smoothing iterations for each round,",
        size=3,
        min=0,
        default=(30, 30, 100),
    )

    field_smoothing_depth: bpy.props.IntVectorProperty(
        name="Distance",
        description="Amount of random walk steps used to average the field for each round.",
        size=3,
        min=0,
        default=(100, 5, 0)
    )

    sharp_angle: bpy.props.FloatProperty(
        name="Sharp Angle",
        description="Snaps edge to sharp features based on face angles (180° = Disable)",
        default=math.pi,
        min=0,
        max=math.pi,
        subtype="ANGLE"
    )

    def draw(self, context):
        layout = self.layout
        row = layout.row(align=True)
        row.prop_enum(self, "polygon_mode", value="QUADS")
        row.prop_enum(self, "polygon_mode", value="TRI")
        row.prop_enum(self, "polygon_mode", value="TRI_AND_QUADS")
        layout.prop(self, "sharp_angle")
        layout.separator()

        layout.prop(self, "resolution")
        layout.prop(self, "mask_resolution")
        layout.prop(self, "subdivisions")
        layout.separator()

        layout.prop(self, "particle_relaxation", slider=True)
        layout.prop(self, "relaxation_steps")
        layout.prop(self, "field_sampling_method")
        layout.separator()

        row = layout.row()
        split = row.split(factor=0.5)
        split.label(text="Symmetry Axes")
        row = split.row(align=True)
        row.prop(self, "mirror_axes", text="X", index=0, toggle=True)
        row.prop(self, "mirror_axes", text="Y", index=1, toggle=True)
        row.prop(self, "mirror_axes", text="Z", index=2, toggle=True)
        layout.separator()

        layout.label(text="Cross Field Smoothing")
        layout.prop(self, "gp_influence", slider=True)
        layout.prop(self, "field_resolution")
        row0 = layout.row(align=True)
        for text in ("", "Round 1", "Round 2", "Round 3"):
            row0.label(text=text)
        row1 = layout.row(align=True)
        row2 = layout.row(align=True)
        row1.label(text="Repeat")
        row2.label(text="Distance")
        for i in range(3):
            row1.prop(self, "field_smoothing_iterations", index=i, text="")
            row2.prop(self, "field_smoothing_depth", index=i, text="")

    def algorithm(self, context):
        obj = context.active_object
        bm = bmesh.new()
        bm.from_mesh(obj.data)

        for vert in bm.verts:
            if len(vert.link_faces) < 1:
                bm.verts.remove(vert)
        bmesh.ops.holes_fill(bm, edges=bm.edges)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        bm.to_mesh(obj.data)

        DebugText.lines = ["Decimating mesh."]
        yield

        model_size = max(context.active_object.dimensions)

        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.decimate(ratio=self.field_resolution / len(bm.verts))
        bpy.ops.object.mode_set(mode="OBJECT")
        yield
        self.particle_manager = SurfaceParticleSystem(context.active_object, model_size, self.resolution,
                                                      self.mask_resolution)

        self.particle_manager.field_sampling_method = self.field_sampling_method
        self.particle_manager.triangle_mode = (self.polygon_mode == "TRI")
        self.particle_manager.field.hex_mode = (self.polygon_mode == "TRI")
        self.particle_manager.field.draw.setup_handler()
        self.particle_manager.draw.setup_handler()
        self.particle_manager.field.preview_fast()
        yield

        if self.gp_influence > 0:
            self.particle_manager.field.initialize_from_gp(context)
            self.particle_manager.field.weights /= self.gp_influence
            self.particle_manager.field.weights = self.particle_manager.field.weights.clip(0, 1)
            self.particle_manager.gp_spawn_particles(context)

        for i in range(3):
            self.particle_manager.field.smooth(self.field_smoothing_iterations[i], self.field_smoothing_depth[i])

            for axis in range(3):
                if self.mirror_axes[axis]:
                    self.particle_manager.field.mirror(axis)

            DebugText.lines = ["Creating Cross Field",
                               f"Step: {i + 1}"]
            yield

        self.particle_manager.field.preview()

        if self.sharp_angle < math.pi:
            self.particle_manager.sharp_edge_spawn_particles(bm, self.sharp_angle)

        if len(self.particle_manager.particles) == 0:
            self.particle_manager.field.detect_singularities()
            self.particle_manager.singularity_spawn_particles()

        if len(self.particle_manager.particles) == 0:
            self.particle_manager.curvature_spawn_particles(5)

        for i, _ in enumerate(self.particle_manager.spread_particles(self.relaxation_steps, self.particle_relaxation)):
            self.particle_manager.draw_particles(self.relaxation_steps)
            DebugText.lines = [f"Propagating particles {('.', '..', '...')[i % 3]}"]
            yield

        # for i, _ in enumerate(self.particle_manager.repeal_particles(iterations=self.relaxation_steps,
        #                                                              factor=self.particle_relaxation)):
        #     self.particle_manager.draw_particles()
        #     DebugText.lines = ["Particle relaxation:",
        #                        f"Step {i + 1}"]
        #     yield

        for i in range(3):
            if self.mirror_axes[i]:
                self.particle_manager.mirror_particles(axis=i)
                print("Mirror")
        self.particle_manager.draw_particles()

        DebugText.lines = ["Tesselating."]
        yield

        bm, bvh = self.particle_manager.create_mesh(bm, self.sharp_angle)
        if self.polygon_mode == "QUADS":
            bm.to_mesh(obj.data)
            yield

            for i in range(self.subdivisions):
                md = context.active_object.modifiers.new(type="SUBSURF", name="Subd")
                md.levels = 1
                md.subdivision_type = "SIMPLE" if i == 0 else "CATMULL_CLARK"
                bpy.ops.object.modifier_apply(modifier=md.name)
                bm = bmesh.new()
                bm.from_mesh(obj.data)
                if i == 0:
                    straigthen_quad_topology(bm)
                    relax_topology(bm)
                bvh_snap(bvh, bm.verts)
                bm.to_mesh(obj.data)

        elif self.polygon_mode == "TRI_AND_QUADS":
            for i in range(self.subdivisions):
                subdivide_split_triangles(bm)
                # relax_topology(bm)
                bvh_snap(bvh, bm.verts)
            bm.to_mesh(obj.data)

        else:
            for i in range(self.subdivisions):
                bmesh.ops.triangulate(bm, faces=bm.faces)
                bmesh.ops.subdivide_edges(bm, edges=bm.edges, cuts=1, use_grid_fill=True)
                relax_topology(bm)
                bvh_snap(bvh, bm.verts)
            bm.to_mesh(obj.data)

        yield True

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == "MESH"

    def execute(self, context):
        self.stepper = self.algorithm(context)
        context.window_manager.modal_handler_add(self)
        self._timer = context.window_manager.event_timer_add(0.05, window=context.window)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        finished = False
        if event.type == "TIMER":
            finished = next(self.stepper)
            context.area.tag_redraw()
            if not finished:
                return {"RUNNING_MODAL"}

        if event.type in {"RIGHTMOUSE", "ESC"} or finished:
            self.particle_manager.draw.remove_handler()
            self.particle_manager.field.draw.remove_handler()
            context.window_manager.event_timer_remove(self._timer)
            DebugText.lines = {}
            return {"CANCELLED"}

        return {"PASS_THROUGH"}
