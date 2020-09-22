import numpy as np
import bpy
import bmesh
from mathutils.bvhtree import BVHTree
from mathutils import Vector
from itertools import product


def norm(arr):
    n = (arr * arr).sum(axis=-1)
    n.shape += (1,)
    return np.sqrt(n)


def normalized(arr):
    return arr / norm(arr)


def dot(arr1, arr2):
    s = (arr1 * arr2).sum(axis=-1)
    s.shape += (1,)
    return s


class Field:
    def __init__(self, object):
        self.bm = bmesh.new()
        self.bm.from_mesh(object.data)
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)
        self.n = len(self.bm.faces)

        self.tree = BVHTree.FromBMesh(self.bm)
        self.field = np.random.sample((self.n, 3)) - 0.5
        self.normals = np.zeros(self.field.shape, dtype=np.float_)
        self.adjacency = np.zeros(self.field.shape, dtype=np.int_)
        self.adjacency[:] = -1

        self.bm.faces.ensure_lookup_table()

        for face in self.bm.faces:
            for i, edge in enumerate(face.edges):
                for other_face in edge.link_faces:
                    if not other_face == face:
                        break
                self.adjacency[face.index, i] = other_face.index

            n = Vector()
            for vert in face.verts:
                n += vert.normal

            self.normals[face.index] = n
        self.normals /= norm(self.normals)

    def reproject(self):
        self.field -= self.normals * dot(self.field, self.normals)
        self.field /= norm(self.field)

    def best_combination(self, fa, fb):
        angs = []
        sums = []

        for a, b in product(fa, fb):
            d = dot(a, b)
            d.shape = d.shape[:1]
            angs.append(d)
            sums.append(a + b)

        ang_stack = np.stack(angs, axis=1)
        sum_stack = np.stack(sums, axis=1)

        n = fa[0].shape[0]
        best_idx = np.argmax(ang_stack, axis=1)
        return sum_stack[range(n), best_idx]


    def smooth_field(self, adjacemcy_depth=10):
        selected_adjacency = range(self.n)
        for _ in range(adjacemcy_depth):
            random_adjacent = np.random.randint(3, size=len(self.bm.faces), dtype=np.int_)
            selected_adjacency = self.adjacency[selected_adjacency, random_adjacent]

        adj_a = self.field[selected_adjacency]
        adj_normals = self.normals[selected_adjacency]
        adj_b = np.cross(adj_a, adj_normals)

        fa = (self.field, np.cross(self.field, self.normals))
        fb = (adj_a, adj_b, -adj_a, -adj_b)

        self.field = self.best_combination(fa, fb)
        self.reproject()
