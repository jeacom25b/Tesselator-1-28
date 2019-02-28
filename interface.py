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
from .multifile import register_class, register_function, unregister_function


class DebugText:
    lines = []

    @classmethod
    def draw(cls, layout):
        for line in cls.lines:
            layout.label(text=line)


@register_class
class TesselatorSettings(bpy.types.PropertyGroup):
    particle_relaxation: bpy.props.FloatProperty(
        name="Relaxation Factor",
        description="Relaxates points after placement",
        min=0.0001,
        max=2,
        default=1
    )

    relaxation_steps: bpy.props.IntProperty(
        name="Relaxation Steps",
        description="Amount of smoothing steps applied to the particles.",
        min=1,
        default=2
    )

    repulsion_iterations: bpy.props.IntProperty(
        name="Repulsion Iterations",
        description="How many times to repeal particles to keep a uniform distribution",
        min=0,
        default=10
    )

    repulsion_strength: bpy.props.FloatProperty(
        name = "Repulstion_strength",
        description="How much to repeal particles in each iteration",
        min = 0.00001,
        max = 1.0,
        default = 0.05
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
        description="Maximum amount of verts the guiding field should have. Increase to make topology more complex",
        min=100,
        default=8000
    )

    field_sampling_method: bpy.props.EnumProperty(
        name="Field Sampling Method",
        description="Precision level of sampling",
        items=[
            ("EULER", "Euler", "Better suited to simpler meshes, very fast."),
            ("MIDPOINT", "Midpoint", "General purpose, slightly slower than Euler"),
            ("RUNGE_KUTTA", "Runge-Kutta Order 4", "Slow but snaps particles to edges and aligns better the topology,"
                                                   "better suited for preserving sharp features.")
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
        description="Amount of smoothing iterations for each round, the higher the values, the more smooth"
                    "the results will be",
        size=3,
        min=0,
        default=(30, 30, 100),
    )

    field_smoothing_depth: bpy.props.IntVectorProperty(
        name="Distance",
        description="Amount of random walk steps used to average the field for each round, higher values yield less"
                    "complex curves",
        size=3,
        min=0,
        default=(100, 30, 0)
    )

    sharp_angle: bpy.props.FloatProperty(
        name="Sharp Angle",
        description="Snaps edge to sharp features based on face angles (180° = Disable)",
        default=math.pi,
        min=0,
        max=math.pi,
        subtype="ANGLE"
    )


@register_class
class TESSELATOR_PT_Panel(bpy.types.Panel):
    bl_idname = "Tesselator.panel"
    bl_label = "Particle Remesh"
    bl_category = "Tesselator"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        settings = context.scene.tesselator_settings
        layout = self.layout

        if DebugText.lines:
            DebugText.draw(layout)
            return

        op = layout.operator("tesselator.remesh")
        op.polygon_mode = settings.polygon_mode
        op.sharp_angle = settings.sharp_angle
        op.resolution = settings.resolution
        op.mask_resolution = settings.mask_resolution
        op.subdivisions = settings.subdivisions
        op.mirror_axes = settings.mirror_axes
        op.field_sampling_method = settings.field_sampling_method
        op.relaxation_steps = settings.relaxation_steps
        op.particle_relaxation = settings.particle_relaxation
        op.repulsion_iterations = settings.repulsion_iterations
        op.repulsion_strength = settings.repulsion_strength
        op.gp_influence = settings.gp_influence
        op.field_resolution = settings.field_resolution
        op.field_smoothing_depth = settings.field_smoothing_depth
        op.field_smoothing_iterations = settings.field_smoothing_iterations

        # box = layout.box()
        col = layout.column(align=True)
        col.label(text="Polygon Mode")
        col.prop_enum(settings, "polygon_mode", value="QUADS")
        col.prop_enum(settings, "polygon_mode", value="TRI")
        col.prop_enum(settings, "polygon_mode", value="TRI_AND_QUADS")
        col = layout.column(align=True)
        col.prop(settings, "sharp_angle")
        col.separator()

        col.label(text="Resolutions")
        col.prop(settings, "resolution")
        col.prop(settings, "mask_resolution")
        col.prop(settings, "subdivisions")
        col.separator()

        col.label(text="Symmetry")
        row = col.row(align=True)
        for i, text, in enumerate("XYZ"):
            row.prop(settings, "mirror_axes", index=i, text=text, toggle=True)
        col.separator()

        col.label(text="Particle Dynamics")
        col.prop(settings, "field_sampling_method")
        col = layout.column(align=True)
        col.prop(settings, "relaxation_steps")
        col.prop(settings, "particle_relaxation")
        col.prop(settings, "repulsion_iterations")
        col.prop(settings, "repulsion_strength")
        col.separator()

        col.label(text="Guiding Fields")
        col.prop(settings, "gp_influence", slider=True)
        col.prop(settings, "field_resolution")
        col.label(text="Field Smoothing")
        row = col.row()
        for text in ("iteration", "Distance", "Repeat"):
            row.label(text=text)
        for i, text in enumerate("123"):
            row = col.row()
            row.label(text=text)
            row.prop(settings, "field_smoothing_depth", text="", index=i)
            row.prop(settings, "field_smoothing_iterations", text="", index=i)


@register_function
def register():
    bpy.types.Scene.tesselator_settings = bpy.props.PointerProperty(type=TesselatorSettings)


@unregister_function
def unregister():
    del bpy.types.Scene.tesselator_settings
