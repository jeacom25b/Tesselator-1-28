import bpy
import math
from .multifile import register_class, register_function, unregister_function


@register_class
class TesselatorSettings(bpy.types.PropertyGroup):
    particle_relaxation: bpy.props.FloatProperty(
        name="Relaxation Factor",
        description="Relaxates points after placement",
        min=0.0001,
        max=0.5,
        default=0.05
    )

    relaxation_steps: bpy.props.IntProperty(
        name="Relaxation Steps",
        description="Amount of smoothing steps applied to the particles.",
        min=0,
        default=4
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
        default=(20, 5, 0)
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
