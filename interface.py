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
from .field import Field
from .draw_3d import DrawCallback
from mathutils import Vector


class DebugText:
    lines = []

    @classmethod
    def draw(cls, layout):
        for line in cls.lines:
            layout.label(text=line)


@register_class
class TesselatorSettings(bpy.types.PropertyGroup):
    pass


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
        layout.operator('tesselator.remesh')

        if DebugText.lines:
            DebugText.draw(layout)
            return

@register_class
class TESSELATOR_OT_remesh(bpy.types.Operator):
    bl_idname = 'tesselator.remesh'
    bl_label = 'Particle Remesh'
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        ob = bpy.context.active_object
        self.field = Field(ob)
        self.field.reproject()
        self.draw = DrawCallback()
        self.draw.setup_handler()


        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type == 'RET' and event.value == 'PRESS':
            for _ in range(10):
                self.field.smooth_field()

            self.draw.clear_data()
            for v, f in zip(self.field.field, self.field.bm.faces):
                v = Vector(v)
                center = f.calc_center_median()
                self.draw.add_line(center + f.normal * 0.01 + f.normal * 0.01 + v * 0.1, center + f.normal * 0.01)

            self.draw.update_batch()
            return {'RUNNING_MODAL'}

        if event.type == 'ESC':
            self.draw.remove_handler()
            return {'FINISHED'}

        return {'PASS_THROUGH'}





@register_function
def register():
    bpy.types.Scene.tesselator_settings = bpy.props.PointerProperty(type=TesselatorSettings)


@unregister_function
def unregister():
    del bpy.types.Scene.tesselator_settings
