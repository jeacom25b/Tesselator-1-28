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

        if DebugText.lines:
            DebugText.draw(layout)
            return


@register_function
def register():
    bpy.types.Scene.tesselator_settings = bpy.props.PointerProperty(type=TesselatorSettings)


@unregister_function
def unregister():
    del bpy.types.Scene.tesselator_settings
