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

bl_info = {
    "name": "Tesselator 1.28.0",
    "description": "Tesselator for blender 2.8",
    "author": "Jean Da Costa Machado",
    "version": (1, 28, 0),
    "blender": (2, 80, 0),
    "wiki_url": "",
    "category": "Sculpt",
    "location": "3D view > Properties Panel > Tesselator"}

from .multifile import add_module, register, unregister, import_modules

add_module("draw_3d")
add_module("vector_fields")
add_module("surface_particles")
add_module("interface")
import_modules()
