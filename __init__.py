bl_info = {
    "name": "Tesselator 2",
    "description": "Tesselator for blender 2.8",
    "author": "Jean Da Costa Machado",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "wiki_url": "",
    "category": "category",
    "location": "location"}

from .multifile import add_module, register, unregister, import_modules

add_module("draw_3d")
add_module("vector_fields")
add_module("surface_particles")
add_module("interface")
import_modules()
