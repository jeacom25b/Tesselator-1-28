import bpy
import importlib
import traceback
import os
import sys

_imported_modules = []
_modules = []
_register_classes = set()
_register_functions = set()
_unregister_functions = set()


def add_module(module_name):
    _modules.append(module_name)


def register_class(cls):
    '''Decorator'''
    _register_classes.add(cls)
    return cls


def register_function(func,):
    _register_functions.add(func)
    return func


def unregister_function(func):
    _unregister_functions.add(func)
    return func

def import_modules():
    for imported_module in _imported_modules:
        importlib.reload(imported_module)
        print("reloaded", imported_module)

    for module_name in _modules:
        try:
            fake_globals = {"__name__": __name__}
            exec(f"from . import {module_name}", fake_globals)
            _imported_modules.append(fake_globals[module_name])

        except Exception as e:
            raise e

def register():
    for item in _register_classes:
        bpy.utils.register_class(item)

    for item in _register_functions:
        item()

def unregister():
    for item in _register_classes:
        bpy.utils.unregister_class(item)

    for item in _unregister_functions:
        item()
