import os
import traceback

import bpy
from bpy.props import BoolProperty, CollectionProperty, StringProperty
from bpy_extras.io_utils import ExportHelper, ImportHelper

from . import export_pmesh, import_panim, import_pmesh
from .model import ConvertError


class EXPORT_SCENE_OT_prisma3d(bpy.types.Operator, ExportHelper):
    bl_idname = "export_scene.prisma3d"
    bl_label = "Export prisma3d (.pmesh)"
    bl_description = "Export the scene to the prisma3d mesh, skeleton, animation and material files"
    bl_options = {"PRESET"}

    filename_ext = ".pmesh"

    filter_glob: StringProperty(default="*.pmesh", options={"HIDDEN"}, maxlen=255)

    use_selection: BoolProperty(
        name="Selection Only",
        description="Export the selected objects only",
        default=False,
    )
    export_apply: BoolProperty(
        name="Apply Modifiers",
        description="Apply the modifiers (except armatures) to the mesh objects",
        default=True,
    )
    export_animations: BoolProperty(
        name="Animations",
        description="Export one .panim file per action",
        default=True,
    )
    export_cameras: BoolProperty(
        name="Cameras",
        description="Export the cameras",
        default=True,
    )

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(self, "use_selection")
        layout.prop(self, "export_apply")
        layout.prop(self, "export_animations")
        layout.prop(self, "export_cameras")

    def execute(self, context):
        options = export_pmesh.Options(
            selection_only=self.use_selection,
            apply_modifiers=self.export_apply,
            animations=self.export_animations,
            cameras=self.export_cameras,
        )
        if context.object is not None and context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        try:
            result = export_pmesh.export_files(self.filepath, options)
        except ConvertError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        except OSError as error:
            self.report({"ERROR"}, "could not write the files: %s" % error)
            return {"CANCELLED"}
        except Exception as error:
            traceback.print_exc()
            self.report({"ERROR"}, "unexpected failure: %s" % error)
            return {"CANCELLED"}
        for warning in result.warnings:
            self.report({"WARNING"}, warning)
        self.report({"INFO"}, "prisma3d: wrote %d files" % len(result.files))
        return {"FINISHED"}


class IMPORT_SCENE_OT_prisma3d(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.prisma3d"
    bl_label = "Import prisma3d (.pmesh)"
    bl_description = "Import a prisma3d mesh with its skeleton, animation and material files"
    bl_options = {"PRESET", "UNDO"}

    filename_ext = ".pmesh"

    filter_glob: StringProperty(default="*.pmesh", options={"HIDDEN"}, maxlen=255)

    def execute(self, context):
        if context.object is not None and context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        try:
            result = import_pmesh.import_pmesh(self.filepath)
        except ConvertError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        except OSError as error:
            self.report({"ERROR"}, "could not read the files: %s" % error)
            return {"CANCELLED"}
        except Exception as error:
            traceback.print_exc()
            self.report({"ERROR"}, "unexpected failure: %s" % error)
            return {"CANCELLED"}
        for warning in result.warnings:
            self.report({"WARNING"}, warning)
        return {"FINISHED"}


class IMPORT_SCENE_OT_prisma3d_animations(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.prisma3d_animations"
    bl_label = "Import prisma3d animations (.panim)"
    bl_description = "Add prisma3d animation clips to the selected armature"
    bl_options = {"PRESET", "UNDO"}

    filename_ext = ".panim"

    filter_glob: StringProperty(default="*.panim", options={"HIDDEN"}, maxlen=255)
    files: CollectionProperty(type=bpy.types.OperatorFileListElement, options={"HIDDEN", "SKIP_SAVE"})
    directory: StringProperty(subtype="DIR_PATH", options={"HIDDEN"})

    def execute(self, context):
        armature = import_panim.find_armature(context)
        if armature is None:
            self.report({"ERROR"}, "select the armature that receives the animations")
            return {"CANCELLED"}
        if self.files:
            paths = sorted(os.path.join(self.directory, entry.name) for entry in self.files)
        else:
            paths = [self.filepath]
        if context.object is not None and context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        try:
            result = import_panim.import_animations(context, paths, armature)
        except ConvertError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        except OSError as error:
            self.report({"ERROR"}, "could not read the files: %s" % error)
            return {"CANCELLED"}
        except Exception as error:
            traceback.print_exc()
            self.report({"ERROR"}, "unexpected failure: %s" % error)
            return {"CANCELLED"}
        for warning in result.warnings:
            self.report({"WARNING"}, warning)
        self.report({"INFO"}, "prisma3d: added %d animations to '%s'" % (result.count, armature.name))
        return {"FINISHED"}


def menu_func_import(self, context):
    self.layout.operator(IMPORT_SCENE_OT_prisma3d.bl_idname, text="prisma3d (.pmesh)")
    self.layout.operator(IMPORT_SCENE_OT_prisma3d_animations.bl_idname, text="prisma3d animations (.panim)")


def menu_func_export(self, context):
    self.layout.operator(EXPORT_SCENE_OT_prisma3d.bl_idname, text="prisma3d (.pmesh)")


classes = (
    EXPORT_SCENE_OT_prisma3d,
    IMPORT_SCENE_OT_prisma3d,
    IMPORT_SCENE_OT_prisma3d_animations,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
