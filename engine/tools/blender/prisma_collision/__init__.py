import bmesh
import bpy
from bpy.props import StringProperty
from bpy_extras.io_utils import ExportHelper

FLAG = "prisma_nocollide"
COLOR = "prisma_collision_preview"
SOLID = (0.75, 0.75, 0.75, 1.0)
REMOVED = (0.9, 0.1, 0.08, 1.0)


def edit_meshes(context):
    return [o for o in context.objects_in_mode if o.type == "MESH"]


def face_layer(bm):
    layer = bm.faces.layers.int.get(FLAG)
    if layer is None:
        layer = bm.faces.layers.int.new(FLAG)
    return layer


def color_layer(bm):
    layer = bm.loops.layers.color.get(COLOR)
    if layer is None:
        layer = bm.loops.layers.color.new(COLOR)
    return layer


def paint(bm, flag, colors):
    for face in bm.faces:
        color = REMOVED if face[flag] else SOLID
        for loop in face.loops:
            loop[colors] = color


def show_preview(context, obj):
    attributes = obj.data.color_attributes
    if COLOR in attributes:
        attributes.active_color = attributes[COLOR]
    space = context.space_data
    if space and space.type == "VIEW_3D":
        space.shading.type = "SOLID"
        space.shading.color_type = "VERTEX"


def set_selected(context, value):
    for obj in edit_meshes(context):
        bm = bmesh.from_edit_mesh(obj.data)
        flag = face_layer(bm)
        colors = color_layer(bm)
        for face in bm.faces:
            if face.select:
                face[flag] = value
        paint(bm, flag, colors)
        bmesh.update_edit_mesh(obj.data)
        show_preview(context, obj)


class MESH_OT_prisma_no_collide(bpy.types.Operator):
    bl_idname = "mesh.prisma_no_collide"
    bl_label = "No collision"
    bl_description = "Mark the selected faces as not colliding"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        set_selected(context, 1)
        return {"FINISHED"}


class MESH_OT_prisma_collide(bpy.types.Operator):
    bl_idname = "mesh.prisma_collide"
    bl_label = "Collide"
    bl_description = "Mark the selected faces as colliding again"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        set_selected(context, 0)
        return {"FINISHED"}


class MESH_OT_prisma_select_no_collide(bpy.types.Operator):
    bl_idname = "mesh.prisma_select_no_collide"
    bl_label = "Select no-collision"
    bl_description = "Select the faces marked as not colliding"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        for obj in edit_meshes(context):
            bm = bmesh.from_edit_mesh(obj.data)
            flag = face_layer(bm)
            for face in bm.faces:
                face.select_set(bool(face[flag]))
            bm.select_flush_mode()
            bmesh.update_edit_mesh(obj.data)
        return {"FINISHED"}


class MESH_OT_prisma_clear_collision(bpy.types.Operator):
    bl_idname = "mesh.prisma_clear_collision"
    bl_label = "Clear"
    bl_description = "Make every face collide again"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        for obj in edit_meshes(context):
            bm = bmesh.from_edit_mesh(obj.data)
            flag = face_layer(bm)
            colors = color_layer(bm)
            for face in bm.faces:
                face[flag] = 0
            paint(bm, flag, colors)
            bmesh.update_edit_mesh(obj.data)
        return {"FINISHED"}


class EXPORT_SCENE_OT_prisma_collision(bpy.types.Operator, ExportHelper):
    bl_idname = "export_scene.prisma_collision"
    bl_label = "Export collision"
    bl_description = "Export the selected meshes without the faces marked as not colliding"

    filename_ext = ".glb"
    filter_glob: StringProperty(default="*.glb", options={"HIDDEN"}, maxlen=255)

    @classmethod
    def poll(cls, context):
        return any(o.type == "MESH" for o in context.selected_objects)

    def invoke(self, context, event):
        if not self.filepath:
            base = bpy.path.basename(bpy.data.filepath) or "level.blend"
            self.filepath = bpy.path.ensure_ext(base[: -len(".blend")] + "_collision", ".glb")
        return super().invoke(context, event)

    def execute(self, context):
        if context.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        sources = [o for o in context.selected_objects if o.type == "MESH"]
        depsgraph = context.evaluated_depsgraph_get()
        collection = bpy.data.collections.new("prisma_collision_export")
        context.scene.collection.children.link(collection)
        copies = []
        removed = 0
        kept = 0
        try:
            for obj in sources:
                evaluated = obj.evaluated_get(depsgraph)
                mesh = bpy.data.meshes.new_from_object(evaluated)
                bm = bmesh.new()
                bm.from_mesh(mesh)
                flag = bm.faces.layers.int.get(FLAG)
                if flag is not None:
                    doomed = [f for f in bm.faces if f[flag]]
                    removed += len(doomed)
                    bmesh.ops.delete(bm, geom=doomed, context="FACES")
                kept += len(bm.faces)
                bm.to_mesh(mesh)
                bm.free()
                for name in [a.name for a in mesh.attributes if a.name in (FLAG, COLOR)]:
                    mesh.attributes.remove(mesh.attributes[name])
                mesh.materials.clear()
                copy = bpy.data.objects.new(obj.name, mesh)
                copy.matrix_world = obj.matrix_world
                collection.objects.link(copy)
                copies.append(copy)
            for o in context.selected_objects:
                o.select_set(False)
            for copy in copies:
                copy.select_set(True)
            bpy.ops.export_scene.gltf(filepath=self.filepath, export_format="GLB", use_selection=True,
                                      export_materials="NONE", export_apply=False)
        finally:
            for copy in copies:
                mesh = copy.data
                bpy.data.objects.remove(copy, do_unlink=True)
                bpy.data.meshes.remove(mesh)
            bpy.data.collections.remove(collection)
            for obj in sources:
                obj.select_set(True)
        self.report({"INFO"}, "Collision exported: %d faces kept, %d removed" % (kept, removed))
        return {"FINISHED"}


class VIEW3D_PT_prisma_collision(bpy.types.Panel):
    bl_label = "Collision"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Prisma"

    def draw(self, context):
        layout = self.layout
        column = layout.column(align=True)
        column.label(text="Selected faces")
        row = column.row(align=True)
        row.operator(MESH_OT_prisma_no_collide.bl_idname, icon="X")
        row.operator(MESH_OT_prisma_collide.bl_idname, icon="CHECKMARK")
        column.operator(MESH_OT_prisma_select_no_collide.bl_idname, icon="RESTRICT_SELECT_OFF")
        column.operator(MESH_OT_prisma_clear_collision.bl_idname, icon="TRASH")
        layout.separator()
        layout.operator(EXPORT_SCENE_OT_prisma_collision.bl_idname, icon="EXPORT")


def menu_export(self, context):
    self.layout.operator(EXPORT_SCENE_OT_prisma_collision.bl_idname, text="prisma3d collision (.glb)")


CLASSES = (
    MESH_OT_prisma_no_collide,
    MESH_OT_prisma_collide,
    MESH_OT_prisma_select_no_collide,
    MESH_OT_prisma_clear_collision,
    EXPORT_SCENE_OT_prisma_collision,
    VIEW3D_PT_prisma_collision,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_export.append(menu_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_export)
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
