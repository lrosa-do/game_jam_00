import filecmp
import os
import shutil
import tempfile

import bpy

from . import material_writer, pmesh_writer
from .gltf_convert import GltfConverter
from .model import ConvertError

ESSENTIAL_ARGUMENTS = (
    "filepath",
    "export_format",
    "export_yup",
    "export_tangents",
    "export_normals",
    "export_texcoords",
    "export_skins",
    "export_animations",
    "export_animation_mode",
    "export_morph",
    "export_lights",
    "export_cameras",
    "export_materials",
    "use_selection",
    "export_apply",
)


class Options:
    def __init__(self, selection_only=False, apply_modifiers=True, animations=True, cameras=True):
        self.selection_only = selection_only
        self.apply_modifiers = apply_modifiers
        self.animations = animations
        self.cameras = cameras


class ExportResult:
    def __init__(self):
        self.files = []
        self.warnings = []


def gltf_arguments(filepath, options):
    return {
        "filepath": filepath,
        "export_format": "GLTF_SEPARATE",
        "export_yup": True,
        "export_tangents": True,
        "export_normals": True,
        "export_texcoords": True,
        "export_vertex_color": "ACTIVE",
        "export_skins": True,
        "export_animations": options.animations,
        "export_animation_mode": "ACTIONS",
        "export_morph": False,
        "export_lights": False,
        "export_cameras": options.cameras,
        "export_materials": "EXPORT",
        "export_image_format": "AUTO",
        "use_selection": options.selection_only,
        "export_apply": options.apply_modifiers,
        "export_texture_dir": "",
        "export_keep_originals": False,
        "export_image_add_webp": False,
        "export_use_gltfpack": False,
        "export_draco_mesh_compression_enable": False,
        "export_meshopt_compression_enable": False,
        "export_extras": False,
        "export_gn_mesh": False,
        "export_attributes": False,
        "export_shared_accessors": False,
        "export_unused_images": False,
        "export_unused_textures": False,
        "export_gpu_instances": False,
        "export_all_influences": False,
        "export_influence_nb": 4,
        "export_def_bones": False,
        "export_leaf_bone": False,
        "export_current_frame": False,
        "export_frame_range": False,
        "export_force_sampling": True,
        "export_action_filter": False,
        "use_mesh_edges": False,
        "use_mesh_vertices": False,
        "use_visible": False,
        "use_renderable": False,
        "use_active_collection": False,
        "use_active_scene": False,
        "will_save_settings": False,
    }


def supported_arguments(arguments):
    known = set(bpy.ops.export_scene.gltf.get_rna_type().properties.keys())
    missing = [name for name in ESSENTIAL_ARGUMENTS if name not in known]
    if missing:
        raise ConvertError(
            "this Blender version has no glTF export option(s): " + ", ".join(missing)
        )
    return {name: value for name, value in arguments.items() if name in known}


def run_gltf_export(directory, options):
    if not hasattr(bpy.ops.export_scene, "gltf"):
        raise ConvertError("the glTF 2.0 add-on is not enabled")
    path = os.path.join(directory, "export.gltf")
    arguments = supported_arguments(gltf_arguments(path, options))
    try:
        result = bpy.ops.export_scene.gltf(**arguments)
    except RuntimeError as error:
        raise ConvertError("the glTF export failed: %s" % str(error).strip())
    if "FINISHED" not in result or not os.path.isfile(path):
        raise ConvertError("the glTF export did not produce a file")
    return path


def same_content(first, second):
    try:
        return filecmp.cmp(first, second, shallow=False)
    except OSError:
        return False


def plan_images(model, directory):
    claimed = {}
    names = []
    copies = []
    for image in model.images:
        source = image.path
        if not os.path.isfile(source):
            raise ConvertError("the exported image '%s' is missing" % os.path.basename(source))
        base = os.path.basename(source)
        stem, extension = os.path.splitext(base)
        counter = 0
        while True:
            candidate = base if counter == 0 else "%s_%d%s" % (stem, counter, extension)
            destination = os.path.join(directory, candidate)
            if destination in claimed:
                if same_content(claimed[destination], source):
                    break
            elif os.path.exists(destination):
                if same_content(destination, source):
                    break
            else:
                claimed[destination] = source
                copies.append((destination, source))
                break
            counter += 1
        names.append(candidate)
    return names, copies


def export_files(filepath, options):
    result = ExportResult()
    directory = os.path.dirname(os.path.abspath(filepath))
    os.makedirs(directory, exist_ok=True)
    stem = pmesh_writer.file_stem(os.path.basename(filepath))
    if not stem:
        raise ConvertError("the file name is empty")

    temporary = tempfile.mkdtemp(prefix="prisma3d_")
    try:
        gltf_path = run_gltf_export(temporary, options)
        converter = GltfConverter(gltf_path)
        model = converter.convert()
        if not model.nodes:
            raise ConvertError("there is nothing to export")

        files = pmesh_writer.build_pmesh(model, converter, stem)
        image_names, image_copies = plan_images(model, directory)
        texture_files = material_writer.texture_files_for(model, image_names)
        material_text, material_warnings = material_writer.build_library(model, texture_files)

        items = []
        mesh_path = os.path.join(directory, stem + "." + pmesh_writer.MESH_EXTENSION)
        items.append((mesh_path, files.mesh))
        for name, data in zip(files.skeleton_names, files.skeletons):
            items.append((os.path.join(directory, name), data))
        for name, data in zip(files.animation_names, files.animations):
            items.append((os.path.join(directory, name), data))
        if model.materials:
            items.append((os.path.join(directory, stem + ".mat"), material_text.encode("utf-8")))
        for destination, source in image_copies:
            with open(source, "rb") as handle:
                items.append((destination, handle.read()))

        pmesh_writer.write_staged(items)
        result.files = [path for path, _ in items]
        result.warnings = list(model.warnings) + material_warnings
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return result
