import os
import re
import shutil
import tempfile

import bpy

from . import material_reader, pmesh_reader
from .gltf_builder import GltfBuilder, write_gltf
from .model import ConvertError
from .pmesh_writer import file_stem

ESSENTIAL_ARGUMENTS = (
    "filepath",
    "import_shading",
    "bone_heuristic",
    "guess_original_bind_pose",
    "merge_vertices",
    "import_pack_images",
)

AUTO_SUFFIX = re.compile(r"\.\d{3}$")


class ImportResult:
    def __init__(self):
        self.warnings = []
        self.count = 0


def gltf_arguments(filepath):
    return {
        "filepath": filepath,
        "import_pack_images": False,
        "merge_vertices": False,
        "import_shading": "NORMALS",
        "bone_heuristic": "BLENDER",
        "disable_bone_shape": True,
        "guess_original_bind_pose": True,
        "import_webp_texture": False,
        "import_unused_materials": False,
        "import_select_created_objects": True,
        "import_scene_extras": False,
        "import_scene_as_collection": True,
        "import_merge_material_slots": False,
        "import_point_as_pointcloud": False,
        "export_import_convert_lighting_mode": "SPEC",
    }


def supported_arguments(arguments):
    known = set(bpy.ops.import_scene.gltf.get_rna_type().properties.keys())
    missing = [name for name in ESSENTIAL_ARGUMENTS if name not in known]
    if missing:
        raise ConvertError("this Blender version has no glTF import option(s): " + ", ".join(missing))
    return {name: value for name, value in arguments.items() if name in known}


def run_gltf_import(gltf_path):
    if not hasattr(bpy.ops.import_scene, "gltf"):
        raise ConvertError("the glTF 2.0 add-on is not enabled")
    arguments = supported_arguments(gltf_arguments(gltf_path))
    try:
        result = bpy.ops.import_scene.gltf(**arguments)
    except RuntimeError as error:
        raise ConvertError("the glTF import failed: %s" % str(error).strip())
    if "FINISHED" not in result:
        raise ConvertError("the glTF import did not finish")


def frame_rate():
    render = bpy.context.scene.render
    # the bundled importer converts seconds to frames with this exact product
    return render.fps * render.fps_base


def restore_action_names(names_before, clip_names):
    messages = []
    wanted = set(clip_names)
    for action in [a for a in bpy.data.actions if a.name not in names_before]:
        base = AUTO_SUFFIX.sub("", action.name)
        if action.name in wanted or base not in wanted:
            continue
        if base in bpy.data.actions:
            messages.append("action '%s' already exists, the clip was imported as '%s'" % (base, action.name))
        else:
            action.name = base
    return messages


def load_library(model, directory, stem, result):
    if not model.material_slots:
        return None
    path = os.path.join(directory, stem + ".mat")
    if not os.path.isfile(path):
        result.warnings.append("'%s.mat' was not found, the materials keep their default values" % stem)
        return None
    try:
        with open(path, "rb") as handle:
            text = handle.read().decode("utf-8", errors="replace")
        return material_reader.parse_material_library(text)
    except (OSError, ConvertError) as error:
        result.warnings.append("%s.mat: %s; the materials keep their default values" % (stem, error))
        return None


def import_pmesh(filepath):
    result = ImportResult()
    directory = os.path.dirname(os.path.abspath(filepath))
    stem = file_stem(os.path.basename(filepath))
    if not stem:
        raise ConvertError("the file name is empty")

    try:
        model = pmesh_reader.load_pmesh(filepath)
    except ConvertError as error:
        raise ConvertError("%s: %s" % (os.path.basename(filepath), error))
    if not model.nodes and not model.meshes:
        raise ConvertError("there is nothing to import")

    library = load_library(model, directory, stem, result)
    builder = GltfBuilder(model, library, directory, stem, frame_rate())
    document, binary = builder.build()
    result.warnings = builder.warnings + result.warnings
    if not document.get("nodes"):
        raise ConvertError("there is nothing to import")

    temporary = tempfile.mkdtemp(prefix="prisma3d_")
    try:
        gltf_path = write_gltf(temporary, stem, document, binary)
        names_before = {action.name for action in bpy.data.actions}
        run_gltf_import(gltf_path)
        result.warnings.extend(
            restore_action_names(names_before, [clip.name for clip in model.animations])
        )
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return result
