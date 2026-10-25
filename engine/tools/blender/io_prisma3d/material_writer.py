import numpy as np

from .model import ALPHA_BLEND, ALPHA_MASK
from .pmesh_writer import clean_name

F = np.float32

TRANSPARENCY_OPAQUE = 0
TRANSPARENCY_ALPHA_SCISSOR = 1
TRANSPARENCY_ALPHA_BLEND = 2
TRANSPARENCY_NAMES = {
    TRANSPARENCY_OPAQUE: "None",
    TRANSPARENCY_ALPHA_SCISSOR: "Cutout",
    TRANSPARENCY_ALPHA_BLEND: "Blend",
}

CULL_BACK = 0
CULL_DISABLED = 2
CULL_NAMES = {0: "Back", 1: "Front", 2: "None"}

EMISSION_ADD = 0
EMISSION_MULTIPLY = 1
EMISSION_OPERATOR_NAMES = {EMISSION_ADD: "Add", EMISSION_MULTIPLY: "Multiply"}

CHANNEL_RED = 0
CHANNEL_GREEN = 1
CHANNEL_BLUE = 2
CHANNEL_NAMES = {0: "Red", 1: "Green", 2: "Blue", 3: "Alpha", 4: "Grayscale"}

TEXTURE_ALBEDO = "albedo"
TEXTURE_METALLIC = "metallic_map"
TEXTURE_ROUGHNESS = "roughness_map"
TEXTURE_EMISSION = "emission"
TEXTURE_NORMAL = "normal"
TEXTURE_OCCLUSION = "occlusion"

KEY_WIDTH = 16


def clamp(value, low, high):
    value = F(value)
    if not (value >= low):
        return F(low)
    if value > high:
        return F(high)
    return value


class MaterialInfo:
    def __init__(self):
        self.name = ""
        self.albedo = (F(1.0), F(1.0), F(1.0), F(1.0))
        self.metallic = F(0.0)
        self.roughness = F(1.0)
        self.emission = (F(0.0), F(0.0), F(0.0))
        self.alpha_scissor_threshold = F(0.5)
        self.metallic_channel = CHANNEL_RED
        self.roughness_channel = CHANNEL_RED
        self.ao_channel = CHANNEL_RED
        self.transparency = TRANSPARENCY_OPAQUE
        self.cull = CULL_BACK
        self.emission_operator = EMISSION_ADD
        self.paths = {}


DEFAULTS = MaterialInfo()


def create_material(material, texture_files):
    info = MaterialInfo()
    info.name = material.name
    info.albedo = tuple(clamp(v, 0.0, 1.0) for v in material.base_color)
    info.metallic = clamp(material.metallic, 0.0, 1.0)
    info.roughness = clamp(material.roughness, 0.0, 1.0)
    info.emission = tuple(clamp(v, 0.0, 1.0) for v in material.emissive)
    if material.alpha_mode == ALPHA_MASK:
        info.transparency = TRANSPARENCY_ALPHA_SCISSOR
        info.alpha_scissor_threshold = clamp(material.alpha_cutoff, 0.0, 1.0)
    elif material.alpha_mode == ALPHA_BLEND:
        info.transparency = TRANSPARENCY_ALPHA_BLEND
    if material.double_sided:
        info.cull = CULL_DISABLED

    def file_at(index):
        if index < 0 or index >= len(texture_files):
            return None
        return texture_files[index]

    albedo = file_at(material.base_color_texture)
    metallic_roughness = file_at(material.metallic_roughness_texture)
    normal = file_at(material.normal_texture)
    occlusion = file_at(material.occlusion_texture)
    emissive = file_at(material.emissive_texture)
    if albedo:
        info.paths[TEXTURE_ALBEDO] = albedo
    if metallic_roughness:
        info.paths[TEXTURE_METALLIC] = metallic_roughness
        info.paths[TEXTURE_ROUGHNESS] = metallic_roughness
    info.metallic_channel = CHANNEL_BLUE
    info.roughness_channel = CHANNEL_GREEN
    if normal:
        info.paths[TEXTURE_NORMAL] = normal
    if occlusion:
        info.paths[TEXTURE_OCCLUSION] = occlusion
    info.ao_channel = CHANNEL_RED
    if emissive:
        info.paths[TEXTURE_EMISSION] = emissive
        info.emission_operator = EMISSION_MULTIPLY
    return info


def append_number(value):
    return "%g" % float(value)


def append_vector(values):
    return "(" + ", ".join(append_number(v) for v in values) + ")"


def write_key(key):
    return "    " + key + " " * max(0, KEY_WIDTH - len(key)) + " = "


def write_number(key, value):
    return write_key(key) + append_number(value) + "\n"


def write_vector(key, values):
    return write_key(key) + append_vector(values) + "\n"


def write_name(key, value):
    return write_key(key) + value + "\n"


def write_string(key, value):
    return write_key(key) + '"' + value + '"\n'


def same_vector(a, b):
    return all(x == y for x, y in zip(a, b))


def write_texture(info, key):
    path = info.paths.get(key)
    if path:
        return write_string(key, path)
    return ""


def write_material(info):
    defaults = DEFAULTS
    out = 'material "' + info.name + '" {\n'

    if info.transparency != defaults.transparency:
        out += write_name("transparency", TRANSPARENCY_NAMES[info.transparency])
    if info.alpha_scissor_threshold != defaults.alpha_scissor_threshold:
        out += write_number("alpha_cutoff", info.alpha_scissor_threshold)
    if info.cull != defaults.cull:
        out += write_name("cull", CULL_NAMES[info.cull])

    if not same_vector(info.albedo, defaults.albedo):
        out += write_vector("color", info.albedo)
    out += write_texture(info, TEXTURE_ALBEDO)
    if info.metallic != defaults.metallic:
        out += write_number("metallic", info.metallic)
    out += write_texture(info, TEXTURE_METALLIC)
    if info.metallic_channel != defaults.metallic_channel:
        out += write_name("metallic_channel", CHANNEL_NAMES[info.metallic_channel])
    if info.roughness != defaults.roughness:
        out += write_number("roughness", info.roughness)
    out += write_texture(info, TEXTURE_ROUGHNESS)
    if info.roughness_channel != defaults.roughness_channel:
        out += write_name("roughness_channel", CHANNEL_NAMES[info.roughness_channel])
    out += write_texture(info, TEXTURE_OCCLUSION)
    if info.ao_channel != defaults.ao_channel:
        out += write_name("occlusion_channel", CHANNEL_NAMES[info.ao_channel])
    if not same_vector(info.emission, defaults.emission):
        out += write_vector("emission_color", info.emission)
    if info.emission_operator != defaults.emission_operator:
        out += write_name("emission_operator", EMISSION_OPERATOR_NAMES[info.emission_operator])
    out += write_texture(info, TEXTURE_EMISSION)
    out += write_texture(info, TEXTURE_NORMAL)

    out += "}\n"
    return out


def build_library(model, texture_files):
    infos = []
    names = set()
    warnings = []
    for i, material in enumerate(model.materials):
        info = create_material(material, texture_files)
        info.name = clean_name(info.name)
        if not info.name:
            info.name = "material_%d" % i
        if info.name in names:
            warnings.append("duplicate material name '%s' skipped" % info.name)
            continue
        names.add(info.name)
        infos.append(info)
    text = ""
    for info in infos:
        if text:
            text += "\n"
        text += write_material(info)
    return text, warnings


def texture_files_for(model, image_files):
    files = []
    for texture in model.textures:
        if 0 <= texture.image < len(image_files):
            files.append(image_files[texture.image])
        else:
            files.append(None)
    return files
