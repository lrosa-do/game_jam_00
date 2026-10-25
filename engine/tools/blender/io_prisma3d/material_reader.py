import re

import numpy as np

from .model import ConvertError

F = np.float32

TEXTURE_KEYS = (
    "albedo",
    "metallic_map",
    "roughness_map",
    "emission",
    "normal",
    "occlusion",
    "clearcoat_map",
    "orm",
    "detail_albedo",
    "detail_normal",
    "detail_mask",
    "refraction_map",
)

CHANNELS = ("Red", "Green", "Blue", "Alpha", "Grayscale")

ENUMERATIONS = {
    "transparency": ("None", "Cutout", "Blend"),
    "alpha_antialiasing": ("Off", "AlphaToCoverage"),
    "blend": ("Alpha", "Add", "Multiply", "Subtract", "Screen"),
    "cull": ("Back", "Front", "None"),
    "metallic_channel": CHANNELS,
    "roughness_channel": CHANNELS,
    "occlusion_channel": CHANNELS,
    "refraction_channel": CHANNELS,
    "emission_operator": ("Add", "Multiply"),
    "detail_blend": ("Alpha", "Add", "Multiply", "Subtract"),
    "detail_uv": ("Uv1", "Uv2"),
    "billboard": ("Disabled", "FixedY"),
}

NUMBER_KEYS = frozenset(
    (
        "alpha_cutoff",
        "alpha_antialiasing_edge",
        "metallic",
        "specular",
        "roughness",
        "refraction",
        "ao_light_affect",
        "emission_energy",
        "normal_scale",
        "clearcoat",
        "clearcoat_roughness",
        "grass_draw_distance",
        "grass_camera_bend",
        "grass_wind_amplitude",
        "grass_wind_speed",
        "leaf_wind_strength",
        "leaf_wind_speed",
        "leaf_wind_height",
        "water_wind_speed",
        "water_wave_scale",
        "water_wave_chop",
        "water_bump",
        "water_bump2",
        "water_specular_mix",
        "water_depth_scale",
        "water_shore_fade",
    )
)

BOOL_KEYS = frozenset(
    (
        "depth_test",
        "lit",
        "detail",
        "billboard_keep_scale",
        "receive_shadows",
        "grass",
        "leaf_wind",
        "water",
        "terrain_morph",
    )
)

VECTOR2_KEYS = frozenset(
    ("water_wind_direction", "water_waves_big", "water_waves_mid", "water_waves_small")
)

VECTOR3_KEYS = frozenset(
    (
        "emission_color",
        "uv1_scale",
        "uv1_offset",
        "uv2_scale",
        "uv2_offset",
        "backlight",
        "grass_tint_a",
        "grass_tint_b",
        "grass_wind_direction",
        "leaf_wind_direction",
        "water_depth_fade",
    )
)

MAX_TEXTURE_SEQUENCE = 64

NUMBER_PATTERN = re.compile(r"[+-]?(?:\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?)")

TOKEN_END = "end"
TOKEN_IDENT = "identifier"
TOKEN_STRING = "string"
TOKEN_NUMBER = "number"
PUNCTUATION = {
    "{": "'{'",
    "}": "'}'",
    "(": "'('",
    ")": "')'",
    ",": "','",
    "=": "'='",
    "[": "'['",
    "]": "']'",
}


class MaterialDef:
    def __init__(self, name):
        self.name = name
        self.values = {}


class Token:
    def __init__(self, kind, text, number, line):
        self.kind = kind
        self.text = text
        self.number = number
        self.line = line


def ident_start(c):
    return ("a" <= c <= "z") or ("A" <= c <= "Z") or c == "_"


def ident_char(c):
    return ident_start(c) or ("0" <= c <= "9")


class Lexer:
    def __init__(self, text):
        self.text = text
        self.pos = 0
        self.line = 1

    def error(self, message):
        raise ConvertError("line %d: %s" % (self.line, message))

    def skip_trivia(self):
        text = self.text
        end = len(text)
        while True:
            while self.pos < end and text[self.pos] in " \t\r":
                self.pos += 1
            if self.pos < end and text[self.pos] == "\n":
                self.pos += 1
                self.line += 1
                continue
            if text.startswith("//", self.pos) or (self.pos < end and text[self.pos] == "#"):
                while self.pos < end and text[self.pos] != "\n":
                    self.pos += 1
                continue
            break

    def next(self):
        self.skip_trivia()
        text = self.text
        end = len(text)
        line = self.line
        if self.pos >= end:
            return Token(TOKEN_END, "", 0.0, line)
        c = text[self.pos]
        if c in PUNCTUATION:
            self.pos += 1
            return Token(c, "", 0.0, line)
        if c == '"':
            start = self.pos + 1
            stop = start
            while stop < end and text[stop] != '"' and text[stop] != "\n":
                stop += 1
            if stop >= end or text[stop] != '"':
                self.error("unterminated string")
            self.pos = stop + 1
            return Token(TOKEN_STRING, text[start:stop], 0.0, line)
        digit = "0" <= c <= "9"
        if digit or (
            c in "-+." and self.pos + 1 < end and (text[self.pos + 1].isdigit() or text[self.pos + 1] == ".")
        ):
            match = NUMBER_PATTERN.match(text, self.pos)
            if match is None:
                self.error("malformed number")
            stop = match.end()
            if stop < end and (ident_char(text[stop]) or text[stop] == "."):
                self.error("malformed number")
            self.pos = stop
            return Token(TOKEN_NUMBER, "", float(match.group()), line)
        if ident_start(c):
            start = self.pos
            while self.pos < end and ident_char(text[self.pos]):
                self.pos += 1
            return Token(TOKEN_IDENT, text[start:self.pos], 0.0, line)
        message = "unexpected character '%s'" % c
        if c == ":":
            message += (
                " (a material does not inherit from another; every key it needs is "
                "written in its own block)"
            )
        self.error(message)


class Parser:
    def __init__(self, text):
        self.lexer = Lexer(text)
        self.current = None
        self.previous = None
        self.advance()

    def advance(self):
        self.previous = self.current
        self.current = self.lexer.next()

    def fail_at(self, line, message):
        raise ConvertError("line %d: %s" % (line, message))

    def fail(self, message):
        self.fail_at(self.current.line, message)

    def describe(self, token):
        kind = token.kind
        if kind == TOKEN_END:
            return "end of file"
        if kind in (TOKEN_IDENT, TOKEN_STRING, TOKEN_NUMBER):
            return kind
        return PUNCTUATION[kind]

    def expect(self, kind, what):
        if self.current.kind != kind:
            message = "expected %s, got %s" % (what, self.describe(self.current))
            if self.current.kind in (TOKEN_IDENT, TOKEN_STRING):
                message += " '%s'" % self.current.text
            self.fail(message)
        self.advance()

    def expect_ident(self, name):
        if self.current.kind != TOKEN_IDENT or self.current.text != name:
            message = "expected '%s', got %s" % (name, self.describe(self.current))
            if self.current.kind == TOKEN_IDENT:
                message += " '%s'" % self.current.text
            self.fail(message)
        self.advance()

    def parse_file(self):
        materials = []
        while self.current.kind != TOKEN_END:
            self.expect_ident("material")
            self.expect(TOKEN_STRING, "material name string")
            if not self.previous.text:
                self.fail("material name must not be empty")
            material = MaterialDef(self.previous.text)
            self.expect("{", "'{'")
            self.parse_body(material)
            self.expect("}", "'}'")
            materials.append(material)
        return materials

    def parse_value(self):
        token = self.current
        if token.kind == TOKEN_IDENT:
            self.advance()
            if token.text == "true" or token.text == "false":
                return ("bool", token.text == "true")
            return ("ident", token.text)
        if token.kind == TOKEN_STRING:
            self.advance()
            return ("string", token.text)
        if token.kind == TOKEN_NUMBER:
            self.advance()
            return ("number", float(F(token.number)))
        if token.kind == "(":
            self.advance()
            items = []
            while True:
                self.expect(TOKEN_NUMBER, "number")
                if len(items) == 4:
                    self.fail("a vector has at most four components")
                items.append(float(F(self.previous.number)))
                if self.current.kind != ",":
                    break
                self.advance()
            if len(items) < 2:
                self.fail("a vector has at least two components")
            self.expect(")", "')'")
            return ("vector", tuple(items))
        if token.kind == "[":
            self.advance()
            items = []
            while self.current.kind != "]":
                self.expect(TOKEN_STRING, "a quoted string inside [ ]")
                items.append(self.previous.text)
                if self.current.kind != ",":
                    break
                self.advance()
            self.expect("]", "']'")
            return ("list", items)
        self.fail("expected a value, got %s" % self.describe(token))

    def parse_body(self, material):
        while self.current.kind != "}":
            self.expect(TOKEN_IDENT, "key")
            key = self.previous.text
            line = self.previous.line
            if self.current.kind == "{":
                self.skip_block(key, line)
                continue
            self.expect("=", "'='")
            value = self.parse_value()
            material.values[key] = self.check(key, value, line)

    def skip_block(self, key, line):
        self.expect("{", "'{'")
        depth = 1
        while depth > 0:
            if self.current.kind == TOKEN_END:
                self.fail_at(line, "block '%s' is never closed" % key)
            if self.current.kind == "{":
                depth += 1
            elif self.current.kind == "}":
                depth -= 1
            self.advance()

    def type_error(self, key, line, expected):
        self.fail_at(line, "'%s' expects %s" % (key, expected))

    def check(self, key, value, line):
        kind, data = value
        for slot_key in TEXTURE_KEYS:
            if key == slot_key:
                if kind == "string":
                    return data
                if kind != "list":
                    self.type_error(key, line, "a quoted path or a list of quoted paths")
                if not data:
                    self.fail_at(line, "'%s': a sequence needs at least one file" % key)
                if len(data) > MAX_TEXTURE_SEQUENCE:
                    self.fail_at(
                        line, "'%s': a sequence has at most %d files" % (key, MAX_TEXTURE_SEQUENCE)
                    )
                return list(data)
            if key == slot_key + "_fps":
                if kind != "number" or not data > 0.0:
                    self.type_error(key, line, "a number greater than 0")
                return data

        if key in ENUMERATIONS:
            if kind != "ident":
                self.type_error(key, line, "a name without quotes")
            if data not in ENUMERATIONS[key]:
                self.fail_at(line, "'%s': unknown value '%s'" % (key, data))
            return data
        if key in NUMBER_KEYS:
            if kind != "number":
                self.type_error(key, line, "a number")
            return data
        if key in BOOL_KEYS:
            if kind != "bool":
                self.type_error(key, line, "true or false")
            return data
        if key == "color":
            if kind != "vector" or len(data) not in (3, 4):
                self.type_error(key, line, "(r, g, b) or (r, g, b, a)")
            return data if len(data) == 4 else data + (1.0,)
        if key in VECTOR2_KEYS:
            if kind != "vector" or len(data) != 2:
                self.type_error(key, line, "(x, y)")
            return data
        if key in VECTOR3_KEYS:
            if kind != "vector" or len(data) != 3:
                self.type_error(key, line, "(x, y, z)")
            return data
        return None


def parse_material_library(text):
    return Parser(text).parse_file()


def find_material(materials, name):
    for material in materials:
        if material.name == name:
            return material
    return None
