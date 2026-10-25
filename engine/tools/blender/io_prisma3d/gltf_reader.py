import json
import os

import numpy as np

from .model import ConvertError

COMPONENT_INT8 = 5120
COMPONENT_UINT8 = 5121
COMPONENT_INT16 = 5122
COMPONENT_UINT16 = 5123
COMPONENT_UINT32 = 5125
COMPONENT_FLOAT = 5126

COMPONENT_DTYPE = {
    COMPONENT_INT8: np.dtype("<i1"),
    COMPONENT_UINT8: np.dtype("<u1"),
    COMPONENT_INT16: np.dtype("<i2"),
    COMPONENT_UINT16: np.dtype("<u2"),
    COMPONENT_UINT32: np.dtype("<u4"),
    COMPONENT_FLOAT: np.dtype("<f4"),
}

TYPE_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}

MAX_JSON_SIZE = 512 * 1024 * 1024


def decode_uri(uri):
    raw = uri.encode("utf-8")
    out = bytearray()
    length = len(raw)
    hex_digits = b"0123456789abcdefABCDEF"
    i = 0
    while i < length:
        if raw[i] == 0x25 and i + 2 < length:
            high = raw[i + 1]
            low = raw[i + 2]
            if high in hex_digits and low in hex_digits:
                out.append(int(bytes((high, low)), 16))
                i += 3
                continue
        out.append(raw[i])
        i += 1
    return bytes(out).decode("utf-8", errors="surrogateescape")


def is_data_uri(uri):
    return uri.startswith("data:")


class AccessorRef:
    def __init__(self):
        self.buffer = -1
        self.offset = 0
        self.stride = 0
        self.count = 0
        self.components = 0
        self.component_type = 0
        self.normalized = False

    def valid(self):
        return self.count > 0 and self.components > 0

    def component_size(self):
        return COMPONENT_DTYPE[self.component_type].itemsize

    def copy(self):
        other = AccessorRef()
        other.buffer = self.buffer
        other.offset = self.offset
        other.stride = self.stride
        other.count = self.count
        other.components = self.components
        other.component_type = self.component_type
        other.normalized = self.normalized
        return other

    def byte_length(self):
        if self.count == 0:
            return 0
        return (self.count - 1) * self.stride + self.components * self.component_size()


class GltfDocument:
    def __init__(self, path):
        self.path = path
        self.directory = os.path.dirname(os.path.abspath(path))
        try:
            with open(path, "rb") as handle:
                text = handle.read()
        except OSError:
            raise ConvertError("cannot open the exported glTF file")
        if len(text) == 0:
            raise ConvertError("empty glTF data")
        if text[:4] == b"glTF":
            raise ConvertError("binary glTF is not handled here, export with separate files")
        if len(text) > MAX_JSON_SIZE:
            raise ConvertError("glTF JSON is too large")
        try:
            self.json = json.loads(text.decode("utf-8"))
        except ValueError:
            raise ConvertError("invalid glTF JSON")
        if not isinstance(self.json, dict):
            raise ConvertError("invalid glTF")
        self.buffer_paths = []
        self.buffer_sizes = []
        self.buffer_data = []
        for buffer in self.json.get("buffers", []):
            uri = buffer.get("uri")
            if uri is None:
                raise ConvertError("buffer without uri")
            if is_data_uri(uri):
                raise ConvertError("data URIs are not supported")
            self.buffer_paths.append(os.path.join(self.directory, decode_uri(uri)))
            self.buffer_sizes.append(int(buffer.get("byteLength", 0)))
            self.buffer_data.append(None)

    def array(self, name):
        return self.json.get(name, [])

    def buffer_bytes(self, index):
        if index < 0 or index >= len(self.buffer_data):
            raise ConvertError("accessor refers to a buffer that does not exist")
        data = self.buffer_data[index]
        if data is None:
            try:
                with open(self.buffer_paths[index], "rb") as handle:
                    data = handle.read()
            except OSError:
                raise ConvertError("cannot open buffer file")
            self.buffer_data[index] = data
        return data

    def accessor_ref(self, index):
        accessors = self.array("accessors")
        if index is None or index < 0 or index >= len(accessors):
            raise ConvertError("accessor does not exist")
        accessor = accessors[index]
        if "sparse" in accessor:
            raise ConvertError("sparse accessors are not supported")
        view_index = accessor.get("bufferView")
        views = self.array("bufferViews")
        if view_index is None or view_index < 0 or view_index >= len(views):
            raise ConvertError("accessor without buffer view")
        view = views[view_index]
        extensions = view.get("extensions", {})
        if "EXT_meshopt_compression" in extensions or "KHR_meshopt_compression" in extensions:
            raise ConvertError("compressed buffer views are not supported")
        if "buffer" not in view:
            raise ConvertError("accessor without buffer view")
        components = TYPE_COMPONENTS.get(accessor.get("type"), 0)
        if components == 0:
            raise ConvertError("unsupported accessor type")
        component_type = accessor.get("componentType")
        if component_type not in COMPONENT_DTYPE:
            raise ConvertError("unsupported component type")
        count = int(accessor.get("count", 0))
        if count > 0xFFFFFFFF:
            raise ConvertError("accessor count is too large")
        element_size = components * COMPONENT_DTYPE[component_type].itemsize

        ref = AccessorRef()
        ref.buffer = int(view["buffer"])
        ref.offset = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
        stride = view.get("byteStride", 0)
        ref.stride = int(stride) if stride else element_size
        ref.count = count
        ref.components = components
        ref.component_type = component_type
        ref.normalized = bool(accessor.get("normalized", False))

        if ref.stride < element_size:
            raise ConvertError("accessor stride is smaller than its element")
        if ref.count > 0:
            length = ref.byte_length()
            if ref.buffer < 0 or ref.buffer >= len(self.buffer_sizes):
                raise ConvertError("accessor exceeds its buffer")
            buffer_size = self.buffer_sizes[ref.buffer]
            if length > buffer_size or ref.offset > buffer_size - length:
                raise ConvertError("accessor exceeds its buffer")
        return ref

    def raw_view(self, ref):
        if ref.count == 0:
            return np.zeros((0, ref.components), dtype=COMPONENT_DTYPE[ref.component_type])
        data = self.buffer_bytes(ref.buffer)
        length = ref.byte_length()
        if ref.offset + length > len(data):
            raise ConvertError("buffer file is smaller than declared")
        dtype = COMPONENT_DTYPE[ref.component_type]
        view = np.ndarray(
            shape=(ref.count, ref.components),
            dtype=dtype,
            buffer=data,
            offset=ref.offset,
            strides=(ref.stride, dtype.itemsize),
        )
        return view


def unpack_floats(document, ref, out_components):
    view = document.raw_view(ref)
    copied = min(ref.components, out_components)
    out = np.zeros((ref.count, out_components), dtype=np.float32)
    if ref.count == 0:
        return out
    source = view[:, :copied]
    t = ref.component_type
    if t == COMPONENT_FLOAT:
        converted = source.astype(np.float32)
    elif t == COMPONENT_INT8:
        converted = source.astype(np.float32)
        if ref.normalized:
            converted = np.maximum(converted / np.float32(127.0), np.float32(-1.0))
    elif t == COMPONENT_UINT8:
        converted = source.astype(np.float32)
        if ref.normalized:
            converted = converted / np.float32(255.0)
    elif t == COMPONENT_INT16:
        converted = source.astype(np.float32)
        if ref.normalized:
            converted = np.maximum(converted / np.float32(32767.0), np.float32(-1.0))
    elif t == COMPONENT_UINT16:
        converted = source.astype(np.float32)
        if ref.normalized:
            converted = converted / np.float32(65535.0)
    else:
        converted = source.astype(np.float32)
    out[:, :copied] = converted
    return out


def unpack_indices(document, ref):
    if ref.component_type not in (COMPONENT_UINT8, COMPONENT_UINT16, COMPONENT_UINT32):
        raise ConvertError("unsupported index accessor")
    if ref.components != 1:
        raise ConvertError("unsupported index accessor")
    view = document.raw_view(ref)
    return view[:, 0].astype(np.uint32)
