import numpy as np

F = np.float32

ZERO = F(0.0)
ONE = F(1.0)
TWO = F(2.0)
HALF = F(0.5)
QUARTER = F(0.25)
EPSILON = F(1e-6)
EPSILON_SQUARED = F(1e-6) * F(1e-6)
SPAN_EPSILON = F(1e-6)
NODE_EPSILON = F(1e-8)
QUAT_EPSILON = F(1e-12)
AFFINE_TOLERANCE = F(1e-4)
UNIT_X = (ONE, ZERO, ZERO)
UNIT_Y = (ZERO, ONE, ZERO)
UNIT_Z = (ZERO, ZERO, ONE)
IDENTITY_QUAT = (ZERO, ZERO, ZERO, ONE)


def is_finite(value):
    return bool(np.isfinite(value))


def sqrt(value):
    return np.sqrt(F(value))


def v3_length_squared(v):
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def v3_length(v):
    return sqrt(v3_length_squared(v))


def v3_div(v, s):
    inv = ONE / s
    return (v[0] * inv, v[1] * inv, v[2] * inv)


def v3_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v3_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v3_scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def quat_length_squared(q):
    return q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]


def quat_normalized(q):
    inv = ONE / sqrt(quat_length_squared(q))
    return (q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv)


def quat_to_mat3(q):
    x, y, z, w = q
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (ONE - TWO * (yy + zz), TWO * (xy + wz), TWO * (xz - wy)),
        (TWO * (xy - wz), ONE - TWO * (xx + zz), TWO * (yz + wx)),
        (TWO * (xz + wy), TWO * (yz - wx), ONE - TWO * (xx + yy)),
    )


def quat_from_mat3(c0, c1, c2):
    m00, m01, m02 = c0
    m10, m11, m12 = c1
    m20, m21, m22 = c2

    four_x = m00 - m11 - m22
    four_y = m11 - m00 - m22
    four_z = m22 - m00 - m11
    four_w = m00 + m11 + m22

    biggest_index = 0
    four_biggest = four_w
    if four_x > four_biggest:
        four_biggest = four_x
        biggest_index = 1
    if four_y > four_biggest:
        four_biggest = four_y
        biggest_index = 2
    if four_z > four_biggest:
        four_biggest = four_z
        biggest_index = 3

    biggest = sqrt(four_biggest + ONE) * HALF
    mult = QUARTER / biggest

    if biggest_index == 1:
        return (biggest, (m01 + m10) * mult, (m20 + m02) * mult, (m12 - m21) * mult)
    if biggest_index == 2:
        return ((m01 + m10) * mult, biggest, (m12 + m21) * mult, (m20 - m02) * mult)
    if biggest_index == 3:
        return ((m20 + m02) * mult, (m12 + m21) * mult, biggest, (m01 - m10) * mult)
    return ((m12 - m21) * mult, (m20 - m02) * mult, (m01 - m10) * mult, biggest)


def compose_local(translation, rotation, scale):
    basis = quat_to_mat3(rotation)
    c0 = v3_scale(basis[0], scale[0])
    c1 = v3_scale(basis[1], scale[1])
    c2 = v3_scale(basis[2], scale[2])
    out = np.zeros(16, dtype=np.float32)
    out[0:3] = c0
    out[3] = ZERO
    out[4:7] = c1
    out[7] = ZERO
    out[8:11] = c2
    out[11] = ZERO
    out[12:15] = translation
    out[15] = ONE
    return out


def pose_from_matrix(m):
    for i in range(16):
        if not is_finite(m[i]):
            return None
    if (
        abs(m[3]) > AFFINE_TOLERANCE
        or abs(m[7]) > AFFINE_TOLERANCE
        or abs(m[11]) > AFFINE_TOLERANCE
        or abs(m[15] - ONE) > AFFINE_TOLERANCE
    ):
        return None

    x = (m[0], m[1], m[2])
    y = (m[4], m[5], m[6])
    z = (m[8], m[9], m[10])

    sx = v3_length(x)
    sy = v3_length(y)
    sz = v3_length(z)
    if v3_dot(x, v3_cross(y, z)) < ZERO:
        sx = -sx

    c0 = v3_div(x, sx) if sx * sx > NODE_EPSILON else UNIT_X
    c1 = v3_div(y, sy) if sy * sy > NODE_EPSILON else UNIT_Y
    c2 = v3_div(z, sz) if sz * sz > NODE_EPSILON else UNIT_Z

    rotation = quat_normalized(quat_from_mat3(c0, c1, c2))
    return ((m[12], m[13], m[14]), rotation, (sx, sy, sz))


def decompose_matrix(m):
    x = (m[0], m[1], m[2])
    y = (m[4], m[5], m[6])
    z = (m[8], m[9], m[10])
    sx = v3_length(x)
    sy = v3_length(y)
    sz = v3_length(z)

    c0 = v3_div(x, sx) if sx * sx > NODE_EPSILON else UNIT_X
    c1 = v3_div(y, sy) if sy * sy > NODE_EPSILON else UNIT_Y
    c2 = v3_div(z, sz) if sz * sz > NODE_EPSILON else UNIT_Z

    rotation = quat_normalized(quat_from_mat3(c0, c1, c2))
    return ((m[12], m[13], m[14]), rotation, (sx, sy, sz))


def sample_vec3(times, values, time):
    n = len(times)
    out = np.empty((len(time), 3), dtype=np.float32)
    if n == 0:
        return None
    if n == 1:
        out[:] = values[0]
        return out
    time = time.astype(np.float32)
    nxt = np.searchsorted(times, time, side="right")
    first = time <= times[0]
    last = time >= times[n - 1]
    nxt = np.clip(nxt, 1, n - 1)
    prev = nxt - 1
    span = times[nxt] - times[prev]
    safe_span = np.where(span > SPAN_EPSILON, span, ONE).astype(np.float32)
    amount = np.where(span > SPAN_EPSILON, (time - times[prev]) / safe_span, ZERO).astype(np.float32)
    a = values[prev]
    b = values[nxt]
    mixed = (a + (b - a) * amount[:, None]).astype(np.float32)
    out[:] = mixed
    out[first] = values[0]
    out[last & ~first] = values[n - 1]
    return out


def sample_quaternion(times, values, time):
    n = len(times)
    out = np.empty((len(time), 4), dtype=np.float32)
    if n == 0:
        return None
    if n == 1:
        out[:] = values[0]
        return out
    time = time.astype(np.float32)
    nxt = np.searchsorted(times, time, side="right")
    first = time <= times[0]
    last = time >= times[n - 1]
    nxt = np.clip(nxt, 1, n - 1)
    prev = nxt - 1
    span = times[nxt] - times[prev]
    safe_span = np.where(span > SPAN_EPSILON, span, ONE).astype(np.float32)
    amount = np.where(span > SPAN_EPSILON, (time - times[prev]) / safe_span, ZERO).astype(np.float32)
    a = values[prev]
    b = values[nxt]
    dot = ((a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]) + a[:, 2] * b[:, 2]) + a[:, 3] * b[:, 3]
    bb = np.where((dot < ZERO)[:, None], -b, b).astype(np.float32)
    t = amount[:, None]
    mixed = (a * (ONE - t) + bb * t).astype(np.float32)
    length_squared = ((mixed[:, 0] * mixed[:, 0] + mixed[:, 1] * mixed[:, 1]) + mixed[:, 2] * mixed[:, 2]) + mixed[:, 3] * mixed[:, 3]
    inv = (ONE / np.sqrt(length_squared)).astype(np.float32)
    out[:] = (mixed * inv[:, None]).astype(np.float32)
    out[first] = values[0]
    out[last & ~first] = values[n - 1]
    return out
