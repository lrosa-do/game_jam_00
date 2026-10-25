#pragma once

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

namespace kinetix {

using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;
using f32 = float;
using f64 = double;
using usize = size_t;

using Real = float;

} // namespace kinetix

#if defined(NDEBUG)
    #define KX_ASSERT(cond, msg) ((void)0)
#else
    #define KX_ASSERT(cond, msg) assert((cond) && (msg))
#endif

#define KX_NON_COPYABLE(Type) \
    Type(const Type&) = delete; \
    Type& operator=(const Type&) = delete
