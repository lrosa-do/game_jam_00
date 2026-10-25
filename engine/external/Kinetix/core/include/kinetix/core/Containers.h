#pragma once

#include "Types.h"

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

namespace kinetix {

template <typename K, typename V, typename Hash = ct::Hash<K>>
using HashMap = ct::HashMap<K, V, Hash>;

template <typename Tag> struct Handle
{
    u32 index = 0;
    u32 generation = 0;

    bool valid() const { return generation != 0; }
    explicit operator bool() const { return valid(); }

    bool operator==(const Handle& other) const
    {
        return index == other.index && generation == other.generation;
    }
    bool operator!=(const Handle& other) const { return !(*this == other); }
};

template <typename Tag> u64 packHandle(Handle<Tag> handle)
{
    return (static_cast<u64>(handle.index) << 32) | static_cast<u64>(handle.generation);
}

template <typename T, typename H> class Pool
{
public:
    H add(const T& value)
    {
        u32 index;
        if (!mFree.empty())
        {
            index = mFree.back();
            mFree.pop_back();
        }
        else
        {
            index = static_cast<u32>(mSlots.size());
            mSlots.push_back(Slot());
        }

        Slot& slot = mSlots[index];
        slot.value = value;
        slot.alive = true;
        if (slot.generation == 0)
            slot.generation = 1;

        H handle;
        handle.index = index;
        handle.generation = slot.generation;
        return handle;
    }

    T* get(H handle)
    {
        if (handle.generation == 0 || handle.index >= mSlots.size())
            return nullptr;
        Slot& slot = mSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;
        return &slot.value;
    }

    const T* get(H handle) const { return const_cast<Pool*>(this)->get(handle); }

    bool remove(H handle, T& out)
    {
        T* value = get(handle);
        if (!value)
            return false;

        out = *value;
        Slot& slot = mSlots[handle.index];
        slot.alive = false;
        slot.value = T();
        ++slot.generation;
        if (slot.generation == 0)
            slot.generation = 1;
        mFree.push_back(handle.index);
        return true;
    }

    template <typename Fn> void forEach(Fn function)
    {
        for (usize i = 0; i < mSlots.size(); ++i)
        {
            if (mSlots[i].alive)
                function(mSlots[i].value);
        }
    }

    usize liveCount() const { return mSlots.size() - mFree.size(); }
    usize capacity() const { return mSlots.size(); }

    void clear()
    {
        mSlots.clear();
        mFree.clear();
    }

private:
    struct Slot
    {
        T value = T();
        u32 generation = 0;
        bool alive = false;
    };

    ct::Vector<Slot> mSlots;
    ct::Vector<u32> mFree;
};

} // namespace kinetix
