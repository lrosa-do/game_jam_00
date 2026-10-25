#include "ZipArchive.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

Archive::~Archive()
{
}

ZipArchive::ZipArchive()
        : mOpen(false)
{
    memset(&mZip, 0, sizeof(mZip));
}

ZipArchive::~ZipArchive()
{
    if (mOpen) mz_zip_reader_end(&mZip);
}

ct::String ZipArchive::normalize(const ct::String& name)
{
    ct::String out(name);
    char* data = out.data();
    for (std::size_t i = 0; i < out.size(); ++i)
        if (data[i] == '\\') data[i] = '/';
    return out;
}

bool ZipArchive::openFromMemory(ct::Vector<std::uint8_t>&& zipData)
{
    mData = static_cast<ct::Vector<std::uint8_t>&&>(zipData);

    memset(&mZip, 0, sizeof(mZip));
    if (!mz_zip_reader_init_mem(&mZip, mData.data(), mData.size(), 0))
    {
        log_error("ZipArchive: mz_zip_reader_init_mem failed (%s)",
                mz_zip_get_error_string(mz_zip_get_last_error(&mZip)));
        return false;
    }

    mOpen = true;
    return true;
}

bool ZipArchive::exists(const ct::String& name) const
{
    if (!mOpen) return false;
    const ct::String n = normalize(name);
    ct::LockGuard lock(mZipMutex);
    return mz_zip_reader_locate_file(&mZip, n.c_str(), nullptr, 0) >= 0;
}

ct::Vector<std::uint8_t> ZipArchive::readBinary(const ct::String& name) const
{
    ct::Vector<std::uint8_t> data;
    if (!mOpen) return data;

    const ct::String n = normalize(name);
    ct::LockGuard lock(mZipMutex);
    const int index = mz_zip_reader_locate_file(&mZip, n.c_str(), nullptr, 0);
    if (index < 0) return data;

    mz_zip_archive_file_stat stat;
    if (!mz_zip_reader_file_stat(&mZip, static_cast<mz_uint>(index), &stat) || stat.m_is_directory)
        return data;

    data.resize(static_cast<std::size_t>(stat.m_uncomp_size));
    if (!mz_zip_reader_extract_to_mem(&mZip, static_cast<mz_uint>(index), data.data(), data.size(),
                0))
    {
        log_error("ZipArchive: failed to extract %s (%s)", name.c_str(),
                mz_zip_get_error_string(mz_zip_get_last_error(&mZip)));
        data.clear();
    }
    return data;
}

IoStream* ZipArchive::open(const ct::String& name) const
{
    if (!mOpen) return nullptr;

    const ct::String n = normalize(name);
    ct::LockGuard lock(mZipMutex);
    const int index = mz_zip_reader_locate_file(&mZip, n.c_str(), nullptr, 0);
    if (index < 0) return nullptr;

    mz_zip_archive_file_stat stat;
    if (!mz_zip_reader_file_stat(&mZip, static_cast<mz_uint>(index), &stat) || stat.m_is_directory)
        return nullptr;
    if (stat.m_uncomp_size == 0) return io_open_memory(nullptr, 0);

    std::size_t size = 0;
    void* buffer = mz_zip_reader_extract_to_heap(&mZip, static_cast<mz_uint>(index), &size, 0);
    if (!buffer)
    {
        log_error("ZipArchive: failed to extract %s (%s)", name.c_str(),
                mz_zip_get_error_string(mz_zip_get_last_error(&mZip)));
        return nullptr;
    }

    IoStream* stream = io_open_memory_owned(buffer, size, mz_free);
    if (!stream) mz_free(buffer);
    return stream;
}

} // namespace prisma3d
