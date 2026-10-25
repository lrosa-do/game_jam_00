#pragma once

#include "prisma3d/core/Archive.h"

#include <ct/thread.hpp>

#include "miniz.h"

namespace prisma3d
{

class ZipArchive : public Archive
{
public:
    ZipArchive();
    ~ZipArchive() override;

    ZipArchive(const ZipArchive&) = delete;
    ZipArchive& operator=(const ZipArchive&) = delete;

    bool openFromMemory(ct::Vector<std::uint8_t>&& zipData);

    bool exists(const ct::String& name) const override;
    ct::Vector<std::uint8_t> readBinary(const ct::String& name) const override;
    IoStream* open(const ct::String& name) const override;

private:
    static ct::String normalize(const ct::String& name);

    ct::Vector<std::uint8_t> mData;
    mutable mz_zip_archive mZip;
    mutable ct::Mutex mZipMutex;
    bool mOpen;
};

} // namespace prisma3d
