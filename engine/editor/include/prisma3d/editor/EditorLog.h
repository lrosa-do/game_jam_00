#pragma once

#include <ct/string.hpp>
#include <ct/thread.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class EditorLog
{
public:
    struct Entry
    {
        int level = 1;
        bool script = false;
        ct::String text;
    };

    static constexpr std::size_t kLimit = 4000;

    static EditorLog& instance();

    void capture(bool on);
    bool capturing() const;

    void post(int level, const char* text, bool script = false);
    std::size_t total() const;
    std::size_t read(std::size_t& cursor, ct::Vector<Entry>& out) const;

private:
    EditorLog();
    ~EditorLog();
    EditorLog(const EditorLog&) = delete;
    EditorLog& operator=(const EditorLog&) = delete;

    static void sink(int level, const char* message, void* user);
    static void print(const char* line, void* user);

    mutable ct::Mutex mLock;
    ct::Vector<Entry> mEntries;
    std::size_t mFirst;
    int mCaptures;
};

} // namespace prisma3d
