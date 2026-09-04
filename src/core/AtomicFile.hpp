#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace nes {

inline bool writeFileAtomically(const std::string& path, const uint8_t* data, std::size_t size,
    bool keepBackup = true, std::string* error = nullptr)
{
    namespace fs = std::filesystem;
    const fs::path target(path);
    const fs::path temp = target.string() + ".tmp";
    const fs::path backup = target.string() + ".bak";

    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        std::error_code cleanupEc;
        fs::remove(temp, cleanupEc);
        return false;
    };

    std::error_code ec;
    if (target.has_parent_path()) {
        fs::create_directories(target.parent_path(), ec);
        if (ec)
            return fail("Could not create save directory: " + ec.message());
    }

    {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        if (!out)
            return fail("Could not open temporary save file.");
        if (size != 0)
            out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
        out.flush();
        if (!out)
            return fail("Failed while writing temporary save file.");
        out.close();
        if (!out)
            return fail("Failed while closing temporary save file.");
    }

    const bool hadTarget = fs::exists(target, ec) && !ec;
    ec.clear();

    if (hadTarget) {
        fs::remove(backup, ec);
        ec.clear();
        fs::rename(target, backup, ec);
        if (ec)
            return fail("Could not preserve previous save: " + ec.message());
    }

    fs::rename(temp, target, ec);
    if (ec) {
        const std::string replaceError = ec.message();
        if (hadTarget) {
            std::error_code rollbackEc;
            fs::rename(backup, target, rollbackEc);
            if (rollbackEc)
                return fail("Could not install new save (" + replaceError + ") and rollback also failed (" + rollbackEc.message() + ").");
        }
        return fail("Could not install new save: " + replaceError);
    }

    if (hadTarget && !keepBackup) {
        ec.clear();
        fs::remove(backup, ec);
    }

    if (error) error->clear();
    return true;
}

}
