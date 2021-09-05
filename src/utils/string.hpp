//
// By Jet <i@jetd.me> 2021.
// Copied from Kuafu repo
//
#pragma once

#include<string>
#include<filesystem>

namespace sapien::utils {

inline bool iequals(std::string_view a, std::string_view b) {
    return std::equal(a.begin(), a.end(),
                      b.begin(), b.end(),
                      [](char a, char b) { return tolower(a) == tolower(b); });
}

// @pram ext: file extension, should contain dot '.'
inline bool hasExtension(std::string_view filename, std::string_view ext) {
    auto fileExt = std::filesystem::path(filename).extension().string();
    return iequals(fileExt, ext);
}

// @pram exts: list of file extensions, should contain dot '.'
inline bool hasExtension(std::string_view filename, const std::vector<std::string_view>& exts) {
    auto fileExt = std::filesystem::path(filename).extension().string();

//    for (auto ext: exts)
//        if (iequals(fileExt, ext))
//            return true;
//    return false;
    return std::any_of(exts.begin(), exts.end(),
                   [fileExt](auto a) { return iequals(fileExt, a); });
}

}

