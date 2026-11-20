#pragma once
// Runtime artifacts belong to a writable user directory, never the source tree.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unistd.h>

namespace fast_lio_runtime {
inline std::filesystem::path default_root() {
  const char * xdg = std::getenv("XDG_DATA_HOME");
  const char * user_home = std::getenv("HOME");
  auto base = xdg && *xdg ? std::filesystem::path(xdg) :
    (user_home && *user_home ? std::filesystem::path(user_home) / ".local/share" :
     std::filesystem::temp_directory_path());
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return base / "pm01_navigation/fast_lio" /
    (std::to_string(seconds) + "_" + std::to_string(::getpid()));
}
inline std::filesystem::path & root() {
  static auto value = default_root();
  return value;
}
inline void configure(const std::string & requested) {
  auto path = std::filesystem::path(requested);
  if (requested.rfind("~/", 0) == 0) {
    const char * user_home = std::getenv("HOME");
    if (user_home) path = std::filesystem::path(user_home) / requested.substr(2);
  }
  root() = std::filesystem::absolute(path);
  std::filesystem::create_directories(root() / "Log");
  std::filesystem::create_directories(root() / "PCD");
}
inline std::string file(const std::string & category, const std::string & name) {
  return (root() / category / name).string();
}
}  // namespace fast_lio_runtime
