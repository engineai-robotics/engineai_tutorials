#include "output_paths.hpp"
#include <fstream>
#include <iostream>
int main() {
  const auto directory = std::filesystem::temp_directory_path() /
    ("pm01-output-paths-" + std::to_string(::getpid()));
  try {
    fast_lio_runtime::configure(directory.string());
    const auto log = fast_lio_runtime::file("Log", "check.txt");
    std::ofstream(log) << "runtime output";
    const auto pcd = fast_lio_runtime::file("PCD", "scans.pcd");
    const bool valid = std::filesystem::exists(log) &&
      std::filesystem::path(pcd).parent_path() == directory / "PCD" &&
      fast_lio_runtime::default_root().string().find("pm01_navigation/fast_lio") != std::string::npos;
    std::filesystem::remove_all(directory);
    return valid ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    std::filesystem::remove_all(directory);
    return 1;
  }
}
