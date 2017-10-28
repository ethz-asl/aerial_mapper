
#ifndef UTILS_COMMON_H_
#define UTILS_COMMON_H_

// SYSTEM
#include <iomanip>
#include <sstream>
#include <string>

namespace utils {
static constexpr int nameWidth = 30;

std::string paramToString(const std::string& name, double value);
std::string paramToString(const std::string& name, int value);
std::string paramToString(const std::string& name, bool value);
std::string paramToString(const std::string& name, const std::string& value);

}  // namespace utils

#endif  // COMMON_H_
