#pragma once

#define A3_SDK_VERSION_MAJOR 1
#define A3_SDK_VERSION_MINOR 1
#define A3_SDK_VERSION_PATCH 15
#define A3_SDK_VERSION "1.1.15"

namespace a3 {

/** This SDK release, not the H7 firmware version. CMake configure overwrites this header. */
inline const char *sdkVersion() noexcept { return A3_SDK_VERSION; }

} // namespace a3
