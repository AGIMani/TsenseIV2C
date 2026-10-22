#pragma once

#if defined(A3_SDK_SHARED)
#if defined(_WIN32)
#if defined(A3_SDK_BUILDING)
#define A3_API __declspec(dllexport)
#else
#define A3_API __declspec(dllimport)
#endif
#else
#define A3_API __attribute__((visibility("default")))
#endif
#else
#define A3_API
#endif
