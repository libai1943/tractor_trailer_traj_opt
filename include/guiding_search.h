#pragma once
// Stable C interface to the protected planar A* implementation.
// Arrays are row-major. Grid arrays use index x + 101*y; coordinates are metres.
#ifdef _WIN32
#ifdef BUILD_GUIDING_SEARCH
#define SEARCH_API __declspec(dllexport)
#else
#define SEARCH_API __declspec(dllimport)
#endif
#else
#define SEARCH_API __attribute__((visibility("default")))
#endif
extern "C" SEARCH_API int search_guiding_path(const double* occupied,const double* potential,
    const double* start,const double* goal,int samples,double* output_xy);
