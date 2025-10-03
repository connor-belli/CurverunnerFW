#ifndef __UTIL_H_
#define __UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Limits value x to between min_val and max_val I.e. [min_val, max_val]
/// @param x Value to clamp
/// @param min_val Minimum bound of output
/// @param max_val Maximum bound of output
/// @return Clamped value of x
float clamp(float x, float min_val, float max_val);

/// @brief Chooses the maximum of x and y
/// @param x 
/// @param y 
/// @return Max value
float max(float x, float y);

#ifdef __cplusplus
}
#endif
#endif // __UTIL_H_