// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef REEL__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define REEL__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_reel __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_reel __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_reel __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_reel __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_reel
    #define ROSIDL_GENERATOR_C_PUBLIC_reel ROSIDL_GENERATOR_C_EXPORT_reel
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_reel ROSIDL_GENERATOR_C_IMPORT_reel
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_reel __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_reel
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_reel __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_reel
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // REEL__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
