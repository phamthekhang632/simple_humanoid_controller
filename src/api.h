#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SimpleHumanoidController_DLLIMPORT __declspec(dllimport)
#  define SimpleHumanoidController_DLLEXPORT __declspec(dllexport)
#  define SimpleHumanoidController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SimpleHumanoidController_DLLIMPORT __attribute__((visibility("default")))
#    define SimpleHumanoidController_DLLEXPORT __attribute__((visibility("default")))
#    define SimpleHumanoidController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SimpleHumanoidController_DLLIMPORT
#    define SimpleHumanoidController_DLLEXPORT
#    define SimpleHumanoidController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SimpleHumanoidController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SimpleHumanoidController_DLLAPI
#  define SimpleHumanoidController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SimpleHumanoidController_EXPORTS
#    define SimpleHumanoidController_DLLAPI SimpleHumanoidController_DLLEXPORT
#  else
#    define SimpleHumanoidController_DLLAPI SimpleHumanoidController_DLLIMPORT
#  endif // SimpleHumanoidController_EXPORTS
#  define SimpleHumanoidController_LOCAL SimpleHumanoidController_DLLLOCAL
#endif // SimpleHumanoidController_STATIC