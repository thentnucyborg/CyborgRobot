
#ifndef RVIZ_EXPORT_H
#define RVIZ_EXPORT_H

#ifdef RVIZ_STATIC_DEFINE
#  define RVIZ_EXPORT
#  define RVIZ_NO_EXPORT
#else
#  ifndef RVIZ_EXPORT
#    ifdef rviz_EXPORTS
        /* We are building this library */
#      define RVIZ_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define RVIZ_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef RVIZ_NO_EXPORT
#    define RVIZ_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef RVIZ_DEPRECATED
#  define RVIZ_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef RVIZ_DEPRECATED_EXPORT
#  define RVIZ_DEPRECATED_EXPORT RVIZ_EXPORT RVIZ_DEPRECATED
#endif

#ifndef RVIZ_DEPRECATED_NO_EXPORT
#  define RVIZ_DEPRECATED_NO_EXPORT RVIZ_NO_EXPORT RVIZ_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define RVIZ_NO_DEPRECATED
#endif

#endif
