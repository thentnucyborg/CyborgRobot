
#ifndef RVIZ_DEFAULT_PLUGIN_EXPORT_H
#define RVIZ_DEFAULT_PLUGIN_EXPORT_H

#ifdef RVIZ_DEFAULT_PLUGIN_STATIC_DEFINE
#  define RVIZ_DEFAULT_PLUGIN_EXPORT
#  define RVIZ_DEFAULT_PLUGIN_NO_EXPORT
#else
#  ifndef RVIZ_DEFAULT_PLUGIN_EXPORT
#    ifdef rviz_default_plugin_EXPORTS
        /* We are building this library */
#      define RVIZ_DEFAULT_PLUGIN_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define RVIZ_DEFAULT_PLUGIN_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef RVIZ_DEFAULT_PLUGIN_NO_EXPORT
#    define RVIZ_DEFAULT_PLUGIN_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef RVIZ_DEFAULT_PLUGIN_DEPRECATED
#  define RVIZ_DEFAULT_PLUGIN_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef RVIZ_DEFAULT_PLUGIN_DEPRECATED_EXPORT
#  define RVIZ_DEFAULT_PLUGIN_DEPRECATED_EXPORT RVIZ_DEFAULT_PLUGIN_EXPORT RVIZ_DEFAULT_PLUGIN_DEPRECATED
#endif

#ifndef RVIZ_DEFAULT_PLUGIN_DEPRECATED_NO_EXPORT
#  define RVIZ_DEFAULT_PLUGIN_DEPRECATED_NO_EXPORT RVIZ_DEFAULT_PLUGIN_NO_EXPORT RVIZ_DEFAULT_PLUGIN_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define RVIZ_DEFAULT_PLUGIN_NO_DEPRECATED
#endif

#endif
