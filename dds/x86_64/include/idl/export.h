
#ifndef IDL_EXPORT_H
#define IDL_EXPORT_H

#ifdef IDL_STATIC_DEFINE
#  define IDL_EXPORT
#  define IDL_NO_EXPORT
#else
#  ifndef IDL_EXPORT
#    ifdef idl_EXPORTS
        /* We are building this library */
#      define IDL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define IDL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef IDL_NO_EXPORT
#    define IDL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef IDL_DEPRECATED
#  define IDL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef IDL_DEPRECATED_EXPORT
#  define IDL_DEPRECATED_EXPORT IDL_EXPORT IDL_DEPRECATED
#endif

#ifndef IDL_DEPRECATED_NO_EXPORT
#  define IDL_DEPRECATED_NO_EXPORT IDL_NO_EXPORT IDL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef IDL_NO_DEPRECATED
#    define IDL_NO_DEPRECATED
#  endif
#endif

#ifndef IDL_INLINE_EXPORT
#  if __MINGW32__ && (!defined(__clang__) || !defined(idl_EXPORTS))
#    define IDL_INLINE_EXPORT
#  else
#    define IDL_INLINE_EXPORT IDL_EXPORT
#  endif
#endif

// Some internal functions are exported even though are not part of the API nor
// foreseen to ever be called by a user of the library (unlike some functions
// that are exported for convenience in building tools or even examples, such as
// the AVL tree).  One reason for this is that they are useful in instrumenting
// Cyclone DDS with some performance analysis tools, and it is in the interest
// of the projec that such analyses can be done.
//
// There is no guarantee that such internal symbols will remain available or
// that their role will be the same.
#ifndef IDL_EXPORT_INTERNAL_FUNCTION
#  define IDL_EXPORT_INTERNAL_FUNCTION IDL_EXPORT
#endif

#endif /* IDL_EXPORT_H */
