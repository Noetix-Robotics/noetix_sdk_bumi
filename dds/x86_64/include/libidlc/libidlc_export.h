
#ifndef LIBIDLC_EXPORT_H
#define LIBIDLC_EXPORT_H

#ifdef LIBIDLC_STATIC_DEFINE
#  define LIBIDLC_EXPORT
#  define LIBIDLC_NO_EXPORT
#else
#  ifndef LIBIDLC_EXPORT
#    ifdef libidlc_EXPORTS
        /* We are building this library */
#      define LIBIDLC_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define LIBIDLC_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef LIBIDLC_NO_EXPORT
#    define LIBIDLC_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef LIBIDLC_DEPRECATED
#  define LIBIDLC_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef LIBIDLC_DEPRECATED_EXPORT
#  define LIBIDLC_DEPRECATED_EXPORT LIBIDLC_EXPORT LIBIDLC_DEPRECATED
#endif

#ifndef LIBIDLC_DEPRECATED_NO_EXPORT
#  define LIBIDLC_DEPRECATED_NO_EXPORT LIBIDLC_NO_EXPORT LIBIDLC_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef LIBIDLC_NO_DEPRECATED
#    define LIBIDLC_NO_DEPRECATED
#  endif
#endif

#ifndef LIBIDLC_INLINE_EXPORT
#  if __MINGW32__ && (!defined(__clang__) || !defined(libidlc_EXPORTS))
#    define LIBIDLC_INLINE_EXPORT
#  else
#    define LIBIDLC_INLINE_EXPORT LIBIDLC_EXPORT
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
#ifndef LIBIDLC_EXPORT_INTERNAL_FUNCTION
#  define LIBIDLC_EXPORT_INTERNAL_FUNCTION LIBIDLC_EXPORT
#endif

#endif /* LIBIDLC_EXPORT_H */
