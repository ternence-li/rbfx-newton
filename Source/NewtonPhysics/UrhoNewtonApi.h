#define URHONEWTON_EXPORT_API __declspec(dllexport)


#ifdef URHONEWTON_STATIC
#  define URHONEWTON_API
#  define URHONEWTON_NO_EXPORT
#else
#  ifndef URHONEWTON_API
#    ifdef URHONEWTON_EXPORTS
/* We are building this library */
#      define URHONEWTON_API URHONEWTON_EXPORT_API
#    else
/* We are using this library */
#      define URHONEWTON_API __declspec(dllimport)
#    endif
#  endif

#  ifndef URHONEWTON_NO_EXPORT
#    define URHONEWTON_NO_EXPORT 
#  endif
#endif
