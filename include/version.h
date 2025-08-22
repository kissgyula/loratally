#define VERSION_STR "1.0.10"
#define VERSION_NUMBER 10

#ifdef DEBUG
    #define BUILD_STR "DEBUG"
#else
    #define BUILD_STR "RELEASE"
#endif

#ifdef HELTEC
    #define PLATFORM_STR "HELTEC"
#endif
#ifdef HELTECV3
    #define PLATFORM_STR "HELTECV3"
#else
    #define PLATFORM_STR "POE"
#endif
