#define VERSION_MAJOR            1
#define VERSION_MINOR            0
#define VERSION_REVISION         2
#define VERSION_BUILDNUM         0
#define VERSION_GCID             0x2a83aca8
#define CUSTOMER_NAME            sdk
#define CN_1                     's'
#define CN_2                     'd'
#define CN_3                     'k'
#define CN_4                     '#'
#define CN_5                     '#'
#define CN_6                     '#'
#define CN_7                     '#'
#define CN_8                     '#'
#define BUILDING_TIME            "Fri Sep 21 13:01:17 2018"
#define NAME2STR(a)              #a
#define CUSTOMER_NAME_S          #NAME2STR(CUSTOMER_NAME)
#define NUM4STR(a,b,c,d)         #a "." #b "." #c "." #d
#define VERSIONBUILDSTR(a,b,c,d) NUM4STR(a,b,c,d)
#define VERSION_BUILD_STR        VERSIONBUILDSTR(VERSION_MAJOR,VERSION_MINOR,VERSION_REVISION,VERSION_BUILDNUM)
