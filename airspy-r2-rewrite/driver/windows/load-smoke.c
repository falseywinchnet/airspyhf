#include "airspy.h"

#include <stdio.h>

int main(void)
{
    airspy_lib_version_t version = {0};
    airspy_lib_version(&version);
    printf(
        "airspy.dll %u.%u.%u loaded\n",
        version.major_version,
        version.minor_version,
        version.revision);
    return version.major_version == AIRSPY_VER_MAJOR
        && version.minor_version == AIRSPY_VER_MINOR
        && version.revision == AIRSPY_VER_REVISION
        ? 0
        : 1;
}
