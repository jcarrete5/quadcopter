#ifndef FLIGHT_CONTROLLER_UTIL_PUBLIC_FILE_DESCRIPTOR_H
#define FLIGHT_CONTROLLER_UTIL_PUBLIC_FILE_DESCRIPTOR_H

#ifdef __unix__
#include <unistd.h>
#if _POSIX_VERSION >= 198809L
#include "../platform/posix/file-descriptor.h"
#endif
#endif

#endif  // FLIGHT_CONTROLLER_UTIL_PUBLIC_FILE_DESCRIPTOR_H
