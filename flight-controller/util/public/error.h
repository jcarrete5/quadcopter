#ifndef FLIGHT_CONTROLLER_UTIL_PUBLIC_ERROR_H
#define FLIGHT_CONTROLLER_UTIL_PUBLIC_ERROR_H

#include <system_error>
#include <cerrno>

/**
 * @brief Check system call return value for errors.
 *
 * If the system call failed for any reason, throw an exception. most system
 * calls return -1 on failure in Linux. This function will check if the system
 * call returned -1.
 *
 * @param return_value Value returned by the system call.
 * @returns The value returned by the system call.
 * @throw std::system_error Exception wrapping errno returned by the system call.
 */
inline auto check_syscall(std::signed_integral auto return_value)
{
    if (return_value == -1) {
        throw std::system_error(errno, std::system_category());
    }
    return return_value;
}

/**
 * @brief Check pthread call return value for errors.
 *
 * pthread function calls return a positive error on failure and 0 on success.
 *
 * @param return_value The pthread function call return value.
 * @return The pthread function call return value.
 * @throw std::system_error Wrapped return value.
 */
inline auto check_pthread_call(std::signed_integral auto return_value)
{
    if (return_value != 0) {
        throw std::system_error(return_value, std::system_category());
    }
   return return_value;
}

#endif  // FLIGHT_CONTROLLER_UTIL_PUBLIC_ERROR_H
