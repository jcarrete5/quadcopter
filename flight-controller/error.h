#ifndef FLIGHT_CONTROLLER_ERROR_H
#define FLIGHT_CONTROLLER_ERROR_H

#include <system_error>
#include <cerrno>

/**
 * @brief Check system call return value for errors.
 *
 * If the system call failed for any reason, throw an exception.
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

#endif  // FLIGHT_CONTROLLER_ERROR_H
