#ifndef FLIGHT_CONTROLLER_UTIL_PUBLIC_DEFER_H
#define FLIGHT_CONTROLLER_UTIL_PUBLIC_DEFER_H

#include <functional>

namespace util {
    /**
     * @brief Defer callback execution until destruction.
     *
     * The callback function will be called when the instance of Defer is
     * destroyed. Use this class to ensure a function is executed at the end of a
     * scope.
     */
    class Defer {
    public:
        explicit Defer(std::function<void()> callback);
        ~Defer();

    private:
        std::function<void()> callback;
    };
}// namespace util

#endif// FLIGHT_CONTROLLER_UTIL_PUBLIC_DEFER_H
