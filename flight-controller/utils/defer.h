#ifndef QUADCOPTER_DEFER_H
#define QUADCOPTER_DEFER_H

#include <functional>
#include <iostream>
#include <utility>

/**
 * @brief Defer callback execution until destruction.
 *
 * The callback function will be called when the instance of Defer is
 * destroyed. Use this class to ensure a function is executed at the end of a
 * scope.
 */
class Defer {
public:
    explicit Defer(std::function<void()> callback)
        : callback{std::move(callback)} { }

    ~Defer()
    {
        try {
            callback();
        } catch (std::bad_function_call& e) {
            std::cerr << "error executing callback: " << e.what() << '\n';
        } catch (...) {
            std::cerr << "error executing callback\n";
        }
    }

private:
    std::function<void()> callback;
};

#endif  // QUADCOPTER_DEFER_H
