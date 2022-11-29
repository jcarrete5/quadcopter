#include "defer.h"

#include <functional>
#include <iostream>
#include <utility>

using namespace util;

Defer::Defer(std::function<void()> callback)
    : callback(std::move(callback))
{
}

Defer::~Defer()
{
    try {
        callback();
    } catch (std::bad_function_call& e) {
        std::cerr << "error executing callback: " << e.what() << '\n';
    } catch (...) {
        std::cerr << "error executing callback\n";
    }
}
