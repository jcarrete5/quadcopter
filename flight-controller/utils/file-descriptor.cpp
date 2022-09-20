#include "file-descriptor.h"
#include "utils/error.h"

#include <iostream>
#include <string>

#include <fcntl.h>
#include <unistd.h>

file_descriptor::file_descriptor(const std::string& path_name, int flags, mode_t mode) noexcept
{
    try {
        check_syscall(raw_descriptor_ = open(path_name.c_str(), flags, mode));
    } catch (std::system_error& e) {
        std::cerr << "failed to open file: " << e.what() << '\n';
        std::cerr << "  path_name: " << path_name << '\n'
                  << "  flags: " << flags << '\n'
                  << "  mode: " << mode << '\n';
    }
}

file_descriptor::file_descriptor(file_descriptor&& other)
    : raw_descriptor_(other.raw_descriptor_)
{
    other.raw_descriptor_ = no_value;
}

file_descriptor& file_descriptor::operator=(file_descriptor&& other)
{
    raw_descriptor_ = other.raw_descriptor_;
    other.raw_descriptor_ = no_value;
    return *this;
}

file_descriptor::~file_descriptor() noexcept
{
    try {
        check_syscall(close(raw_descriptor_));
    } catch (std::system_error& e) {
        std::cerr << "failed to close file: " << e.what() << '\n';
    }
}

file_descriptor::operator file_descriptor_type() const
{
    return raw_descriptor_;
}

