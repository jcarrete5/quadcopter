#include <iostream>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#include "file-descriptor.h"
#include "error.h"

/**
 * @brief Open a new file descriptor.
 *
 * See open(2) for more information.
 *
 * @param path_name Path to open.
 * @param flags flags to open the file with.
 * @param mode mode to open the file with.
 */
FileDescriptor::FileDescriptor(const std::string& path_name, int flags, mode_t mode)
{
    try {
        check_syscall(raw_descriptor_ = open(path_name.c_str(), flags, mode));
    } catch (std::system_error& e) {
        std::cerr << "failed to open file: " << e.what() << '\n';
        std::cerr << "  path_name: " << path_name << '\n'
                  << "  flags: " << flags << '\n'
                  << "  mode: " << mode << '\n';
        throw;
    }
}

FileDescriptor::FileDescriptor(FileDescriptor&& other) noexcept
    : raw_descriptor_(other.raw_descriptor_)
{
    other.raw_descriptor_ = no_value;
}

FileDescriptor& FileDescriptor::operator=(FileDescriptor&& other) noexcept
{
    raw_descriptor_ = other.raw_descriptor_;
    other.raw_descriptor_ = no_value;
    return *this;
}

FileDescriptor::~FileDescriptor() noexcept
{
    if (raw_descriptor_ == no_value) {
        // No need to clean up an empty file descriptor
        return;
    }

    try {
        check_syscall(close(raw_descriptor_));
    } catch (std::system_error& e) {
        std::cerr << "failed to close file: " << e.what() << '\n';
    }
}

FileDescriptor::operator raw_descriptor_type() const
{
    return raw_descriptor_;
}
