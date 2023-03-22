#ifndef FLIGHT_CONTROLLER_UTIL_PLATFORM_POSIX_FILE_DESCRIPTOR_H
#define FLIGHT_CONTROLLER_UTIL_PLATFORM_POSIX_FILE_DESCRIPTOR_H

#include <string>

namespace util {
    /**
     * @brief RAII-style file descriptor wrapper.
     *
     * The contained file descriptor is closed upon destruction. An instance of this
     * type can be passed to Linux system calls where a file descriptor is required.
     *
     * @invariant Always contains either a valid file descriptor or the sentinel
     * @c FileDescriptor::no_value after being moved from. A valid file descriptor
     * is a file descriptor which refers to an active file description in the
     * kernel.
     */
    class FileDescriptor {
    public:
        using raw_descriptor_type = int;

        FileDescriptor(const std::string& path_name, int flags, mode_t mode = 0);
        ~FileDescriptor() noexcept;

        FileDescriptor(const FileDescriptor& other) = delete;
        FileDescriptor& operator=(const FileDescriptor& other) = delete;
        FileDescriptor(FileDescriptor&& other) noexcept;
        FileDescriptor& operator=(FileDescriptor&& other) noexcept;

        operator raw_descriptor_type() const;// NOLINT(google-explicit-constructor)

    private:
        static constexpr raw_descriptor_type no_value = -1;

        raw_descriptor_type raw_descriptor_;
    };
}// namespace util

#endif// FLIGHT_CONTROLLER_UTIL_PLATFORM_POSIX_FILE_DESCRIPTOR_H
