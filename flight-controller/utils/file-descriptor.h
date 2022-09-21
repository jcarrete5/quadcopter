#ifndef UTIL_FILE_DESCRIPTOR_H
#define UTIL_FILE_DESCRIPTOR_H

#include <string>

/**
 * @brief RAII-style file descriptor wrapper
 *
 * `open` is called on construction
 * `close` is called on destruction
 * Allows for implicit conversion to `int`
 */
class FileDescriptor {
public:
    using raw_descriptor_type = int;

    FileDescriptor(const std::string& path_name, int flags, mode_t mode = 0) noexcept;
    ~FileDescriptor() noexcept;

    FileDescriptor(const FileDescriptor& other) = delete;
    FileDescriptor& operator=(const FileDescriptor& other) = delete;
    FileDescriptor(FileDescriptor&& other) noexcept;
    FileDescriptor& operator=(FileDescriptor&& other) noexcept;

    operator raw_descriptor_type() const; // NOLINT(google-explicit-constructor)

private:
    static constexpr raw_descriptor_type no_value = -1;

    raw_descriptor_type raw_descriptor_;
};

#endif// UTIL_FILE_DESCRIPTOR_H
