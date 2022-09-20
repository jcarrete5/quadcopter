#ifndef UTIL_FILE_DESCRIPTOR_H
#define UTIL_FILE_DESCRIPTOR_H

#include <string>

/**
 * @brief RAII-style file descriptor wrapper
 *
 * `open` is called on construction
 * `close` is called on destruction
 */
class file_descriptor {
    using file_descriptor_type = int;
    static constexpr file_descriptor_type no_value = -1;
public:
    file_descriptor(const std::string& path_name, int flags, mode_t mode = 0) noexcept;
    ~file_descriptor() noexcept;

    file_descriptor(const file_descriptor& other) = delete;
    file_descriptor& operator=(const file_descriptor& other) = delete;
    file_descriptor(file_descriptor&& other);
    file_descriptor& operator=(file_descriptor&& other);

    operator file_descriptor_type() const;

private:
    file_descriptor_type raw_descriptor_;
};

#endif// UTIL_FILE_DESCRIPTOR_H
