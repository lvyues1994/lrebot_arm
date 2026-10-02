#pragma once

#include <tl/expected.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace larm {

enum class ErrorCode : std::uint8_t {
    InvalidArgument,
    InvalidConfig,
    NotFound,
    Io,
    Unsupported,
    SolverFailed,
    Internal,
};

struct Error {
    ErrorCode code{};
    std::string message;
};

template <class T> using Expected = tl::expected<T, Error>;

inline tl::unexpected<Error> makeError(ErrorCode const code, std::string message) {
    return tl::make_unexpected(Error{.code = code, .message = std::move(message)});
}

std::string_view toString(ErrorCode code) noexcept;

} // namespace larm
