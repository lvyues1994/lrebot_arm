#include <larm/core/Error.h>

namespace larm {

std::string_view toString(ErrorCode const code) noexcept {
    switch (code) {
    case ErrorCode::InvalidArgument:
        return "invalid argument";
    case ErrorCode::InvalidConfig:
        return "invalid configuration";
    case ErrorCode::NotFound:
        return "not found";
    case ErrorCode::Io:
        return "i/o error";
    case ErrorCode::Unsupported:
        return "unsupported";
    case ErrorCode::SolverFailed:
        return "solver failed";
    case ErrorCode::Internal:
        return "internal error";
    }
    return "unknown error";
}

} // namespace larm
