#pragma once

#include <cstdint>
#include <iomanip>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>

namespace larm::probe {

// One JSON object per line: {"event": ..., fields...}.
struct JsonLine {
    explicit JsonLine(std::string_view const event) {
        out << std::setprecision(9) << "{\"event\":";
        quoted(event);
    }

    JsonLine &field(std::string_view const key, std::string_view const value) {
        name(key);
        quoted(value);
        return *this;
    }
    JsonLine &field(std::string_view const key, char const *const value) {
        return field(key, std::string_view{value});
    }
    JsonLine &field(std::string_view const key, bool const value) {
        name(key);
        out << (value ? "true" : "false");
        return *this;
    }
    JsonLine &field(std::string_view const key, double const value) {
        name(key);
        out << value;
        return *this;
    }
    JsonLine &field(std::string_view const key, std::uint64_t const value) {
        name(key);
        out << value;
        return *this;
    }
    JsonLine &field(std::string_view const key, int const value) {
        name(key);
        out << value;
        return *this;
    }
    JsonLine &field(std::string_view const key, std::span<double const> const values) {
        name(key);
        out << '[';
        for (std::size_t i = 0; i < values.size(); ++i) {
            out << (i == 0 ? "" : ",") << values[i];
        }
        out << ']';
        return *this;
    }

    void writeTo(std::ostream &stream) const { stream << out.str() << "}\n" << std::flush; }

  private:
    void name(std::string_view const key) {
        out << ',';
        quoted(key);
        out << ':';
    }

    void quoted(std::string_view const text) {
        out << '"';
        for (auto const c : text) {
            if (c == '"' or c == '\\') {
                out << '\\' << c;
            } else if (static_cast<unsigned char>(c) < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
                    << std::dec << std::setfill(' ');
            } else {
                out << c;
            }
        }
        out << '"';
    }

    std::ostringstream out;
};

} // namespace larm::probe
