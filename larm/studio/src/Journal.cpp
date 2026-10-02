#include "Journal.h"

#include <QDebug>

#include <algorithm>

namespace larm::studio {

void Journal::write(QString const &line) {
    qInfo().noquote() << "studio:" << line;
    auto const lock = std::lock_guard{mutex};
    lines.append(line);
}

bool Journal::contains(QString const &text) const {
    auto const lock = std::lock_guard{mutex};
    return std::any_of(lines.begin(), lines.end(), [&](QString const &line) { return line.contains(text); });
}

QStringList Journal::since(qsizetype const from) const {
    auto const lock = std::lock_guard{mutex};
    return lines.mid(from);
}

qsizetype Journal::size() const {
    auto const lock = std::lock_guard{mutex};
    return lines.size();
}

} // namespace larm::studio
