#pragma once

#include <QString>
#include <QStringList>

#include <mutex>

namespace larm::studio {

// What the studio did, in order: shown in the log view and checked by the scripted run.
// Safe to write from any thread.
struct Journal {
    void write(QString const &line);
    bool contains(QString const &text) const;
    // Lines from `from` on.
    QStringList since(qsizetype from) const;
    qsizetype size() const;

  private:
    mutable std::mutex mutex;
    QStringList lines;
};

} // namespace larm::studio
