#pragma once

#include "Journal.h"
#include "MainWindow.h"

#include <QDir>
#include <QElapsedTimer>
#include <QTimer>

#include <functional>
#include <optional>
#include <vector>

namespace larm::studio {

// Drives the studio through its real buttons, as a user would: enable, joint move, pose move, jog,
// grip, a waypoint path, a long move stopped half way, emergency stop and reset, park, disable, quit.
// Each step waits for the previous one's outcome in the journal. Optionally saves screenshots.
struct Script {
    Script(MainWindow &window_, runtime::RobotSession &session_, Journal &journal_,
           std::optional<QDir> screenshots_, std::function<void()> quit_);

    void start();
    // After the event loop: whether every step ran and no operation failed.
    bool verify() const;

  private:
    struct Step {
        QString name;
        // Sees the journal lines written since the previous step acted.
        std::function<bool(QStringList const &)> ready;
        std::function<void()> act;
    };

    void tick();
    void capture(QString const &name);

    MainWindow &window;
    runtime::RobotSession &session;
    Journal &journal;
    std::optional<QDir> screenshots;
    std::function<void()> quit;
    std::vector<Step> steps;
    std::size_t next{};
    qsizetype mark{};
    QElapsedTimer sinceAct;
    QElapsedTimer total;
    QTimer timer;
    // Timed out, or the profile lacks the groups the script drives.
    bool failed{};
};

} // namespace larm::studio
