#pragma once

#include "CartesianPanel.h"
#include "GripperPanel.h"
#include "JointPanel.h"
#include "PathPanel.h"
#include "SceneView.h"
#include "SessionPanel.h"

#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QTimer>

namespace larm::studio {

// The viewport with the session, joint, Cartesian, gripper and path panels and the journal.
// Uses the first group with a tool frame as the arm and the first single-joint group as the gripper.
// Closing it requests the application's stop.
struct MainWindow final : QMainWindow {
    MainWindow(StudioContext const &context_, sim::SceneMirror &mirror_, lexec::inplace_stop_source &stop_);

    SessionPanel *sessionPanel() const { return session; }
    JointPanel *jointPanel() const { return joints; }
    CartesianPanel *cartesianPanel() const { return cartesian; }
    GripperPanel *gripperPanel() const { return gripper; }
    PathPanel *pathPanel() const { return path; }
    // Null without OpenGL.
    SceneView *sceneView() const { return view; }
    // The widget hosting the scene view's native window, or its placeholder.
    QWidget *viewport() const { return viewportHost; }
    // Shows the session's latest snapshot and new journal lines; also runs on a 30 Hz timer.
    void refresh();
    void showPanel(QWidget *panel) { tabs->setCurrentWidget(panel); }

  protected:
    void closeEvent(QCloseEvent *event) override;

  private:
    StudioContext context;
    sim::SceneMirror &mirror;
    lexec::inplace_stop_source &stop;
    SceneView *view{};
    QWidget *viewportHost{};
    SessionPanel *session{};
    JointPanel *joints{};
    CartesianPanel *cartesian{};
    GripperPanel *gripper{};
    PathPanel *path{};
    QTabWidget *tabs{};
    QPlainTextEdit *log{};
    QLabel *connection{};
    QTimer timer;
    qsizetype journalShown{};
};

} // namespace larm::studio
