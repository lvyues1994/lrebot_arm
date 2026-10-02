#include "MainWindow.h"

#include <QCloseEvent>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QVBoxLayout>

namespace larm::studio {
namespace {

JointGroupSpec const *firstGroup(RobotProfile const &profile, bool const wantGripper) {
    for (auto const &group : profile.groups) {
        if (wantGripper ? group.joints.size() == 1 : not group.toolFrame.empty()) {
            return &group;
        }
    }
    return nullptr;
}

} // namespace

MainWindow::MainWindow(StudioContext const &context_, sim::SceneMirror &mirror_,
                       lexec::inplace_stop_source &stop_)
    : context{context_}, mirror{mirror_}, stop{stop_} {
    auto const &profile = context.session->profile();
    setWindowTitle(QStringLiteral("larm studio — %1").arg(QString::fromStdString(profile.name)));
    resize(1400, 860);

    if (openGlAvailable()) {
        view = new SceneView{mirror};
        viewportHost = QWidget::createWindowContainer(view);
    } else {
        context.journal->write(QStringLiteral("app: no OpenGL, the scene view is off"));
        auto *const placeholder = new QLabel{QStringLiteral("No OpenGL: the scene view is off")};
        placeholder->setAlignment(Qt::AlignCenter);
        viewportHost = placeholder;
    }
    viewportHost->setMinimumSize(640, 480);

    session = new SessionPanel{context};
    tabs = new QTabWidget;
    if (auto const *const arm = firstGroup(profile, false)) {
        joints = new JointPanel{context, *arm};
        cartesian = new CartesianPanel{context, *arm, [this](std::optional<Pose3> const &target) {
                                           if (view) {
                                               view->setTarget(target);
                                           }
                                       }};
        path = new PathPanel{context, *arm, [this] { return joints->targets(); }};
        tabs->addTab(joints, QStringLiteral("Joints"));
        tabs->addTab(cartesian, QStringLiteral("Cartesian"));
        tabs->addTab(path, QStringLiteral("Path"));
    }
    if (auto const *const hand = firstGroup(profile, true)) {
        gripper = new GripperPanel{context, *hand};
        tabs->addTab(gripper, QStringLiteral("Gripper"));
    }
    log = new QPlainTextEdit;
    log->setReadOnly(true);
    log->setMaximumBlockCount(500);

    auto *const side = new QWidget;
    auto *const sideLayout = new QVBoxLayout{side};
    sideLayout->addWidget(session);
    sideLayout->addWidget(tabs, 1);
    sideLayout->addWidget(log, 1);
    auto *const splitter = new QSplitter;
    splitter->addWidget(viewportHost);
    splitter->addWidget(side);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    setCentralWidget(splitter);

    connection = new QLabel{QStringLiteral("waiting for joint states")};
    statusBar()->addWidget(connection);

    QObject::connect(&timer, &QTimer::timeout, this, [this] { refresh(); });
    timer.start(33);
}

void MainWindow::closeEvent(QCloseEvent *const event) {
    context.journal->write(QStringLiteral("app: shutdown requested"));
    stop.request_stop();
    event->accept();
}

void MainWindow::refresh() {
    auto const snapshot = context.session->latest();
    if (snapshot.state.cycle > 0) {
        if (view) {
            mirror.show(snapshot.state.joints.position);
            view->update();
        }
        connection->setText(
            QStringLiteral("connected · robot time %1 s · feedback %2")
                .arg(toSeconds(snapshot.state.stamp.time_since_epoch()), 0, 'f', 2)
                .arg(snapshot.state.isFresh ? QStringLiteral("fresh") : QStringLiteral("stale")));
    }
    session->refresh(snapshot);
    if (joints) {
        joints->refresh(snapshot);
        cartesian->refresh(snapshot);
        path->refresh(snapshot);
    }
    if (gripper) {
        gripper->refresh(snapshot);
    }
    for (auto const &line : context.journal->since(journalShown)) {
        log->appendPlainText(line);
        ++journalShown;
    }
}

} // namespace larm::studio
