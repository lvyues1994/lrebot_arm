// larm studio: watches and operates a robot served by a larm runtime node.
//
//   larm_studio --profile FILE [--server /larm_runtime]
//   larm_studio --profile FILE --script [--screenshots DIR]    drives itself through the UI and checks it
#include "MainWindow.h"
#include "Script.h"

#include <larm/model/RobotModel.h>
#include <larm/ros/RemoteSession.h>
#include <larm/sim/SceneMirror.h>

#include <lqtexec/EventLoopContext.h>
#include <lqtexec/ExecWithScope.h>
#include <lqtexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QCommandLineParser>
#include <QSurfaceFormat>

#include <thread>

namespace {

struct Options {
    QString profile;
    QString server;
    bool script{};
    std::optional<QDir> screenshots;
};

// ROS arguments, which launch files may append, are left to rclcpp.
Options parse(int const argc, char const *const *const argv) {
    auto parser = QCommandLineParser{};
    parser.addHelpOption();
    auto const profile = QCommandLineOption{QStringLiteral("profile"), QStringLiteral("Robot profile."),
                                            QStringLiteral("file")};
    auto const server = QCommandLineOption{QStringLiteral("server"), QStringLiteral("Runtime node name."),
                                           QStringLiteral("name"), QStringLiteral("/larm_runtime")};
    auto const script =
        QCommandLineOption{QStringLiteral("script"), QStringLiteral("Drive the studio and check it.")};
    auto const screenshots = QCommandLineOption{
        QStringLiteral("screenshots"), QStringLiteral("Save screenshots of the scripted run into <dir>."),
        QStringLiteral("dir")};
    parser.addOptions({profile, server, script, screenshots});
    auto arguments = QStringList{};
    for (auto const &argument : rclcpp::remove_ros_arguments(argc, argv)) {
        arguments.append(QString::fromStdString(argument));
    }
    parser.process(arguments);
    auto options = Options{
        .profile = parser.value(profile), .server = parser.value(server), .script = parser.isSet(script)};
    if (parser.isSet(screenshots)) {
        options.screenshots.emplace(parser.value(screenshots));
        options.screenshots->mkpath(QStringLiteral("."));
    }
    return options;
}

// MuJoCo's renderer needs the compatibility profile.
void requestOpenGl() {
    auto format = QSurfaceFormat{};
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);
}

int fail(QString const &message) {
    qCritical().noquote() << "larm_studio:" << message;
    return 2;
}

} // namespace

int main(int argc, char **argv) {
    // Before any thread, so that every thread inherits the blocked signals.
    auto stop = lexec::inplace_stop_source{};
    auto const signalStop = lqtexec::SignalStop{stop};
    requestOpenGl();
    auto app = QApplication{argc, argv};
    app.setQuitOnLastWindowClosed(false);
    // QApplication has taken its own arguments out of argv.
    auto const options = parse(argc, argv);
    if (options.profile.isEmpty()) {
        return fail(QStringLiteral("--profile is required"));
    }

    auto profile = larm::loadRobotProfile(options.profile.toStdString());
    if (not profile) {
        return fail(QString::fromStdString(profile.error().message));
    }
    auto model = larm::model::loadRobotModel(*profile);
    auto mirror = larm::sim::loadSceneMirror(*profile);
    if (not model or not mirror) {
        return fail(QString::fromStdString(model ? mirror.error().message : model.error().message));
    }

    auto initOptions = rclcpp::InitOptions{};
    initOptions.shutdown_on_signal = false;
    rclcpp::init(argc, argv, initOptions, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("larm_studio");
    auto session = larm::ros::makeRemoteSession(node, *profile, {.server = options.server.toStdString()});
    auto rosScope = lexec::counting_scope{};
    auto rosStop = lexec::inplace_stop_source{};
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    auto rosThread = std::thread{[&] { lrclexec::spin_with_scope(executor, rosScope, rosStop.get_token()); }};

    auto code = 0;
    auto passed = true;
    {
        auto gui = lqtexec::EventLoopContext{};
        auto appScope = lexec::counting_scope{};
        auto journal = larm::studio::Journal{};
        auto const context = larm::studio::StudioContext{.session = session.get(),
                                                         .model = model->get(),
                                                         .gui = gui.get_scheduler(),
                                                         .appScope = &appScope,
                                                         .journal = &journal};
        auto window = larm::studio::MainWindow{context, **mirror, stop};
        window.show();
        auto script = std::unique_ptr<larm::studio::Script>{};
        if (options.script) {
            script = std::make_unique<larm::studio::Script>(window, *session, journal, options.screenshots,
                                                            [&stop] { stop.request_stop(); });
            script->start();
        }
        code = lqtexec::exec_with_scope(app, appScope, stop);
        journal.write(QStringLiteral("app: drained, the event loop exited with %1").arg(code));
        passed = not script or script->verify();
    }

    // The GUI's operations have drained; the ROS side can go now, then the session it served.
    rosStop.request_stop();
    rosThread.join();
    session.reset();
    node.reset();
    rclcpp::shutdown();
    return passed ? code : 2;
}
