#pragma once

#include <larm/core/Pose.h>
#include <larm/sim/SceneMirror.h>

#include <QOpenGLWindow>
#include <QPointF>

#include <optional>

namespace larm::studio {

// Renders the mirrored MuJoCo scene. A QOpenGLWindow rather than a QOpenGLWidget: MuJoCo draws its
// window buffer into the default framebuffer, which a widget would redirect to its own FBO.
// Left drag orbits, right drag pans, the wheel zooms. Create it only if openGlAvailable().
struct SceneView final : QOpenGLWindow {
    explicit SceneView(sim::SceneMirror &mirror_);
    ~SceneView() override;
    SceneView(SceneView const &) = delete;
    SceneView &operator=(SceneView const &) = delete;

    // A pose marker in world coordinates; nullopt hides it.
    void setTarget(std::optional<Pose3> const &target_);

  protected:
    void initializeGL() override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

  private:
    void addTargetMarker();

    sim::SceneMirror &mirror;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
    bool contextReady{};
    std::optional<Pose3> target;
    QPointF lastMouse;
};

// Whether an OpenGL context can be made current; QOpenGLWindow crashes without one.
bool openGlAvailable();

} // namespace larm::studio
