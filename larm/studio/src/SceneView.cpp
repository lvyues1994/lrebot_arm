#include "SceneView.h"

#include <QDebug>
#include <QMouseEvent>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QWheelEvent>

#include <algorithm>
#include <array>

namespace larm::studio {
namespace {

constexpr int kMaxGeoms = 4000;
constexpr int kMarkerGeoms = 4;

std::array<mjtNum, 3> toArray(Eigen::Vector3d const &vector) { return {vector.x(), vector.y(), vector.z()}; }

// Frames the bodies at the model's reference pose; the model's own extent includes the floor.
void frameBodies(mjModel const &model, mjData &data, mjvCamera &camera) {
    mj_forward(&model, &data);
    auto box = Eigen::AlignedBox3d{};
    for (int body = 1; body < model.nbody; ++body) {
        box.extend(Eigen::Vector3d{data.xpos[3 * body], data.xpos[3 * body + 1], data.xpos[3 * body + 2]});
    }
    if (box.isEmpty()) {
        return;
    }
    auto const center = box.center();
    for (int axis = 0; axis < 3; ++axis) {
        camera.lookat[axis] = center[axis];
    }
    camera.distance = std::max(0.5, 2.2 * box.diagonal().norm());
}

} // namespace

SceneView::SceneView(sim::SceneMirror &mirror_) : mirror{mirror_} {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    auto const &model = mirror.model();
    camera.type = mjCAMERA_FREE;
    camera.azimuth = model.vis.global.azimuth;
    camera.elevation = model.vis.global.elevation;
    frameBodies(model, mirror.data(), camera);
}

SceneView::~SceneView() {
    if (contextReady) {
        makeCurrent();
        mjr_freeContext(&context);
        doneCurrent();
    }
    mjv_freeScene(&scene);
}

void SceneView::setTarget(std::optional<Pose3> const &target_) {
    target = target_;
    update();
}

void SceneView::initializeGL() {
    if (defaultFramebufferObject() != 0) {
        qWarning() << "the window's default framebuffer is not 0; MuJoCo may draw elsewhere";
    }
    mjv_makeScene(&mirror.model(), &scene, kMaxGeoms);
    mjr_makeContext(&mirror.model(), &context, mjFONTSCALE_100);
    contextReady = true;
}

void SceneView::paintGL() {
    mjv_updateScene(&mirror.model(), &mirror.data(), &option, nullptr, &camera, mjCAT_ALL, &scene);
    addTargetMarker();
    auto const ratio = devicePixelRatio();
    auto const viewport =
        mjrRect{0, 0, static_cast<int>(width() * ratio), static_cast<int>(height() * ratio)};
    mjr_setBuffer(mjFB_WINDOW, &context);
    mjr_render(viewport, &scene, &context);
}

void SceneView::addTargetMarker() {
    if (not target or scene.ngeom + kMarkerGeoms > scene.maxgeom) {
        return;
    }
    auto const origin = toArray(target->translation);
    auto const sphere = std::array<mjtNum, 3>{0.012, 0.0, 0.0};
    auto const yellow = std::array<float, 4>{1.0f, 0.85f, 0.1f, 0.8f};
    mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, sphere.data(), origin.data(), nullptr,
                 yellow.data());
    auto const rotation = target->rotation.toRotationMatrix();
    for (int axis = 0; axis < 3; ++axis) {
        auto color = std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f};
        color[static_cast<std::size_t>(axis)] = 1.0f;
        auto const tip = toArray(target->translation + 0.08 * rotation.col(axis));
        auto *const geom = &scene.geoms[scene.ngeom++];
        mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, color.data());
        mjv_connector(geom, mjGEOM_ARROW, 0.004, origin.data(), tip.data());
    }
}

void SceneView::mousePressEvent(QMouseEvent *const event) { lastMouse = event->position(); }

void SceneView::mouseMoveEvent(QMouseEvent *const event) {
    auto const delta = event->position() - lastMouse;
    lastMouse = event->position();
    auto const shift = event->modifiers().testFlag(Qt::ShiftModifier);
    auto action = int{mjMOUSE_NONE};
    if (event->buttons().testFlag(Qt::LeftButton)) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else if (event->buttons().testFlag(Qt::RightButton)) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (event->buttons().testFlag(Qt::MiddleButton)) {
        action = mjMOUSE_ZOOM;
    }
    if (action == mjMOUSE_NONE or height() == 0) {
        return;
    }
    mjv_moveCamera(&mirror.model(), action, delta.x() / height(), delta.y() / height(), &scene, &camera);
    update();
}

void SceneView::wheelEvent(QWheelEvent *const event) {
    mjv_moveCamera(&mirror.model(), mjMOUSE_ZOOM, 0.0, -0.0005 * event->angleDelta().y(), &scene, &camera);
    update();
}

bool openGlAvailable() {
    auto surface = QOffscreenSurface{};
    surface.create();
    auto probe = QOpenGLContext{};
    return probe.create() and probe.makeCurrent(&surface);
}

} // namespace larm::studio
