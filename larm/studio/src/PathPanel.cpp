#include "PathPanel.h"

#include "Widgets.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QVBoxLayout>

namespace larm::studio {

PathPanel::PathPanel(StudioContext const &context_, JointGroupSpec group_, JointSource jointTargets_,
                     QWidget *const parent)
    : QWidget{parent}, context{context_}, group{std::move(group_)}, jointTargets{std::move(jointTargets_)},
      current{zeroJointVector(group.joints.size())} {
    list = new QListWidget;
    segment = makeSpin(0.2, 20.0, 0.5, 1, QStringLiteral(" s"));
    segment->setValue(1.5);
    addCurrent = new QPushButton{QStringLiteral("Add current")};
    addTargets = new QPushButton{QStringLiteral("Add joint targets")};
    remove = new QPushButton{QStringLiteral("Remove")};
    clear = new QPushButton{QStringLiteral("Clear")};
    run = new QPushButton{QStringLiteral("Run")};
    stop = new QPushButton{QStringLiteral("Stop")};
    auto *const buttons = new QGridLayout;
    buttons->addWidget(addCurrent, 0, 0);
    buttons->addWidget(addTargets, 0, 1);
    buttons->addWidget(remove, 1, 0);
    buttons->addWidget(clear, 1, 1);
    buttons->addWidget(run, 2, 0);
    buttons->addWidget(stop, 2, 1);
    auto *const form = new QFormLayout;
    form->addRow(QStringLiteral("Time between waypoints"), segment);
    auto *const layout = new QVBoxLayout{this};
    layout->addWidget(list);
    layout->addLayout(form);
    layout->addLayout(buttons);

    operations = std::make_unique<OperationRunner>(this, context, stop);
    QObject::connect(addCurrent, &QPushButton::clicked, this, [this] { append(current); });
    QObject::connect(addTargets, &QPushButton::clicked, this, [this] { append(jointTargets()); });
    QObject::connect(remove, &QPushButton::clicked, this, [this] {
        auto const row = list->currentRow();
        if (row >= 0) {
            waypoints.erase(waypoints.begin() + row);
            delete list->takeItem(row);
        }
    });
    QObject::connect(clear, &QPushButton::clicked, this, [this] {
        waypoints.clear();
        list->clear();
    });
    QObject::connect(run, &QPushButton::clicked, this, [this] {
        auto path = runtime::JointPath{};
        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            path.waypoints.push_back({.time = fromSeconds(segment->value() * static_cast<double>(i + 1)),
                                      .position = waypoints[i]});
        }
        auto *const motion = context.session->motion(group.name);
        operations->run(QStringLiteral("follow path"), motion->followPath(std::move(path)));
    });
}

void PathPanel::refresh(control::RobotSnapshot const &snapshot) {
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        current[idx(i)] = snapshot.state.joints.position[idx(group.joints[i])];
    }
}

void PathPanel::append(JointVector const &waypoint) {
    waypoints.push_back(waypoint);
    auto text = QStringList{};
    for (auto const value : waypoint) {
        text.append(formatJoint(value, 2));
    }
    list->addItem(QStringLiteral("%1: [%2]").arg(waypoints.size()).arg(text.join(QStringLiteral(", "))));
}

} // namespace larm::studio
