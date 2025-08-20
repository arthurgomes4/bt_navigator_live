#include <QApplication>
#include <QDialog>
#include <nodes/NodeStyle>
#include <nodes/FlowViewStyle>
#include <nodes/ConnectionStyle>
#include <nodes/DataModelRegistry>

#include "mainwindow.h"
#include "XML_utilities.hpp"
#include "models/RootNodeModel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>

using QtNodes::DataModelRegistry;
using QtNodes::FlowViewStyle;
using QtNodes::NodeStyle;
using QtNodes::ConnectionStyle;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr bt_monitor_node_ptr = std::make_shared<rclcpp::Node>("bt_monitor_node_ptr");

    std::thread ros_thread([&]() {
        rclcpp::spin(bt_monitor_node_ptr);
    });

    QApplication app(argc, argv);
    app.setApplicationName("BT Monitor");
    app.setWindowIcon(QPixmap(":/icons/BT.png"));
    app.setOrganizationName("EurecatRobotics");
    app.setOrganizationDomain("eurecat.org");

    qRegisterMetaType<AbsBehaviorTree>();

    QFile styleFile(":/stylesheet.qss");
    styleFile.open(QFile::ReadOnly);
    QString style(styleFile.readAll());
    app.setStyleSheet(style);

    // Start the main application in editor mode
    MainWindow win(GraphicMode::EDITOR, bt_monitor_node_ptr);
    win.show();

    int result = app.exec();

    // Shutdown ROS2
    rclcpp::shutdown();
    ros_thread.join();

    return result;
}
