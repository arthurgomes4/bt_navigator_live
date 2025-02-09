#include <QApplication>
#include <QDialog>
#include <nodes/NodeStyle>
#include <nodes/FlowViewStyle>
#include <nodes/ConnectionStyle>
#include <nodes/DataModelRegistry>

#include "mainwindow.h"
#include "XML_utilities.hpp"
#include "startup_dialog.h"
#include "models/RootNodeModel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using QtNodes::DataModelRegistry;
using QtNodes::FlowViewStyle;
using QtNodes::NodeStyle;
using QtNodes::ConnectionStyle;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr bt_monitor_node = std::make_shared<rclcpp::Node>("bt_monitor_node");

    std::thread ros_thread([&]() {
        rclcpp::spin(bt_monitor_node);
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

    // Set the mode to MONITOR
    auto mode = GraphicMode::MONITOR;

    // Default monitor options
    const QString monitor_address = "localhost";
    const QString monitor_pub_port = "1666";
    const QString monitor_srv_port = "1667";
    const bool monitor_autoconnect = false;

    // Start the main application in monitor mode
    MainWindow win(mode, monitor_address, monitor_pub_port, monitor_srv_port, monitor_autoconnect);
    win.show();

    int result = app.exec();

    // Shutdown ROS2
    rclcpp::shutdown();
    ros_thread.join();

    return result;
}
