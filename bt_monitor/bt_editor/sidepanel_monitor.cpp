#include "sidepanel_monitor.h"
#include "ui_sidepanel_monitor.h"
#include <QLineEdit>
#include <QPushButton>
#include <QMessageBox>
#include <QTimer>
#include <QLabel>
#include <QDebug>

#include "mainwindow.h"
#include "utils.h"

SidepanelMonitor::SidepanelMonitor(QWidget *parent,
                                   rclcpp::Node::SharedPtr node_ptr) :
    QFrame(parent),
    ui(new Ui::SidepanelMonitor),
    _connected(false),
    _msg_count(0),
    _parent(parent),
    _node_ptr(node_ptr)
{
    // Register the type for Qt's signal/slot system
    qRegisterMetaType<std::vector<std::pair<int, NodeStatus>>>("std::vector<std::pair<int, NodeStatus>>");

    ui->setupUi(this);
    this->set_load_tree_timeout_ms(_load_tree_default_timeout_ms);

    const QString address = "127.0.0.1";
    const QString publisher_port = "1666";
    const QString server_port = "1667";

    if ( !address.isEmpty() )
    {
        ui->lineEdit_address->setText(address);
    }
    if ( !publisher_port.isEmpty() )
    {
        ui->lineEdit_publisher->setText(publisher_port);
    }
    if ( !server_port.isEmpty() )
    {
        ui->lineEdit_server->setText(server_port);
    }


    _timer = new QTimer(this);
    connect( _timer, &QTimer::timeout, this, &SidepanelMonitor::on_timer );

    // Initialize ROS2 subscribers
    full_bt_subscriber_ = _node_ptr->create_subscription<std_msgs::msg::ByteMultiArray>(
        "/full_bt", 10, std::bind(&SidepanelMonitor::fullBtCallback, this, std::placeholders::_1));

    bt_updates_subscriber_ = _node_ptr->create_subscription<std_msgs::msg::ByteMultiArray>(
        "/bt_updates", 10, std::bind(&SidepanelMonitor::btUpdatesCallback, this, std::placeholders::_1));
}

SidepanelMonitor::~SidepanelMonitor()
{
    delete ui;
}

void SidepanelMonitor::fullBtCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    const char* buffer = reinterpret_cast<const char*>(msg->data.data());

    auto fb_behavior_tree = Serialization::GetBehaviorTree(buffer);

    auto res_pair = BuildTreeFromFlatbuffers(fb_behavior_tree);

    _loaded_tree = std::move(res_pair.first);
    _uid_to_index = std::move(res_pair.second);

    // add new models to registry
    for (const auto& tree_node : _loaded_tree.nodes()) {
        const auto& registration_ID = tree_node.model.registration_ID;
        if (BuiltinNodeModels().count(registration_ID) == 0) {
            addNewModel(tree_node.model);
        }
    }

    try {
        loadBehaviorTree(_loaded_tree, "BehaviorTree");
    } catch (std::exception& err) {
        QMessageBox messageBox;
        messageBox.critical(this, "Error Loading Behavior Tree", err.what());
        messageBox.show();
        return;
    }

    std::vector<std::pair<int, NodeStatus>> node_status;
    node_status.reserve(_loaded_tree.nodesCount());

    for (size_t t = 0; t < _loaded_tree.nodesCount(); t++) {
        node_status.push_back({t, _loaded_tree.nodes()[t].status});
    }
    emit changeNodeStyle("BehaviorTree", node_status);
}

void SidepanelMonitor::btUpdatesCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    _msg_count++;
    ui->labelCount->setText( QString("Messages received: %1").arg(_msg_count) );

    qDebug() << "[SidepanelMonitor] Received update message with size:" << msg->data.size();

    const char* buffer = reinterpret_cast<const char*>(msg->data.data());

    const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>( buffer );
    const uint32_t num_transitions = flatbuffers::ReadScalar<uint32_t>( &buffer[4+header_size] );

    qDebug() << "[SidepanelMonitor] Message header_size:" << header_size << "num_transitions:" << num_transitions;

    std::vector<std::pair<int, NodeStatus>> node_status;
    // check uid in the index, if failed load tree from server
    try{
        // First pass - validate UIDs
        for(size_t offset = 4; offset < header_size +4; offset +=3 )
        {
            const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset]);
            if(_uid_to_index.find(uid) == _uid_to_index.end()) {
                qDebug() << "[SidepanelMonitor] Unknown UID in status update:" << uid;
                throw std::out_of_range("Unknown UID in status update");
            }
        }

        for(size_t t=0; t < num_transitions; t++)
        {
            size_t offset = 8 + header_size + 12*t;
            const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);
            if(_uid_to_index.find(uid) == _uid_to_index.end()) {
                qDebug() << "[SidepanelMonitor] Unknown UID in transition:" << uid;
                throw std::out_of_range("Unknown UID in transition");
            }
        }

        // Process status updates
        for(size_t offset = 4; offset < header_size +4; offset +=3 )
        {
            const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset]);
            const uint16_t index = _uid_to_index.at(uid);
            AbstractTreeNode* node = _loaded_tree.node( index );
            auto new_status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+2] ));
            node->status = new_status;
            node_status.push_back( {index, new_status} );
            qDebug() << "[SidepanelMonitor] Updated node" << uid << "index" << index << "to status" << static_cast<int>(new_status);
        }

        // Process transitions
        for(size_t t=0; t < num_transitions; t++)
        {
            size_t offset = 8 + header_size + 12*t;
            const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);
            const uint16_t index = _uid_to_index.at(uid);
            NodeStatus status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+11] ));

            _loaded_tree.node(index)->status = status;
            node_status.push_back( {index, status} );
            qDebug() << "[SidepanelMonitor] Transition for node" << uid << "index" << index << "to status" << static_cast<int>(status);
        }

        qDebug() << "[SidepanelMonitor] Emitting style changes for" << node_status.size() << "nodes";
        
        // update the graphic part
        emit changeNodeStyle( "BehaviorTree", node_status );

        // lock editing of nodes
        auto main_win = dynamic_cast<MainWindow*>( _parent );
        if (main_win) {
            main_win->lockEditing(true);
            qDebug() << "[SidepanelMonitor] Locked editing";
        }
    }
    catch( std::out_of_range& err) {
        qDebug() << "[SidepanelMonitor] Error processing update:" << err.what();
        return;
    }
}

void SidepanelMonitor::clear()
{
    if( _connected ) this->on_Connect();
}

void SidepanelMonitor::on_timer()
{
    if( !_connected ) return;
    
    // Timer is just for UI updates now, since ROS2 callbacks handle the data
    // The actual data processing happens in the ROS2 callbacks
}

bool SidepanelMonitor::getTreeFromServer()
{
    // Since we're using ROS2 topics, we don't need to actively request trees
    // The tree will be received via the /full_bt topic callback
    qDebug() << "[SidepanelMonitor] Waiting for behavior tree from ROS2 topic /full_bt";
    return true;
}

void SidepanelMonitor::on_Connect()
{
    if( !_connected )
    {
        QString address = ui->lineEdit_address->text();
        if( address.isEmpty() )
        {
            address = ui->lineEdit_address->placeholderText();
            ui->lineEdit_address->setText(address);
        }

        QString publisher_port = ui->lineEdit_publisher->text();
        if( publisher_port.isEmpty() )
        {
            publisher_port = ui->lineEdit_publisher->placeholderText();
            ui->lineEdit_publisher->setText(publisher_port);
        }

        QString server_port = ui->lineEdit_server->text();
        if( server_port.isEmpty() )
        {
          publisher_port = ui->lineEdit_server->placeholderText();
          ui->lineEdit_server->setText(publisher_port);
        }

        // For ROS2 mode, we don't need address/port configuration
        // ROS2 topics handle the communication
        bool failed = false;

        if( !failed )
        {
            _connected = true;
            ui->lineEdit_address->setDisabled(true);
            ui->lineEdit_publisher->setDisabled(true);
            _timer->start(_timer_period_ms);
            connectionUpdate(true);
            
            qDebug() << "[SidepanelMonitor] Connected - listening for ROS2 topics /full_bt and /bt_updates";
        }
        else{
            QMessageBox::warning(this,
                                 tr("ROS2 connection"),
                                 tr("Unable to start ROS2 monitoring"),
                                 QMessageBox::Close);
        }
    }
    else{
        _connected = false;
        ui->lineEdit_address->setDisabled(false);
        ui->lineEdit_publisher->setDisabled(false);
        _timer->stop();

        connectionUpdate(false);
        qDebug() << "[SidepanelMonitor] Disconnected from ROS2 monitoring";
    }
}
