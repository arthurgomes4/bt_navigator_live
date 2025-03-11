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
    _zmq_context(1),
    _zmq_subscriber(_zmq_context, ZMQ_SUB),
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

    zmq::message_t msg;
    try{
        while(  _zmq_subscriber.recv(msg) )
        {
            _msg_count++;
            ui->labelCount->setText( QString("Messages received: %1").arg(_msg_count) );

            const char* buffer = reinterpret_cast<const char*>(msg.data());

            const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>( buffer );
            const uint32_t num_transitions = flatbuffers::ReadScalar<uint32_t>( &buffer[4+header_size] );

            std::vector<std::pair<int, NodeStatus>> node_status;
            // check uid in the index, if failed load tree from server
            try{
                for(size_t offset = 4; offset < header_size +4; offset +=3 )
                {
                    const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset]);
                    _uid_to_index.at(uid);
                }

                for(size_t t=0; t < num_transitions; t++)
                {
                    size_t offset = 8 + header_size + 12*t;
                    const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);
                    _uid_to_index.at(uid);
                }

                for(size_t offset = 4; offset < header_size +4; offset +=3 )
                {
                    const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset]);
                    const uint16_t index = _uid_to_index.at(uid);
                    AbstractTreeNode* node = _loaded_tree.node( index );
                    node->status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+2] ));
                }

                //qDebug() << "--------";
                for(size_t t=0; t < num_transitions; t++)
                {
                    size_t offset = 8 + header_size + 12*t;

                    // const double t_sec  = flatbuffers::ReadScalar<uint32_t>( &buffer[offset] );
                    // const double t_usec = flatbuffers::ReadScalar<uint32_t>( &buffer[offset+4] );
                    // double timestamp = t_sec + t_usec* 0.000001;
                    const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);
                    const uint16_t index = _uid_to_index.at(uid);
                    // NodeStatus prev_status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[index+10] ));
                    NodeStatus status  = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+11] ));

                    _loaded_tree.node(index)->status = status;
                    node_status.push_back( {index, status} );

                }
            }
            catch( std::out_of_range& err) {
                qDebug() << "Reload tree from server";
                if( !getTreeFromServer() ) {
                    _connected = false;
                    ui->lineEdit_address->setDisabled(false);
                    _timer->stop();
                    connectionUpdate(false);
                    return;
                }
            }

            // update the graphic part
            emit changeNodeStyle( "BehaviorTree", node_status );

            // lock editing of nodes
            auto main_win = dynamic_cast<MainWindow*>( _parent );
            main_win->lockEditing(true);
        }
    }
    catch( zmq::error_t& err)
    {
        qDebug() << "ZMQ receive failed: " << err.what();
    }
}

bool SidepanelMonitor::getTreeFromServer()
{
    try{
        zmq::message_t request(0);
        zmq::message_t reply;

        zmq::socket_t  zmq_client( _zmq_context, ZMQ_REQ );
        zmq_client.connect( _connection_address_req.c_str() );

        zmq_client.setsockopt(ZMQ_RCVTIMEO, &_load_tree_timeout_ms, sizeof(int) );

        zmq_client.send(request, zmq::send_flags::none);

        auto bytes_received  = zmq_client.recv(reply, zmq::recv_flags::none);
        if( !bytes_received || *bytes_received == 0 )
        {
            return false;
        }

        // std::cout << "Reply data: ";
        // for (size_t i = 0; i < reply.size(); ++i) {
        //     std::cout << "{ ";
        //     for (size_t i = 0; i < reply.size(); ++i) {
        //         std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(reinterpret_cast<const unsigned char*>(reply.data())[i]);
        //         if (i != reply.size() - 1) {
        //             std::cout << ", ";
        //         }
        //     }
        //     std::cout << " }";
        // }
        // std::cout << std::dec << std::endl;

        const char* buffer = reinterpret_cast<const char*>(reply.data());

        auto fb_behavior_tree = Serialization::GetBehaviorTree( buffer );

        auto res_pair = BuildTreeFromFlatbuffers( fb_behavior_tree );

        _loaded_tree  = std::move( res_pair.first );
        _uid_to_index = std::move( res_pair.second );

        // add new models to registry
        for(const auto& tree_node: _loaded_tree.nodes())
        {
            const auto& registration_ID = tree_node.model.registration_ID;
            if( BuiltinNodeModels().count(registration_ID) == 0)
            {
                addNewModel( tree_node.model );
            }
        }

        try {
            loadBehaviorTree( _loaded_tree, "BehaviorTree" );
        }
        catch (std::exception& err) {
            QMessageBox messageBox;
            messageBox.critical(this,"Error Connecting to remote server", err.what() );
            messageBox.show();
            return false;
        }

        std::vector<std::pair<int, NodeStatus>> node_status;
        node_status.reserve(_loaded_tree.nodesCount());

        //  qDebug() << "--------";

        for(size_t t=0; t < _loaded_tree.nodesCount(); t++)
        {
            node_status.push_back( { t, _loaded_tree.nodes()[t].status } );
        }
        emit changeNodeStyle( "BehaviorTree", node_status );
    }
    catch( zmq::error_t& err)
    {
        qDebug() << "ZMQ client receive failed: " << err.what();
        return false;
    }
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

        bool failed = false;
        if( !address.isEmpty() )
        {
            _connection_address_pub = "tcp://" + address.toStdString() + ":" + publisher_port.toStdString();
            _connection_address_req = "tcp://" + address.toStdString() + ":" + server_port.toStdString();

            try{
                _zmq_subscriber.connect( _connection_address_pub.c_str() );

                int timeout_ms = 1;
                _zmq_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
                _zmq_subscriber.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(int) );

                if( !getTreeFromServer() )
                {
                    failed = true;
                    _connected = false;
                }
                // After we try get a tree on connect, reset to the default timeout.
                // This is done so that we only use the increased autoconnect timeout once.
                this->set_load_tree_timeout_ms(_load_tree_default_timeout_ms);
            }
            catch(zmq::error_t& err)
            {
                failed = true;
            }
        }
        else {
            failed = true;
        }

        if( !failed )
        {
            _connected = true;
            ui->lineEdit_address->setDisabled(true);
            ui->lineEdit_publisher->setDisabled(true);
            _timer->start(_timer_period_ms);
            connectionUpdate(true);
        }
        else{
            QMessageBox::warning(this,
                                 tr("ZeroMQ connection"),
                                 tr("Was not able to connect to [%1]\n").arg(_connection_address_pub.c_str()),
                                 QMessageBox::Close);
        }
    }
    else{
        _connected = false;
        ui->lineEdit_address->setDisabled(false);
        ui->lineEdit_publisher->setDisabled(false);
        _timer->stop();

        connectionUpdate(false);
    }
}
