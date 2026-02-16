// Copyright 2024 Laboratoire des signaux et systèmes
//
// This program is free software: you can redistribute it and/or 
// modify it under the terms of the GNU General Public License as 
// published by the Free Software Foundation, either version 3 of 
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License 
// along with this program. If not, see <https://www.gnu.org/licenses/>. 
//
// Author: Aarsh Thakker <aarsh.thakker@centralesupelec.fr>

#include "natnet_ros2/natnet_ros2.hpp"
#include "natnet_ros2/nn_filter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Geometry>

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


NatNetNode::NatNetNode(rclcpp::NodeOptions& node_options) : LifecycleNode("natnet_ros2_node")
{
    //declare_parameter<std::string>("node_name", "");
    declare_parameter<bool>("log_internals", false);
    declare_parameter<bool>("log_frames", false);
    declare_parameter<bool>("log_latencies", false);
    declare_parameter<bool>("pub_rigid_body", true);
    declare_parameter<bool>("pub_rigid_body_marker", false);
    declare_parameter<bool>("pub_individual_marker", false);
    declare_parameter<bool>("pub_pointcloud", false);
    declare_parameter<std::string>("individual_marker_msg_type","PoseStamped");
    declare_parameter<std::string>("global_frame", "world");
    declare_parameter<bool>("remove_latency",false);
    declare_parameter<std::string>("serverIP", "192.168.0.100");
    declare_parameter<std::string>("clientIP", "192.168.0.103");
    declare_parameter<std::string>("serverType", "multicast");
    declare_parameter<std::string>("multicastAddress", "239.255.42.99");
    declare_parameter<int>("serverCommandPort", 1510);
    declare_parameter<int>("serverDataPort", 1511);
    declare_parameter<bool>("use_kalman_filter", true);
    declare_parameter<double>("odom_max_accel", 10.0);
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

}

NatNetNode::~NatNetNode()
{
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
    }
}

void NatNetNode::get_node_params()
{
    log_internals = get_parameter("log_internals").as_bool();
    log_frames = get_parameter("log_frames").as_bool();
    log_latencies = get_parameter("log_latencies").as_bool();
    pub_rigid_body = get_parameter("pub_rigid_body").as_bool();
    pub_rigid_body_marker = get_parameter("pub_rigid_body_marker").as_bool();
    pub_individual_marker = get_parameter("pub_individual_marker").as_bool();
    pub_pointcloud = get_parameter("pub_pointcloud").as_bool();
    immt = get_parameter("individual_marker_msg_type").as_string();
    global_frame = get_parameter("global_frame").as_string();
    remove_latency = get_parameter("remove_latency").as_bool();
    use_kalman_filter = get_parameter("use_kalman_filter").as_bool();
    odom_max_accel = get_parameter("odom_max_accel").as_double();
    if (pub_individual_marker)
    {
        declare_parameter<std::vector<std::string>>("object_names", {});
        object_names = get_parameter("object_names").as_string_array();
        if(object_names.size()>0)
        {
            RCLCPP_INFO(get_logger(),"Got list of %li objects", object_names.size());
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"Unable to get the list of objects");
            RCLCPP_ERROR(get_logger(),"Signaling node shutdown");
            rclcpp::shutdown();
        }
        for (int i=0 ; i < (int)object_names.size() ; i++)
        {
            std::vector<double> tmp_pose;
            declare_parameter(object_names[i]+".pose.position", tmp_pose);
            declare_parameter(object_names[i]+".marker_config", tmp_obj.marker_config);
            get_parameter(object_names[i]+".pose.position", tmp_pose);
            if (tmp_pose.size()!=3)
            {
                RCLCPP_ERROR(get_logger(),"More/less than 3 data for the position array received for Object name : %s",object_names[i].c_str());
                rclcpp::shutdown();
            }
            tmp_obj.name = object_names[i];
            tmp_obj.x = tmp_pose[0];
            tmp_obj.y = tmp_pose[1];
            tmp_obj.z = tmp_pose[2];
            tmp_obj.detected = false;
            object_list.push_back(tmp_obj);
            get_parameter(object_names[i]+"/marker_config",tmp_obj.marker_config);
            RCLCPP_INFO(get_logger(),"Got initial position of %s : [%f %f %f]",tmp_obj.name.c_str(),tmp_obj.x , tmp_obj.y, tmp_obj.z);
        }
        object_list_prev = object_list;
    }
}

void NatNetNode::get_conn_params()
{
    if (get_parameter("serverIP", serverIP))
        {
            RCLCPP_INFO(get_logger(),"Got server IP : %s", serverIP.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get server IP, using default 192.168.1.114");
            serverIP = "192.168.1.114";
        }

        if (get_parameter("clientIP", clientIP))
        {
            RCLCPP_INFO(get_logger(),"Got client IP : %s", clientIP.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get client IP, using default 192.168.0.101");
            clientIP = "192.168.0.101";
        }

        if (get_parameter("serverType", serverType))
        {
            RCLCPP_INFO(get_logger(),"Got server Type : %s", serverType.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get server type, using default multicast");
            serverType = "multicast";
        }

        if (serverType == "multicast")
        {
            if (get_parameter("multicastAddress", multicastAddress))
                RCLCPP_INFO(get_logger(),"Got server Type : %s", multicastAddress.c_str());
            else
            {
                RCLCPP_WARN(get_logger(),"Failed to get server IP, using default multicast address 239.255.42.99");
                multicastAddress = "239.255.42.99";
            }
        }

        if (get_parameter("serverCommandPort", serverCommandPort))
        {
            RCLCPP_INFO(get_logger(),"Got server Command Port : %i", serverCommandPort);
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get server command port, using default 1510");
            serverCommandPort = 1510;
        }

        if (get_parameter("serverDataPort", serverDataPort))
        {
            RCLCPP_INFO(get_logger(),"Got server Command Port : %i", serverDataPort);
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get server command port, using default 1511");
            serverDataPort = 1511;
        }
}

void NatNetNode::set_conn_params()
{
    if (serverType == "unicast")
        g_ConnectionType = ConnectionType_Unicast;

    // Setting up parameters for the natnet connection
    g_connectParams.connectionType = g_ConnectionType;
    g_connectParams.serverCommandPort = serverCommandPort;
    g_connectParams.serverDataPort = serverDataPort;
    g_connectParams.serverAddress = serverIP.c_str();
    g_connectParams.localAddress = clientIP.c_str();
    g_connectParams.multicastAddress = serverType=="multicast" ? multicastAddress.c_str() : NULL;
}


bool NatNetNode::connect()
{
    g_pClient->Disconnect();
    get_conn_params();
    get_node_params();
    set_conn_params();
    RCLCPP_INFO(get_logger(),"Trying to connect to Optitrack NatNET SDK at %s ...", serverIP.c_str());
    RCLCPP_INFO(get_logger(),"serverType: %s\t serverAddress: %s\t localAddress: %s\t serverDataPort: %i\t serverCommandPort: %i\t multicastAddress: %s",
                            serverType.c_str(),g_connectParams.serverAddress, g_connectParams.localAddress
                            , g_connectParams.serverDataPort,g_connectParams.serverCommandPort,
                            g_connectParams.multicastAddress);
    int retCode = g_pClient->Connect(g_connectParams);
    RCLCPP_INFO(get_logger(),"Connected at %s ...", serverIP.c_str());
    if (retCode != ErrorCode_OK)
    {
        RCLCPP_ERROR(get_logger(),"Unable to connect to server.  Error code: %d. Exiting.", retCode);
        return false;
    }
    else
    {
        // connection succeeded
        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            RCLCPP_ERROR(get_logger(),"Unable to connect to server. Host not present. Exiting.");
            return false;
        }

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            mocap_frame_rate_ = *((float*)pResult);
            RCLCPP_INFO(get_logger(),"Mocap Framerate : %3.2f", mocap_frame_rate_);
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"Error getting frame rate. Setting to default 200.0f");
            mocap_frame_rate_ = 200.0f;
        }

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            //g_analogSamplesPerMocapFrame = *((int*)pResult);
            RCLCPP_INFO(get_logger(),"Analog Samples Per Mocap Frame : %d", *((int*)pResult));
        }
        else
            RCLCPP_WARN(get_logger(),"Did not recieve Analog frame rate.");
    }

    return true;
}


bool NatNetNode::disconnect()
{
    void * response;
    int nBytes;
    if (g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) 
    {
        g_pClient->Disconnect();
        RCLCPP_INFO(get_logger(), "Client Disconnected");
        return true;
    } 
    else 
    {
        RCLCPP_ERROR(get_logger(), "Client Disconnect not successful..");
        return false;
    }
}

void NatNetNode::get_info()
{
    RCLCPP_INFO(get_logger(),"Requesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        RCLCPP_ERROR(get_logger(),"Unable to retrieve Data Descriptions.");
    }
    else
    {
        RCLCPP_INFO(get_logger(),"Received %d Data/Devices Descriptions:", pDataDefs->nDataDescriptions );

        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                RCLCPP_INFO(get_logger(),"RigidBody found : %s", pRB->szName);
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "RigidBody ID : %d", pRB->ID);
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "RigidBody Parent ID : %d", pRB->parentID);
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "Parent Offset : %3.2f,%3.2f,%3.2f", pRB->offsetx, pRB->offsety, pRB->offsetz);
                
                // Creating publisher for the rigid bodies if found any
                std::string body_name(pRB->szName);
                if(pub_rigid_body)
                {
                    ListRigidBodies[pRB->ID] = body_name;
                    RigidbodyPub[pRB->szName] = create_publisher<geometry_msgs::msg::PoseStamped>(body_name+"/pose", rclcpp::QoS(1000));
                    RigidbodyOdomPub[pRB->szName] = create_publisher<nav_msgs::msg::Odometry>(body_name+"/odom", rclcpp::QoS(1000));
                }
                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                {
                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
                        // Creating publisher for the markers of the rigid bodies
                        if(pub_rigid_body_marker)
                            RigidbodyMarkerPub[std::to_string(pRB->ID)+std::to_string(markerIdx+1)] = create_publisher<geometry_msgs::msg::PointStamped>(body_name+"/marker"+std::to_string(markerIdx)+"/pose", rclcpp::QoS(1000));
                        RCLCPP_INFO_EXPRESSION(get_logger(),log_internals,  "\tMarker #%d:", markerIdx );
                        RCLCPP_INFO_EXPRESSION(get_logger(),log_internals,  "\t\tPosition: %.2f, %.2f, %.2f", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
                            RCLCPP_INFO_EXPRESSION(get_logger(),log_internals,  "\t\tRequired active label: %d", markerRequiredLabel );
                        }
                    }
                }
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera
                sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "Camera Name : %s", pCamera->strName);
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "Camera Position (%3.2f, %3.2f, %3.2f)", pCamera->x, pCamera->y, pCamera->z);
                RCLCPP_INFO_EXPRESSION(get_logger(),log_internals, "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            }
            else
            {
                RCLCPP_WARN_EXPRESSION(get_logger(),log_internals, "Unknown data type detected.");
                // Unknown
            }
        }
        if (pub_pointcloud)
        {
            PointcloudPub = create_publisher<sensor_msgs::msg::PointCloud>("pointcloud", rclcpp::QoS(1000));
        }
        if (pub_individual_marker)
        {
            for (int i=0; i<(int) object_list.size(); i++)
            {
            	if (immt=="PoseStamped")
                    {IndividualMarkerPosePub[object_list[i].name] = create_publisher<geometry_msgs::msg::PoseStamped>(object_list[i].name+"/pose", rclcpp::QoS(1000));};
                if (immt=="PointStamped")
                    {IndividualMarkerPointPub[object_list[i].name] = create_publisher<geometry_msgs::msg::PointStamped>(object_list[i].name+"/pose", rclcpp::QoS(1000));};
                
            }
        }
    }
}

std::chrono::nanoseconds NatNetNode::get_latency_info(sFrameOfMocapData * data)
{
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    if (bSystemLatencyAvailable) 
    {
        const double clientLatencySec =
        g_pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
        const double clientLatencyMillisec = clientLatencySec * 1000.0;
        const double transitLatencyMillisec = g_pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

        const double largeLatencyThreshold = 100.0;
        if (clientLatencyMillisec >= largeLatencyThreshold && log_latencies) 
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500,
            "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
            largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
    } 
    else 
    {
        RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
        return std::chrono::nanoseconds::zero();
    }
}

void NatNetNode::process_frame(sFrameOfMocapData* data)
{
    frame_delay = rclcpp::Duration(get_latency_info(data));

    RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "FrameID : %d", data->iFrame);
    RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "Rigid Bodies [Count=%d]", data->nRigidBodies);
    for(unsigned int i=0; i < data->nRigidBodies; i++)
    {
        if(pub_rigid_body)
        {
            process_rigid_body(data->RigidBodies[i]);
        }
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "Rigid Body [ID=%d  Error=%3.2f]", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError);//, bTrackingValid);
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "x\ty\tz\tqx\tqy\tqz\tqw");
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f",
            data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
            data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw);
    }
    for(unsigned int i=0; i < data->nLabeledMarkers; i++) 
    {  
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "Markers [Count=%i]", i);
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "x\ty\tz");
        RCLCPP_INFO_EXPRESSION(get_logger(),log_frames, "%3.2f\t%3.2f\t%3.2f", data->LabeledMarkers[i].x, data->LabeledMarkers[i].y, data->LabeledMarkers[i].z);
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        if(pub_individual_marker && bUnlabeled)
        {   
            process_individual_marker(data->LabeledMarkers[i]);
        }
        if (pub_pointcloud)
        {
            process_pointcloud(data->LabeledMarkers[i]);
        }
        if(pub_rigid_body_marker  && !bUnlabeled)
        {
            process_rigid_body_marker(data->LabeledMarkers[i]);
        }
        if(pub_pointcloud)
        {
            msgPointcloud.header.frame_id= global_frame;
            msgPointcloud.header.stamp = remove_latency ? this->get_clock()->now()-frame_delay : this->get_clock()->now();
            PointcloudPub->publish(msgPointcloud);
        }
        unlabled_count=0;
        msgPointcloud.points.clear();
    }

}

void NatNetNode::process_rigid_body(sRigidBodyData &data)
{
    rclcpp::Time stamp = remove_latency ? this->get_clock()->now() - frame_delay : this->get_clock()->now();
    const std::string& body_name = ListRigidBodies[data.ID];

    Eigen::Vector3d position(data.x, data.y, data.z);
    Eigen::Quaterniond orientation(data.qw, data.qx, data.qy, data.qz);

    geometry_msgs::msg::PoseStamped msgRigidBodyPose;
    msgRigidBodyPose.header.frame_id = global_frame;
    msgRigidBodyPose.header.stamp = stamp;
    msgRigidBodyPose.pose.position.x = data.x;
    msgRigidBodyPose.pose.position.y = data.y;
    msgRigidBodyPose.pose.position.z = data.z;
    msgRigidBodyPose.pose.orientation.x = data.qx;
    msgRigidBodyPose.pose.orientation.y = data.qy;
    msgRigidBodyPose.pose.orientation.z = data.qz;
    msgRigidBodyPose.pose.orientation.w = data.qw;
    RigidbodyPub[body_name]->publish(msgRigidBodyPose);

    // Odometry with constant-velocity Kalman filter
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = global_frame;
    odom_msg.child_frame_id = body_name;
    odom_msg.pose.pose = msgRigidBodyPose.pose;

    if (use_kalman_filter)
    {
        auto kf_it = kalman_filters.find(body_name);
        if (kf_it == kalman_filters.end())
        {
            double dt_expected = 1.0 / static_cast<double>(mocap_frame_rate_);
            mocap_kalman::KalmanFilter::Matrix12d process_noise;
            mocap_kalman::KalmanFilter::Matrix6d measurement_noise;
            process_noise.setZero();
            process_noise.topLeftCorner<6, 6>() = 0.5 * Eigen::Matrix<double, 6, 6>::Identity() * dt_expected * dt_expected * odom_max_accel;
            process_noise.bottomRightCorner<6, 6>() = Eigen::Matrix<double, 6, 6>::Identity() * dt_expected * odom_max_accel;
            process_noise = process_noise * process_noise;  // make covariance (diagonal squared)
            measurement_noise = Eigen::Matrix<double, 6, 6>::Identity() * 1e-3;
            measurement_noise = measurement_noise * measurement_noise;

            auto kf = std::make_shared<mocap_kalman::KalmanFilter>();
            kf->init(process_noise, measurement_noise, static_cast<int>(mocap_frame_rate_));
            kalman_filters[body_name] = kf;
            kf_it = kalman_filters.find(body_name);
            RCLCPP_INFO(get_logger(), "Created Kalman filter for '%s'", body_name.c_str());
        }

        auto& kf = kf_it->second;
        double current_time = stamp.seconds();

        if (!kf->isReady())
        {
            kf->prepareInitialCondition(current_time, orientation, position);
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
        else
        {
            kf->prediction(current_time);
            kf->update(orientation, position);

            odom_msg.pose.pose.position.x = kf->position.x();
            odom_msg.pose.pose.position.y = kf->position.y();
            odom_msg.pose.pose.position.z = kf->position.z();
            odom_msg.pose.pose.orientation.w = kf->attitude.w();
            odom_msg.pose.pose.orientation.x = kf->attitude.x();
            odom_msg.pose.pose.orientation.y = kf->attitude.y();
            odom_msg.pose.pose.orientation.z = kf->attitude.z();

            odom_msg.twist.twist.linear.x = kf->linear_vel.x();
            odom_msg.twist.twist.linear.y = kf->linear_vel.y();
            odom_msg.twist.twist.linear.z = kf->linear_vel.z();
            odom_msg.twist.twist.angular.x = kf->angular_vel.x();
            odom_msg.twist.twist.angular.y = kf->angular_vel.y();
            odom_msg.twist.twist.angular.z = kf->angular_vel.z();

            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_cov(odom_msg.pose.covariance.data());
            pose_cov.topLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(3, 3);
            pose_cov.topRightCorner<3, 3>() = kf->state_cov.block<3, 3>(3, 0);
            pose_cov.bottomLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(0, 3);
            pose_cov.bottomRightCorner<3, 3>() = kf->state_cov.block<3, 3>(0, 0);

            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> vel_cov(odom_msg.twist.covariance.data());
            vel_cov.topLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(9, 9);
            vel_cov.topRightCorner<3, 3>() = kf->state_cov.block<3, 3>(9, 6);
            vel_cov.bottomLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(6, 9);
            vel_cov.bottomRightCorner<3, 3>() = kf->state_cov.block<3, 3>(6, 6);
        }
    }
    else
    {
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
    }

    RigidbodyOdomPub[body_name]->publish(odom_msg);

    // TF frame for visualization
    geometry_msgs::msg::TransformStamped msgTFRigidBodies;
    msgTFRigidBodies.header.stamp = stamp;
    msgTFRigidBodies.header.frame_id = global_frame;
    msgTFRigidBodies.child_frame_id = body_name;
    msgTFRigidBodies.transform.translation.x = data.x;
    msgTFRigidBodies.transform.translation.y = data.y;
    msgTFRigidBodies.transform.translation.z = data.z;
    msgTFRigidBodies.transform.rotation.x = data.qx;
    msgTFRigidBodies.transform.rotation.y = data.qy;
    msgTFRigidBodies.transform.rotation.z = data.qz;
    msgTFRigidBodies.transform.rotation.w = data.qw;
    tfBroadcaster->sendTransform(msgTFRigidBodies);
}

void NatNetNode::process_individual_marker(sMarker &data)
{
    int update = nn_filter(object_list, data, E,  E_x, E_y, E_z, individual_error, error_amp);
    if (update>=0)
    {   unlabled_count+=1;
        object_list[update].detected = true;
        object_list[update].x = data.x;
        object_list[update].y = data.y;
        object_list[update].z = data.z;
    	if (immt=="PoseStamped")
        {    
            geometry_msgs::msg::PoseStamped msgMarkerPose;
            msgMarkerPose.header.frame_id = global_frame;
            msgMarkerPose.header.stamp = remove_latency ? this->get_clock()->now()-frame_delay : this->get_clock()->now();
            msgMarkerPose.pose.position.x = data.x;
            msgMarkerPose.pose.position.y = data.y;
            msgMarkerPose.pose.position.z = data.z;
            msgMarkerPose.pose.orientation.x = 0.0;
            msgMarkerPose.pose.orientation.y = 0.0;
            msgMarkerPose.pose.orientation.z = 0.0;
            msgMarkerPose.pose.orientation.w = 1.0;
            IndividualMarkerPosePub[object_list[update].name]->publish(msgMarkerPose);
        }
        
        if (immt=="PointStamped")
        {    
            geometry_msgs::msg::PointStamped msgMarkerPose;
            msgMarkerPose.header.frame_id = global_frame;
            msgMarkerPose.header.stamp = remove_latency ? this->get_clock()->now()-frame_delay : this->get_clock()->now();
            msgMarkerPose.point.x = data.x;
            msgMarkerPose.point.y = data.y;
            msgMarkerPose.point.z = data.z;
            IndividualMarkerPointPub[object_list[update].name]->publish(msgMarkerPose);
        }

        // creating tf frame to visualize in the rviz
        geometry_msgs::msg::TransformStamped msgTFMarker;
        msgTFMarker.header.stamp = remove_latency ? this->get_clock()->now()-frame_delay : this->get_clock()->now();
        msgTFMarker.header.frame_id = global_frame;
        msgTFMarker.child_frame_id = object_list[update].name;
        msgTFMarker.transform.translation.x = data.x;
        msgTFMarker.transform.translation.y = data.y;
        msgTFMarker.transform.translation.z = data.z;
        msgTFMarker.transform.rotation.x = 0.0;
        msgTFMarker.transform.rotation.y = 0.0;
        msgTFMarker.transform.rotation.z = 0.0;
        msgTFMarker.transform.rotation.w = 1.0;
        tfBroadcaster->sendTransform(msgTFMarker);
    }
}

void NatNetNode::process_pointcloud(sMarker &data)
{
    geometry_msgs::msg::Point32 msgPoint;
    msgPoint.x = data.x;
    msgPoint.y = data.y;
    msgPoint.z = data.z;
    msgPointcloud.points.push_back(msgPoint);
}

void NatNetNode::process_rigid_body_marker(sMarker &data)
{
    int modelID, markerID;
    NatNet_DecodeID( data.ID, &modelID, &markerID );

    geometry_msgs::msg::PointStamped msgMarkerPose;
    msgMarkerPose.header.frame_id = std::to_string(modelID)+std::to_string(markerID);
    msgMarkerPose.header.stamp = remove_latency ? this->get_clock()->now()-frame_delay : this->get_clock()->now();
    msgMarkerPose.point.x = data.x;
    msgMarkerPose.point.y = data.y;
    msgMarkerPose.point.z = data.z;

    RigidbodyMarkerPub[std::to_string(modelID)+std::to_string(markerID)]->publish(msgMarkerPose);
}

void NatNetNode::del_info()
{
    frame_number = 0;
    ListRigidBodies.clear();
    RigidbodyPub.clear();
    RigidbodyOdomPub.clear();
    kalman_filters.clear();
    RigidbodyMarkerPub.clear();
    IndividualMarkerPosePub.clear();
    IndividualMarkerPointPub.clear();
}

CallbackReturnT NatNetNode::on_configure(const rclcpp_lifecycle::State & state)
{
    g_pClient = new NatNetClient();
    if(!this->connect())
        {RCLCPP_ERROR(get_logger(),"Unable to connect"); this->~NatNetNode() ;rclcpp::shutdown();}
    this->get_info();
    RCLCPP_INFO(get_logger(), "Configured!\n");
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT NatNetNode::on_activate(const rclcpp_lifecycle::State & state)
{
    //if(!connect())
    //    return CallbackReturnT::FAILURE;
    if(pub_rigid_body)
    {
        for(auto const& i : RigidbodyPub)
        {
            i.second->on_activate();
        }
        for(auto const& i : RigidbodyOdomPub)
        {
            i.second->on_activate();
        }
    }
    if(pub_rigid_body_marker)
    {
        for(auto const& i: RigidbodyMarkerPub)
        {
            i.second->on_activate();
        }
    }
    if(pub_individual_marker)
    {
        for(auto const& i: IndividualMarkerPosePub)
        {
            i.second->on_activate();
        }
        for (auto const& i: IndividualMarkerPointPub)
        {
            i.second->on_activate();
        }
    }
    if(pub_pointcloud)
        PointcloudPub->on_activate();
    g_pClient->SetFrameReceivedCallback(frame_callback, this);
    RCLCPP_INFO(get_logger(), "Activated!\n");
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT NatNetNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    this->del_info();
    RCLCPP_INFO(get_logger(), "Shutdown!\n");
    //g_pClient->Disconnect();
    this->~NatNetNode();
    return CallbackReturnT::SUCCESS;
}

void NATNET_CALLCONV frame_callback(sFrameOfMocapData * data, void * pUserData)
{
    static_cast<NatNetNode *>(pUserData)->process_frame(data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    //node_options.allow_undeclared_parameters(true);
    auto node = std::make_shared<NatNetNode>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
