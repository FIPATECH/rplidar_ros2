/*
 *  RPLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rplidar_node.hpp>

namespace rplidar_ros2
{

    /*
     * Constructor for the rplidar_node.
     *
     * This constructor initializes the node parameters, creates and connects the RPLIDAR driver,
     * checks device information and health, sets up the ROS 2 publisher and services, and starts the
     * scanning process based on the auto-standby parameter.
     */
    rplidar_node::rplidar_node(const rclcpp::NodeOptions &options)
        : Node("rplidar_node", options)
    {
        /* Set parameters from the ROS 2 parameter server */
        channel_type_ = this->declare_parameter("channel_type", "serial");
        tcp_ip_ = this->declare_parameter("tcp_ip", "192.168.0.7");
        tcp_port_ = this->declare_parameter("tcp_port", 20108);
        serial_port_ = this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        serial_baudrate_ = this->declare_parameter("serial_baudrate", 115200);
        frame_id_ = this->declare_parameter("frame_id", std::string("laser_frame"));
        inverted_ = this->declare_parameter("inverted", false);
        angle_compensate_ = this->declare_parameter("angle_compensate", false);
        flip_x_axis_ = this->declare_parameter("flip_x_axis", false);
        scan_mode_ = this->declare_parameter("scan_mode", std::string());
        topic_name_ = this->declare_parameter("topic_name", std::string("scan"));

        /* Auto-standby parameter: When enabled, scanning starts only if there are subscribers */
        auto_standby_ = this->declare_parameter("auto_standby", false);
        is_scanning_ = false;

        /* Log the SDK version */
        RCLCPP_INFO(this->get_logger(), "RPLIDAR running on ROS2 package rplidar_ros2. SDK Version: '%s'", RPLIDAR_SDK_VERSION);

        /* Create the RPLIDAR driver based on the selected channel type */
        if (channel_type_ == "tcp")
        {
            m_drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
        }
        else
        {
            m_drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
        }
        if (nullptr == m_drv)
        {
            /* Exit early if the driver could not be constructed */
            RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
            return;
        }

        /* Establish connection with the device */
        u_result op_result;
        if (channel_type_ == "tcp")
        {
            op_result = m_drv->connect(tcp_ip_.c_str(), static_cast<_u32>(tcp_port_));
            if (IS_FAIL(op_result))
            {
                /* Log error if connection over TCP fails */
                RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to TCP host %s:%d", tcp_ip_.c_str(), tcp_port_);
                RPlidarDriver::DisposeDriver(m_drv);
                m_drv = nullptr;
                return;
            }
        }
        else
        {
            op_result = m_drv->connect(serial_port_.c_str(), static_cast<_u32>(serial_baudrate_));
            if (IS_FAIL(op_result))
            {
                /* Log error if connection via serial port fails */
                RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port %s", serial_port_.c_str());
                RPlidarDriver::DisposeDriver(m_drv);
                m_drv = nullptr;
                return;
            }
        }

        /* Retrieve the device info and check its health */
        if (!getRPLIDARDeviceInfo())
        {
            RPlidarDriver::DisposeDriver(m_drv);
            m_drv = nullptr;
            return;
        }
        if (!checkRPLIDARHealth())
        {
            RPlidarDriver::DisposeDriver(m_drv);
            m_drv = nullptr;
            return;
        }

        /* Setup ROS 2 components (publisher and services) */
        /* Create the publisher for the LaserScan messages on the configured topic */
        m_publisher = this->create_publisher<LaserScan>(topic_name_, 10);

        /* Create the service to stop the motor */
        m_stop_motor_service = this->create_service<std_srvs::srv::Empty>(
            "stop_motor",
            std::bind(&rplidar_node::stop_motor, this, std::placeholders::_1, std::placeholders::_2));

        /* Create the service to start the motor */
        m_start_motor_service = this->create_service<std_srvs::srv::Empty>(
            "start_motor",
            std::bind(&rplidar_node::start_motor, this, std::placeholders::_1, std::placeholders::_2));

        /* If auto-standby is disabled, start scanning immediately */
        if (!auto_standby_)
        {
            if (!set_scan_mode())
            {
                /* Log error and clean up if setting scan mode fails */
                RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode");
                RPlidarDriver::DisposeDriver(m_drv);
                m_drv = nullptr;
                return;
            }
            /* Start the motor and initiate scanning */
            m_drv->startMotor();
            m_drv->startScan(0, 1);
            is_scanning_ = true;
        }

        /* Create a wall timer to call the publish_loop() function periodically */
        m_timer = this->create_wall_timer(1ms, std::bind(&rplidar_node::publish_loop, this));
    }

    /*
     * Destructor for the rplidar_node.
     *
     * This destructor stops the scanning, stops the motor, and disposes of the driver.
     */
    rplidar_node::~rplidar_node()
    {
        if (m_drv)
        {
            m_drv->stop();
            m_drv->stopMotor();
            RPlidarDriver::DisposeDriver(m_drv);
            m_drv = nullptr;
        }
    }

    /*
     * publish_scan:
     *   Converts the raw scan data into a LaserScan message and publishes it.
     *
     * Parameters:
     *   - scan_time: Duration of the scan
     *   - nodes: Unique pointer to the array of raw measurement nodes
     *   - node_count: Number of measurement nodes in the scan
     */
    void rplidar_node::publish_scan(const double scan_time, ResponseNodeArray nodes, size_t node_count)
    {
        LaserScan scan_msg;
        /* Set header timestamp and frame ID */
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = frame_id_;

        /* Determine if the scan angles are reversed */
        bool reversed = (angle_max > angle_min);
        if (reversed)
        {
            /* Adjust angles if scan data is reversed */
            scan_msg.angle_min = M_PI - angle_max;
            scan_msg.angle_max = M_PI - angle_min;
        }
        else
        {
            scan_msg.angle_min = M_PI - angle_min;
            scan_msg.angle_max = M_PI - angle_max;
        }
        /* Calculate angle increment based on the number of nodes */
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);
        scan_msg.scan_time = scan_time;
        scan_msg.time_increment = scan_time / (double)(node_count - 1);
        scan_msg.range_min = min_distance;
        scan_msg.range_max = max_distance;

        /* Resize the vectors to hold range and intensity values */
        scan_msg.ranges.resize(node_count);
        scan_msg.intensities.resize(node_count);

        /* Determine if the data order should be reversed based on the inversion flag */
        bool reverse_data = (!inverted_ && (angle_max > angle_min)) || (inverted_ && !(angle_max > angle_min));
        size_t scan_midpoint = node_count / 2;
        for (size_t i = 0; i < node_count; ++i)
        {
            /* Convert raw distance value from mm to meters */
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            size_t apply_index = i;
            if (reverse_data)
            {
                /* Reverse the index order if needed */
                apply_index = node_count - 1 - i;
            }
            if (flip_x_axis_)
            {
                /* Flip the index for X axis inversion */
                if (apply_index >= scan_midpoint)
                    apply_index = apply_index - scan_midpoint;
                else
                    apply_index = apply_index + scan_midpoint;
            }
            /* If read value is zero, set the range to infinity */
            if (read_value == 0.0)
                scan_msg.ranges[apply_index] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[apply_index] = read_value;
            /* Extract intensity information from the quality field */
            scan_msg.intensities[apply_index] = (float)(nodes[i].quality >> 2);
        }

        /* Publish the LaserScan message */
        m_publisher->publish(scan_msg);
    }

    /*
     * publish_loop:
     *   Called periodically by the timer. This method grabs the scan data, processes it,
     *   compensates angles if required, and then publishes the scan.
     */
    void rplidar_node::publish_loop()
    {
        /* Auto-standby logic: start scanning only if there are subscribers */
        if (auto_standby_)
        {
            if (m_publisher->get_subscription_count() > 0 && !is_scanning_)
            {
                /* Attempt to set scan mode before starting scan */
                if (!set_scan_mode())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode in auto-standby mode");
                    return;
                }
                m_drv->startMotor();
                m_drv->startScan(0, 1);
                is_scanning_ = true;
            }
            else if (m_publisher->get_subscription_count() == 0 && is_scanning_)
            {
                /* Stop scanning when there are no subscribers */
                m_drv->stop();
                m_drv->stopMotor();
                is_scanning_ = false;
                return;
            }
        }
        /* If scanning is not active, do not proceed with data acquisition */
        if (!is_scanning_)
        {
            return;
        }

        /* Record start time of the scan for duration calculation */
        rclcpp::Time start_scan_time = this->now();
        size_t count = 360 * 8; /* Number of expected scan points (adjust according to expected density) */
        auto nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(count);

        /* Grab the high-quality scan data from the lidar */
        u_result op_result = m_drv->grabScanDataHq(nodes.get(), count);
        rclcpp::Time end_scan_time = this->now();
        /* Calculate the scan duration in seconds */
        double scan_duration = (end_scan_time - start_scan_time).nanoseconds() * 1e-9;

        if (op_result != RESULT_OK)
        {
            /* If grabbing scan data fails, exit the loop */
            return;
        }

        /* Reorder the scan data in ascending order based on the angle */
        op_result = m_drv->ascendScanData(nodes.get(), count);
        /* Update the minimum and maximum angles from the first and last valid points */
        angle_min = deg_2_rad(getAngle(nodes[0]));
        angle_max = deg_2_rad(getAngle(nodes[count - 1]));

        if (op_result == RESULT_OK)
        {
            if (angle_compensate_)
            {
                /* If angle compensation is enabled, create a new array with a fixed number of points */
                const int angle_compensate_nodes_count = 360 * m_angle_compensate_multiple;
                int angle_compensate_offset = 0;
                auto angle_compensate_nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(angle_compensate_nodes_count);
                /* Zero out the compensation nodes array */
                memset(angle_compensate_nodes.get(), 0, angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));

                size_t i = 0, j = 0;
                /* Iterate through each node to fill in the compensated data */
                for (; i < count; i++)
                {
                    if (nodes[i].dist_mm_q2 != 0)
                    {
                        float angle = getAngle(nodes[i]);
                        int angle_value = static_cast<int>(angle * m_angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0)
                            angle_compensate_offset = angle_value;
                        /* Duplicate the measurement data across multiple indices to achieve compensation */
                        for (j = 0; j < m_angle_compensate_multiple; j++)
                        {
                            int index = angle_value - angle_compensate_offset + j;
                            if (index >= angle_compensate_nodes_count)
                                index = angle_compensate_nodes_count - 1;
                            angle_compensate_nodes[index] = nodes[i];
                        }
                    }
                }
                /* Publish the compensated scan data */
                publish_scan(scan_duration, std::move(angle_compensate_nodes), angle_compensate_nodes_count);
            }
            else
            {
                /* If no angle compensation is needed, find the first and last valid measurement nodes */
                int start_node = 0, end_node = 0, i = 0;
                while (i < static_cast<int>(count) && nodes[i].dist_mm_q2 == 0)
                {
                    i++;
                }
                start_node = i;
                i = count - 1;
                while (i >= 0 && nodes[i].dist_mm_q2 == 0)
                {
                    i--;
                }
                end_node = i;
                if (start_node > end_node)
                    return;
                /* Update angles based on the valid range of nodes */
                angle_min = deg_2_rad(getAngle(nodes[start_node]));
                angle_max = deg_2_rad(getAngle(nodes[end_node]));
                size_t valid_count = end_node - start_node + 1;
                auto valid_nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(valid_count);
                for (size_t x = 0; x < valid_count; x++)
                {
                    valid_nodes[x] = nodes[start_node + x];
                }
                /* Publish the scan data using only the valid nodes */
                publish_scan(scan_duration, std::move(valid_nodes), valid_count);
            }
        }
        else if (op_result == RESULT_OPERATION_FAIL)
        {
            /* If the operation fails, fallback to default angles and publish the data as is */
            angle_min = deg_2_rad(0.0);
            angle_max = deg_2_rad(359.0);
            publish_scan(scan_duration, std::move(nodes), count);
        }
    }

    /*
     * getRPLIDARDeviceInfo:
     *   Retrieves and logs the device information (serial number, firmware version, hardware revision).
     *
     * Returns:
     *   true if successful; false otherwise.
     */
    bool rplidar_node::getRPLIDARDeviceInfo() const
    {
        rplidar_response_device_info_t devinfo;
        u_result op_result = m_drv->getDeviceInfo(devinfo);
        if (IS_FAIL(op_result))
        {
            if (op_result == RESULT_OPERATION_TIMEOUT)
            {
                RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: %x", op_result);
            }
            return false;
        }
        /* Build and log the device serial number */
        std::string serial_no = "RPLIDAR S/N: ";
        for (int pos = 0; pos < 16; ++pos)
        {
            char buf[3];
            snprintf(buf, sizeof(buf), "%02X", devinfo.serialnum[pos]);
            serial_no += buf;
        }
        RCLCPP_INFO(this->get_logger(), "%s", serial_no.c_str());
        /* Log firmware and hardware versions */
        RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", (int)devinfo.hardware_version);
        return true;
    }

    /*
     * checkRPLIDARHealth:
     *   Checks the health status of the RPLIDAR device.
     *
     * Returns:
     *   true if the device is healthy; false otherwise.
     */
    bool rplidar_node::checkRPLIDARHealth() const
    {
        rplidar_response_device_health_t healthinfo;
        u_result op_result = m_drv->getHealth(healthinfo);
        if (IS_OK(op_result))
        {
            RCLCPP_INFO(this->get_logger(), "RPLidar health status : %d", healthinfo.status);
            if (healthinfo.status == RPLIDAR_STATUS_ERROR)
            {
                /* Log error if the device reports an internal error */
                RCLCPP_ERROR(this->get_logger(), "Error, rplidar internal error detected. Please reboot the device to retry");
                return false;
            }
            return true;
        }
        /* Log error if unable to retrieve health code */
        RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }

    /*
     * stop_motor:
     *   Stops the motor of the RPLIDAR device.
     *   If auto-standby mode is enabled, the stop request is ignored.
     */
    void rplidar_node::stop_motor(const EmptyRequest req, EmptyResponse res)
    {
        (void)req;
        (void)res;
        if (auto_standby_)
        {
            /* Log that stop_motor is ignored in auto-standby mode */
            RCLCPP_INFO(this->get_logger(), "Ignoring stop_motor request because auto-standby mode is enabled.");
            return;
        }
        if (m_drv)
        {
            /* Stop scanning and the motor */
            m_drv->stop();
            m_drv->stopMotor();
            is_scanning_ = false;
            RCLCPP_INFO(this->get_logger(), "Motor stopped.");
        }
    }

    /*
     * start_motor:
     *   Starts the motor and initiates scanning.
     *   If auto-standby mode is enabled, the start request is ignored.
     */
    void rplidar_node::start_motor(const EmptyRequest req, EmptyResponse res)
    {
        (void)req;
        (void)res;
        if (auto_standby_)
        {
            /* Log that start_motor is ignored in auto-standby mode */
            RCLCPP_INFO(this->get_logger(), "Ignoring start_motor request because auto-standby mode is enabled.");
            return;
        }
        if (m_drv)
        {
            if (!set_scan_mode())
            {
                /* Log error if setting scan mode fails during start */
                RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode during start_motor.");
                return;
            }
            /* Start motor and scanning */
            m_drv->startMotor();
            m_drv->startScan(0, 1);
            is_scanning_ = true;
            RCLCPP_INFO(this->get_logger(), "Motor started.");
        }
    }

    /*
     * set_scan_mode:
     *   Configures the scanning mode of the RPLIDAR device.
     *
     * Returns:
     *   true if the scan mode was set successfully; false otherwise.
     */
    bool rplidar_node::set_scan_mode()
    {
        u_result op_result;
        RplidarScanMode current_scan_mode;
        if (scan_mode_.empty())
        {
            /* If no specific scan mode is requested, start a typical scan */
            op_result = m_drv->startScan(false, true, 0, &current_scan_mode);
        }
        else
        {
            /* Retrieve and iterate over all supported scan modes */
            std::vector<RplidarScanMode> allSupportedScanModes;
            op_result = m_drv->getAllSupportedScanModes(allSupportedScanModes);
            if (IS_OK(op_result))
            {
                auto iter = std::find_if(allSupportedScanModes.begin(), allSupportedScanModes.end(),
                                         [this](const RplidarScanMode &mode)
                                         {
                                             return std::string(mode.scan_mode) == scan_mode_;
                                         });
                if (iter == allSupportedScanModes.end())
                {
                    /* Log error if the requested scan mode is not supported */
                    RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode_.c_str());
                    for (const auto &mode : allSupportedScanModes)
                    {
                        RCLCPP_ERROR(this->get_logger(), "%s: max_distance: %.1f m, Point number: %.1fK",
                                     mode.scan_mode, mode.max_distance, (1000 / mode.us_per_sample));
                    }
                    op_result = RESULT_OPERATION_FAIL;
                }
                else
                {
                    /* Start an express scan with the selected mode */
                    op_result = m_drv->startScanExpress(false, iter->id, 0, &current_scan_mode);
                }
            }
        }
        if (IS_OK(op_result))
        {
            /* Calculate the angle compensation multiplier based on the scan sample rate */
            m_angle_compensate_multiple = static_cast<int>(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
            if (m_angle_compensate_multiple < 1)
                m_angle_compensate_multiple = 1;
            /* Set the maximum detectable distance from the scan mode info */
            max_distance = current_scan_mode.max_distance;
            /* Log the selected scan mode parameters */
            RCLCPP_INFO(this->get_logger(),
                        "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK, angle_compensate: %d, flip_x_axis: %d",
                        current_scan_mode.scan_mode, current_scan_mode.max_distance,
                        (1000 / current_scan_mode.us_per_sample), m_angle_compensate_multiple, flip_x_axis_);
            return true;
        }
        else
        {
            /* Log error if starting scan fails */
            RCLCPP_ERROR(this->get_logger(), "Cannot start scan: '%08x'", op_result);
            return false;
        }
    }

} // namespace rplidar_ros2

/* Register the node as a component so that it can be dynamically loaded */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rplidar_ros2::rplidar_node)
