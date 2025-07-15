/**
 * @file estimation_manager.cpp
 * @brief Implementation of the EstimationManager class.
 * @author Wagner Dantas Garcia <wagnergarcia@eng.ci.ufpb.br>
 * @date June 25, 2025
 */

#include <laser_estimation_manager/estimation_manager.hpp>
#include <chrono> // Required for std::chrono::duration

namespace laser_estimation_manager
{

    /* EstimationManager() //{ */
    EstimationManager::EstimationManager(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("estimation_manager", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating the EstimationManager...");

        // Declaration of all parameters that the node will use.
        // This allows them to be configured via a YAML file.
        declare_parameter("frequency", rclcpp::ParameterValue(100.0));
        declare_parameter("uav_name", rclcpp::ParameterValue("uav"));
        declare_parameter("initial_odometry_source", rclcpp::ParameterValue("px4_api_odom"));

        // Parameters for data sources, using parallel vectors.
        declare_parameter("odometry_source_names", rclcpp::ParameterValue(std::vector<std::string>{}));
        declare_parameter("odometry_source_types", rclcpp::ParameterValue(std::vector<std::string>{}));
        declare_parameter("odometry_source_frames", rclcpp::ParameterValue(std::vector<std::string>{}));
        declare_parameter("odometry_source_topics", rclcpp::ParameterValue(std::vector<std::string>{}));

        declare_parameter("data_timeout_ms", rclcpp::ParameterValue(200));
        declare_parameter("health_check_rate", rclcpp::ParameterValue(1.0));

        // Initialization of TF components.
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(), "EstimationManager initialized and ready to be configured.");
    }
    /*//}*/

    /* ~EstimationManager() //{ */
    EstimationManager::~EstimationManager()
    {
        RCLCPP_INFO(get_logger(), "Destroying the EstimationManager.");
    }
    /*//}*/

    // --------------------------------------------------------------
    // |                     Lifecycle Callbacks                    |
    // --------------------------------------------------------------

    /* on_configure() //{ */
    CallbackReturn EstimationManager::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring the EstimationManager...");

        // Orchestrates the node's complete configuration by calling helper functions.
        getParameters();
        configPubSub();
        configTimers();
        configServices();

        RCLCPP_INFO(get_logger(), "Configuration completed successfully.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_activate() //{ */
    CallbackReturn EstimationManager::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating the EstimationManager...");

        // Activates the main publisher so it can start publishing.
        pub_odom_->on_activate();

        // Sets the control flag to allow processing in callbacks.
        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = true;
        }

        RCLCPP_INFO(get_logger(), "Activation completed.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_deactivate() //{ */
    CallbackReturn EstimationManager::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating the EstimationManager...");

        // Deactivates the publisher to stop publications.
        pub_odom_->on_deactivate();

        // Disables the control flag to stop processing in callbacks.
        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = false;
        }

        RCLCPP_INFO(get_logger(), "Deactivation completed.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_cleanup() //{ */
    CallbackReturn EstimationManager::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up EstimationManager resources...");

        // Release all fixed resources, such as publishers and services.
        pub_odom_.reset();
        set_odometry_service_.reset();

        RCLCPP_INFO(get_logger(), "Resetting all data source subscribers...");
        for (auto &source_variant : generic_source_configs_)
        {
            // For each source, visit the variant to access the subscriber and reset it.
            std::visit([this](auto &&arg)
                       {
                           if (arg && arg->subscription)
                           {
                               RCLCPP_INFO(get_logger(), "Cleaning up subscriber for source '%s'", arg->name.c_str());
                               arg->subscription.reset();
                           } },
                       source_variant);
        }

        // Clears the configuration vector, freeing the memory of the config objects.
        RCLCPP_INFO(get_logger(), "Clearing configuration list...");
        generic_source_configs_.clear();

        RCLCPP_INFO(get_logger(), "Cleanup complete.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_shutdown() //{ */
    CallbackReturn EstimationManager::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down EstimationManager...");
        // Final shutdown logic can be added here if necessary.
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    // --------------------------------------------------------------
    // |                   Configuration Methods                    |
    // --------------------------------------------------------------

    /* getParameters() //{ */
    void EstimationManager::getParameters()
    {
        RCLCPP_INFO(get_logger(), "Loading parameters...");

        get_parameter("frequency", _rate_odometry_publisher_);
        get_parameter("initial_odometry_source", current_active_odometry_name_);
        get_parameter("uav_name", _uav_name_);

        // Load the string vectors that define the odometry sources.
        std::vector<std::string> odometry_sources_names;
        get_parameter("odometry_source_names", odometry_sources_names);
        std::vector<std::string> odometry_sources_types;
        get_parameter("odometry_source_types", odometry_sources_types);
        std::vector<std::string> odometry_sources_frames;
        get_parameter("odometry_source_frames", odometry_sources_frames);
        std::vector<std::string> odometry_sources_topics;
        get_parameter("odometry_source_topics", odometry_sources_topics);

        // Validate that all parameter vectors have the same size.
        if (odometry_sources_types.size() != odometry_sources_names.size() ||
            odometry_sources_frames.size() != odometry_sources_names.size() ||
            odometry_sources_topics.size() != odometry_sources_names.size())
        {
            RCLCPP_ERROR(get_logger(), "Mismatch in the size of odometry source parameter vectors. Check the configuration file.");
            return;
        }

        generic_source_configs_.clear(); // Ensure the configuration list is empty before filling it.

        // Iterate over the sources defined in the parameters to create the subscribers.
        for (size_t i = 0; i < odometry_sources_names.size(); ++i)
        {
            const auto &name = odometry_sources_names[i];
            const auto &type = odometry_sources_types[i];
            const auto &frame = odometry_sources_frames[i];
            const auto &topic_base = odometry_sources_topics[i];
            const std::string full_topic = "/" + _uav_name_ + "/" + topic_base;

            // "Factory" pattern: decides which subscriber type to create based on the 'type' string.
            if (type == "nav_msgs/Odometry" || type == "nav_msgs::msg::Odometry")
            {
                auto config = std::make_shared<OdometryConfig>();
                config->name = name;
                config->frame_id = frame;
                config->topic = full_topic;

                // Create a lambda for the callback, capturing the specific configuration for this source.
                auto callback_lambda = [this, config](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    // Delegate the call to the main callback, passing the message and context.
                    this->odometryCallback(msg, config);
                };

                config->subscription = create_subscription<nav_msgs::msg::Odometry>(full_topic, 10, callback_lambda);

                generic_source_configs_.push_back(config);
                RCLCPP_INFO(get_logger(), "Loaded odometry source: Name='%s', Type='%s', Topic='%s'", name.c_str(), type.c_str(), full_topic.c_str());
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unsupported source type '%s' for source '%s'. Skipping.", type.c_str(), name.c_str());
            }
        }

        if (generic_source_configs_.empty())
        {
            RCLCPP_ERROR(get_logger(), "No valid odometry sources were configured. Check your YAML file.");
        }
    }
    /*//}*/

    /* configPubSub() //{ */
    void EstimationManager::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Initializing publishers...");
        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry_out", 10);
    }
    /*//}*/

    /* configTimers() //{ */
    void EstimationManager::configTimers()
    {
        RCLCPP_INFO(get_logger(), "Initializing timers...");
        tmr_odometry_publisher_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / _rate_odometry_publisher_),
            std::bind(&EstimationManager::tmrOdometryPublisher, this));
    }
    /*//}*/

    /* configServices() //{ */
    void EstimationManager::configServices()
    {
        RCLCPP_INFO(get_logger(), "Initializing services...");
        set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
            "~/set_odometry", // Using '~/` for a node-private service topic
            std::bind(&EstimationManager::setOdometryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
    }
    /*//}*/

    // --------------------------------------------------------------
    // |                       Callback Methods                     |
    // --------------------------------------------------------------

    /* odometryCallback() //{ */
    void EstimationManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, std::shared_ptr<OdometryConfig> config)
    {
        // "Guard Clauses" to exit early if conditions are not met.
        if (!is_active_)
        {
            return;
        }
        if (!msg)
        {
            RCLCPP_WARN(get_logger(), "Received null odometry message on topic %s.", config->topic.c_str());
            return;
        }
        if (msg->header.frame_id.empty() || msg->child_frame_id.empty())
        {
            RCLCPP_WARN(get_logger(), "Odometry from source '%s' has an empty frame_id or child_frame_id.", config->name.c_str());
            return;
        }

        // Store the latest message and its reception time in the source's configuration.
        config->last_message = msg;
        config->last_message_time = this->get_clock()->now();

        // If the message came from the currently active source, process it.
        if (config->name == current_active_odometry_name_)
        {
            processActiveOdometry(msg, config);
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Received odometry from inactive source: %s", config->name.c_str());
        }
    }
    /*//}*/

    /* setOdometryCallback() //{ */
    void EstimationManager::setOdometryCallback(
        const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
        std::shared_ptr<laser_msgs::srv::SetString::Response> response)
    {
        RCLCPP_INFO(get_logger(), "SetOdometry service called with request: %s", request->data.c_str());

        std::lock_guard<std::mutex> lock(mtx_);

        bool source_found = false;
        // Search the generic configuration list for the source with the requested name.
        for (const auto &source_variant : generic_source_configs_)
        {
            std::visit([&](auto &&arg)
                       {
                           if (arg->name == request->data)
                           {
                               current_active_odometry_name_ = arg->name;
                               source_found = true;
                           } },
                       source_variant);
            if (source_found)
            {
                break;
            } // Optimization: exit the loop as soon as the source is found.
        }

        // Build the service response based on the search result.
        if (source_found)
        {
            response->success = true;
            response->message = "Odometry source changed to: " + current_active_odometry_name_;
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        }
        else
        {
            response->success = false;
            response->message = "Odometry source not found: " + request->data;
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        }
    }
    /*//}*/

    /* tmrOdometryPublisher() //{ */
    void EstimationManager::tmrOdometryPublisher()
    {
        // This function is called periodically by the timer.
        // Currently empty, but could be used to publish state
        // information or perform periodic checks.
    }
    /*//}*/

    // --------------------------------------------------------------
    // |                       Helper Methods                       |
    // --------------------------------------------------------------

    /* processActiveOdometry() //{ */
    void EstimationManager::processActiveOdometry(const nav_msgs::msg::Odometry::SharedPtr msg, std::shared_ptr<OdometryConfig> config)
    {
        RCLCPP_DEBUG(get_logger(), "Processing odometry from active source: %s", config->name.c_str());

        // Step 1: Forward the odometry message from the active source to the output topic.
        pub_odom_->publish(*msg);

        // Step 2: Assemble and broadcast the corresponding TF transform.
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = config->frame_id; // Use the frame_id from our configuration.
        transform.child_frame_id = msg->child_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        broadcaster_->sendTransform(transform);
    }
    /*//}*/

} // namespace laser_estimation_manager