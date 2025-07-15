/**
 * @file estimation_manager_node.hpp
 * @brief Definition of the EstimationManager class, a ROS2 lifecycle node
 * for managing and selecting between multiple odometry sources.
 * @author Wagner Dantas Garcia <wagnergarcia@eng.ci.ufpb.br>
 * @date June 25, 2025
 */

#ifndef LASER_ESTIMATION_MANAGER__ESTIMATION_MANAGER_NODE_HPP_
#define LASER_ESTIMATION_MANAGER__ESTIMATION_MANAGER_NODE_HPP_

/* includes //{ */

#include <memory>
#include <mutex>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_msgs/srv/set_string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/*//}*/

// Alias to simplify writing the return type of lifecycle callbacks.
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_estimation_manager
{

    /**
     * @struct GenericSourceConfig
     * @brief A template struct to store the configuration and state
     * of a generic data source (subscriber).
     * @tparam MsgT The ROS2 message type that this subscriber will receive.
     */
    template <typename MsgT>
    struct GenericSourceConfig
    {
        std::string name;                                            ///< A unique, human-readable identifier for the source (e.g., "odom_visual").
        std::string frame_id;                                        ///< The TF frame_id associated with this source's message header.
        std::string topic;                                           ///< The full ROS2 topic that the source is subscribed to.
        typename rclcpp::Subscription<MsgT>::SharedPtr subscription; ///< The ROS2 subscription object.
        typename MsgT::SharedPtr last_message;                       ///< A pointer to the last message received from this source.
        rclcpp::Time last_message_time{0, 0};                        ///< The timestamp of when the last message was received.
    };

    /// @brief An alias for a specific configuration for Odometry messages.
    using OdometryConfig = GenericSourceConfig<nav_msgs::msg::Odometry>;

    /// @brief A variant to store a pointer to any of the supported source configuration types.
    /// @note To add support for a new message type, it must be added here.
    using SourceVariant = std::variant<
        std::shared_ptr<OdometryConfig>>;

    /**
     * @class EstimationManager
     * @brief A lifecycle node that manages multiple odometry sources,
     * allows selecting an active source via a service, and publishes the data
     * from the selected source.
     */
    class EstimationManager : public rclcpp_lifecycle::LifecycleNode
    {
        /* public: //{ */
    public:
        /**
         * @brief Constructor for the EstimationManager class.
         * @param options Configuration options for the ROS2 node.
         */
        explicit EstimationManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         */
        ~EstimationManager() override;

        /*//}*/

        /* private: //{ */
    private:
        /* Lifecycle Callbacks //{ */

        /**
         * @brief Callback called on the transition to the 'configuring' state.
         * Allocates all necessary resources (parameters, pub/sub, services).
         * @param state The previous state of the node.
         * @return CallbackReturn::SUCCESS if configuration is successful, otherwise FAILURE.
         */
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief Callback called on the transition to the 'activating' state.
         * Activates publishers and other functionalities that should run in the active state.
         * @param state The previous state of the node.
         * @return CallbackReturn::SUCCESS if activation is successful.
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief Callback called on the transition to the 'deactivating' state.
         * Deactivates publishers and functionalities, but keeps resources allocated.
         * @param state The previous state of the node.
         * @return CallbackReturn::SUCCESS if deactivation is successful.
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief Callback called on the transition to the 'cleaning up' state.
         * Releases all resources allocated during 'on_configure'.
         * @param state The previous state of the node.
         * @return CallbackReturn::SUCCESS if cleanup is successful.
         */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief Callback called on the final transition to 'shutdown'.
         * @param state The previous state of the node.
         * @return CallbackReturn::SUCCESS.
         */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        /*//}*/

        /* Configuration Methods //{ */

        /**
         * @brief Loads all necessary parameters from the ROS parameter server.
         */
        void getParameters();

        /**
         * @brief Configures all publishers and subscribers for the node.
         */
        void configPubSub();

        /**
         * @brief Configures all timers for the node.
         */
        void configTimers();

        /**
         * @brief Configures all service servers for the node.
         */
        void configServices();

        /*//}*/

        /* Publishers //{ */

        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_; ///< Lifecycle publisher for the active odometry source's data.

        /*//}*/

        /* Subscribers //{ */

        /**
         * @brief Main callback for all odometry sources.
         * @param msg The incoming odometry message.
         * @param config The specific configuration object for the source that triggered this callback.
         */
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, std::shared_ptr<OdometryConfig> config);

        /**
         * @brief Processes the odometry message only if it comes from the active source.
         * Responsible for publishing the odometry and its TF transform.
         * @param msg The incoming odometry message.
         * @param config The configuration of the source that triggered the message.
         */
        void processActiveOdometry(const nav_msgs::msg::Odometry::SharedPtr msg, std::shared_ptr<OdometryConfig> config);

        /*//}*/

        /* Timers //{ */

        /**
         * @brief Timer callback function. Currently empty, can be used for
         * periodic publishing or other tasks.
         */
        void tmrOdometryPublisher();

        rclcpp::TimerBase::SharedPtr tmr_odometry_publisher_; ///< Pointer to the timer object.
        double _rate_odometry_publisher_{100.0};              ///< The frequency (in Hz) for the publishing timer.

        /*//}*/

        /* Services //{ */

        /**
         * @brief Callback for the service that sets the active odometry source.
         * @param request The service request, containing the name of the desired source.
         * @param response The service response, indicating success or failure.
         */
        void setOdometryCallback(
            const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
            std::shared_ptr<laser_msgs::srv::SetString::Response> response);

        rclcpp::Service<laser_msgs::srv::SetString>::SharedPtr set_odometry_service_; ///< Server for the odometry selection service.

        /*//}*/

        /* TF Handling //{ */

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                              ///< Buffer to store TF transformations.
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                 ///< Listener to receive TF updates.
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; ///< Broadcaster for publishing static transforms.
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;              ///< Broadcaster for publishing dynamic transforms.

        /*//}*/

        /* General Variables //{ */

        std::mutex mtx_;                                    ///< Mutex to protect shared data access across callbacks.
        bool is_active_{false};                             ///< Flag to track the node's 'active' lifecycle state.
        std::vector<SourceVariant> generic_source_configs_; ///< Vector that stores all configured data source configurations.
        std::string _uav_name_;                             ///< The UAV name, used for building topics and frames.
        std::string current_active_odometry_name_;          ///< The name of the currently active odometry source.

        /*//}*/

        /*//}*/

    }; // class EstimationManager

} // namespace laser_estimation_manager

#endif // LASER_ESTIMATION_MANAGER__ESTIMATION_MANAGER_NODE_HPP_