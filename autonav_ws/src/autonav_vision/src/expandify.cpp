#include "scr/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

struct ExpandifyConfig
{
    float vertical_fov;
    float horizontal_fov;
    float map_res;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ExpandifyConfig, vertical_fov, horizontal_fov, map_res)
};

struct Circle
{
    int x;
    int y;
    double radius;
};

class ExpandifyNode : public SCR::Node
{
public:
    ExpandifyNode() : SCR::Node("autonav_vision_expandifier") {}
    ~ExpandifyNode() {}

    void init() override
    {
        map = nav_msgs::msg::MapMetaData();
        map.width = 100;
        map.height = 100;
        map.resolution = 0.1;
        map.origin = geometry_msgs::msg::Pose();
        map.origin.position.x = -10.0;
        map.origin.position.y = -10.0;

        auto tempRange = maxRange * noGoPercent;
        maxRange = (int)(maxRange / (config.horizontal_fov / config.map_res));
        noGoRange = (int)(tempRange / (config.horizontal_fov / config.map_res));

        circles.push_back(Circle{0, 0, 0});
        for (int x = -maxRange; x <= maxRange; x++)
        {
            for (int y = -maxRange; y <= maxRange; y++)
            {
                if (maxRange * noGoPercent <= sqrt(x * x + y * y) && sqrt(x * x + y * y) < maxRange)
                {
                    circles.push_back(Circle{x, y, sqrt(x * x + y * y)});
                }
            }
        }

        raw_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 20, std::bind(&ExpandifyNode::onConfigSpaceReceived, this, std::placeholders::_1));
        expanded_map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20);
        debug_publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/cfg_space/expanded/image", 20);

        set_device_state(SCR::DeviceState::OPERATING);
    }

    void config_updated(json newConfig) override
    {
        config = newConfig.template get<ExpandifyConfig>();
    }

    json get_default_config() override
    {
        ExpandifyConfig newConfig;
        newConfig.vertical_fov = 2.75;
        newConfig.horizontal_fov = 3;
        newConfig.map_res = 80.0f;
        return newConfig;
    }

    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
    {
        if (updated.state == SCR::SystemState::AUTONOMOUS && device_state == SCR::DeviceState::READY)
        {
            set_device_state(SCR::DeviceState::OPERATING);
        }

        if (updated.state != SCR::SystemState::AUTONOMOUS && device_state == SCR::DeviceState::OPERATING)
        {
            set_device_state(SCR::DeviceState::READY);
        }
    }

    void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
    {
        if (device_state != SCR::DeviceState::OPERATING || system_state != SCR::SystemState::AUTONOMOUS)
        {
            return;
        }

        std::vector<int8_t> cfg_space = std::vector<int8_t>((config.map_res) * config.map_res);
        std::fill(cfg_space.begin(), cfg_space.end(), 0);

        for (int x = 0; x < config.map_res; x++)
        {
            for (int y = 1; y < config.map_res; y++)
            {
                if (cfg->data.at(x + y * config.map_res) > 0)
                {
                    for (Circle &circle : circles)
                    {
                        auto idx = (x + circle.x) + config.map_res * (y + circle.y);
                        auto expr_x = (x + circle.x) < config.map_res && (x + circle.x) >= 0;
                        auto expr_y = (y + circle.y) < config.map_res && (y + circle.y) >= 0;
                        if (expr_x && expr_y)
                        {
                            auto val = cfg_space.at(idx);
                            auto linear = 100 - ((circle.radius - noGoRange) / (maxRange - noGoRange) * 100);

                            if (circle.radius <= noGoRange)
                            {
                                cfg_space.at(idx) = 100;
                            }
                            else if (cfg_space.at(idx) <= 100 && val <= linear)
                            {
                                cfg_space.at(idx) = int(linear);
                            }
                        }
                    }
                }
            }
        }

        auto newSpace = nav_msgs::msg::OccupancyGrid();
        newSpace.info = map;
        newSpace.data = cfg_space;
        expanded_map_publisher->publish(newSpace);

        cv::Mat image = cv::Mat(config.map_res, config.map_res, CV_8UC1, cfg_space.data());
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        cv::resize(image, image, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);

        auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
        compressed->header.stamp = this->now();
        compressed->header.frame_id = "map";
        compressed->format = "jpeg";
        debug_publisher->publish(*compressed);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expanded_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_publisher;

    nav_msgs::msg::MapMetaData map;

    float maxRange = 0.10;
    float noGoPercent = 0.38;
    int noGoRange = 0;
    std::vector<Circle> circles;
    ExpandifyConfig config;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<ExpandifyNode>());
    rclcpp::shutdown();
    return 0;
}