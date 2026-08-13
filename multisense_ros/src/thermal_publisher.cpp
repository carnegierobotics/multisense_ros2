/**
 * @file thermal_publisher.cpp
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <MultiSense/MultiSenseUtilities.hh>

#include "multisense_ros/thermal_publisher.h"

namespace multisense_ros
{
namespace
{

namespace thermal = multisense::secondary_application::thermal;

constexpr size_t IMAGERS_PER_BANK = 2;
constexpr size_t ENABLE_MASK_BITS_PER_BANK = 4;

bool imager_enabled(const uint32_t enable_mask, const size_t imager_id)
{
    const size_t bank = imager_id / IMAGERS_PER_BANK;
    const size_t bank_imager = imager_id % IMAGERS_PER_BANK;
    const size_t mask_bit = bank * ENABLE_MASK_BITS_PER_BANK + bank_imager;
    return (enable_mask & (1u << mask_bit)) != 0;
}

uint16_t post_processing_mask(const multisense_thermal::Params::Thermal::PostProcessing &parameters)
{
    uint16_t output = 0;
    const auto enable = [&output](const bool enabled, const thermal::PostProcessing correction)
    {
        if (enabled)
        {
            output |= static_cast<uint16_t>(correction);
        }
    };

    enable(parameters.ffc, thermal::PostProcessing::FFC);
    enable(parameters.gain, thermal::PostProcessing::GAIN);
    enable(parameters.temperature, thermal::PostProcessing::TEMPERATURE);
    enable(parameters.bad_pixel, thermal::PostProcessing::BAD_PIXEL);
    enable(parameters.scnr, thermal::PostProcessing::COLUMN_NOISE);
    enable(parameters.srnr, thermal::PostProcessing::ROW_NOISE);
    enable(parameters.tf, thermal::PostProcessing::TEMPORAL_FILTER);
    enable(parameters.spnr, thermal::PostProcessing::SPATIAL_NOISE);
    enable(parameters.sffc, thermal::PostProcessing::SUPPLEMENTAL_FFC);
    enable(parameters.toss, thermal::PostProcessing::HIGH_PASS);
    enable(parameters.bcnr, thermal::PostProcessing::BAD_PIXEL_COLUMN);
    return output;
}

class ThermalStreamGuard
{
public:
    explicit ThermalStreamGuard(multisense::Channel &channel) : channel_(channel) {}
    ~ThermalStreamGuard()
    {
        channel_.stop_streams({multisense::DataSource::THERMAL});
    }

private:
    multisense::Channel &channel_;
};

sensor_msgs::msg::CameraInfo create_camera_info(const multisense::CameraCalibration &calibration,
                                                uint32_t width,
                                                uint32_t height,
                                                const std::string &frame_id,
                                                const rclcpp::Time &stamp)
{
    sensor_msgs::msg::CameraInfo output;
    output.header.frame_id = frame_id;
    output.header.stamp = stamp;
    output.width = width;
    output.height = height;

    switch (calibration.distortion_type)
    {
        case multisense::CameraCalibration::DistortionType::PLUMBBOB:
            output.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            break;
        case multisense::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL:
            output.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            break;
        default:
            break;
    }

    output.d.assign(calibration.D.begin(), calibration.D.end());
    for (size_t row = 0; row < calibration.K.size(); ++row)
    {
        for (size_t column = 0; column < calibration.K[row].size(); ++column)
        {
            output.k[row * calibration.K[row].size() + column] = calibration.K[row][column];
            output.r[row * calibration.R[row].size() + column] = calibration.R[row][column];
        }
        for (size_t column = 0; column < calibration.P[row].size(); ++column)
        {
            output.p[row * calibration.P[row].size() + column] = calibration.P[row][column];
        }
    }
    return output;
}

bool populate_image(const multisense::Image &input,
                    const std::string &frame_id,
                    const rclcpp::Time &stamp,
                    sensor_msgs::msg::Image &output)
{
    if (!input.raw_data || input.image_data_offset < 0 || input.width <= 0 || input.height <= 0)
    {
        return false;
    }

    const auto offset = static_cast<size_t>(input.image_data_offset);
    const auto length = static_cast<size_t>(input.image_data_length);
    if (offset > input.raw_data->size() || length > input.raw_data->size() - offset)
    {
        return false;
    }

    output.header.frame_id = frame_id;
    output.header.stamp = stamp;
    output.width = input.width;
    output.height = input.height;
    output.is_bigendian = (htonl(1) == 1);

    switch (input.format)
    {
        case multisense::Image::PixelFormat::MONO8:
            output.encoding = sensor_msgs::image_encodings::MONO8;
            output.step = input.width;
            break;
        case multisense::Image::PixelFormat::MONO16:
            output.encoding = sensor_msgs::image_encodings::MONO16;
            output.step = input.width * sizeof(uint16_t);
            break;
        default:
            return false;
    }

    output.data.resize(length);
    std::memcpy(output.data.data(), input.raw_data->data() + offset, length);
    return true;
}

} // namespace

std::shared_ptr<ThermalPublisher> ThermalPublisher::create(
    rclcpp::Node *node,
    multisense::Channel &channel,
    const std::string &tf_prefix,
    const rclcpp::QoS &qos,
    const bool use_image_transport,
    PublisherOptionsFactory publisher_options_factory,
    TimestampFactory timestamp_factory)
{
    auto output = std::shared_ptr<ThermalPublisher>(new ThermalPublisher(
        node, channel, tf_prefix, qos, use_image_transport,
        std::move(publisher_options_factory), std::move(timestamp_factory)));
    output->initialize();

    const std::weak_ptr<ThermalPublisher> weak_output = output;
    channel.add_secondary_application_callback(
        [weak_output](const multisense::SecondaryApplicationData &packet)
        {
            if (const auto publisher = weak_output.lock())
            {
                publisher->handle_packet(packet);
            }
        });

    output->processing_thread_ = std::thread(&ThermalPublisher::run, output.get());
    return output;
}

ThermalPublisher::ThermalPublisher(rclcpp::Node *node,
                                   multisense::Channel &channel,
                                   std::string tf_prefix,
                                   rclcpp::QoS qos,
                                   const bool use_image_transport,
                                   PublisherOptionsFactory publisher_options_factory,
                                   TimestampFactory timestamp_factory)
    : node_(node),
      channel_(channel),
      tf_prefix_(std::move(tf_prefix)),
      qos_(std::move(qos)),
      use_image_transport_(use_image_transport),
      publisher_options_factory_(std::move(publisher_options_factory)),
      timestamp_factory_(std::move(timestamp_factory))
{
    if (!node_ || !publisher_options_factory_ || !timestamp_factory_)
    {
        throw std::invalid_argument("Invalid thermal publisher dependency");
    }
}

ThermalPublisher::~ThermalPublisher()
{
    shutdown();
}

void ThermalPublisher::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (stopped_)
        {
            return;
        }
        shutdown_ = true;
        stopped_ = true;
    }
    channel_.add_secondary_application_callback({});
    frame_condition_.notify_all();
    if (processing_thread_.joinable())
    {
        processing_thread_.join();
    }
    channel_.stop_streams({multisense::DataSource::THERMAL});

    for (auto &imager : imagers_)
    {
        imager.reset();
    }
    parameter_listener_.reset();
    publisher_options_factory_ = {};
    timestamp_factory_ = {};
}

void ThermalPublisher::initialize()
{
    // Constructing this listener declares thermal.*. Since ThermalPublisher is
    // only created for STT6 hardware, other devices never expose these parameters.
    parameter_listener_ = std::make_shared<multisense_thermal::ParamListener>(node_);
    const auto parameters = parameter_listener_->get_params();

    thermal::Config config;
    std::array<std::optional<multisense::CameraCalibration>, MAX_IMAGERS> calibrations{};
    {
        if (channel_.start_streams({multisense::DataSource::THERMAL}) != multisense::Status::OK)
        {
            throw std::runtime_error("Unable to activate the thermal application");
        }
        ThermalStreamGuard stream_guard(channel_);

        const auto queried_config = thermal::query_config(channel_);
        if (!queried_config)
        {
            throw std::runtime_error("Unable to query the thermal configuration");
        }
        config = queried_config.value();

        config.rectified = parameters.thermal.rectified;
        config.bits_per_pixel = static_cast<uint8_t>(parameters.thermal.bits_per_pixel);
        config.post_proc_mask = post_processing_mask(parameters.thermal.post_processing);
        if (const auto status = thermal::send_config(channel_, config); status != multisense::Status::OK)
        {
            throw std::runtime_error("Unable to apply the thermal configuration: " +
                                     multisense::to_string(status));
        }

        for (size_t id = 0; id < MAX_IMAGERS; ++id)
        {
            if (imager_enabled(config.imager_enable_mask, id))
            {
                calibrations[id] = thermal::get_calibration(channel_, static_cast<uint8_t>(id));
            }
        }
    }

    for (size_t id = 0; id < MAX_IMAGERS; ++id)
    {
        if (!imager_enabled(config.imager_enable_mask, id))
        {
            continue;
        }

        const std::string topic = "thermal/imager_" + std::to_string(id) + "/image";
        const std::string frame_id = tf_prefix_ + "/thermal_" + std::to_string(id) + "_optical_frame";

        multisense::CameraCalibration calibration;
        if (calibrations[id])
        {
            calibration = calibrations[id].value();
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Unable to query calibration for thermal imager %zu; publishing uncalibrated CameraInfo",
                        id);
        }

        auto camera_info = create_camera_info(calibration, config.width, config.height,
                                              frame_id, node_->now());
        auto publisher = std::make_shared<ImagePublisher>(
            node_, topic, camera_info, qos_,
            publisher_options_factory_({multisense::DataSource::THERMAL},
                                       get_full_topic_name(node_, topic)),
            use_image_transport_);

        ImagerPublisher imager;
        imager.publisher = std::move(publisher);
        imager.camera_info = std::move(camera_info);
        imager.frame_id = frame_id;
        imagers_[id] = std::move(imager);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Configured STT6 thermal output: %ux%u mono%u %s, enable mask 0x%02x, correction mask 0x%03x",
                static_cast<unsigned>(config.width), static_cast<unsigned>(config.height),
                static_cast<unsigned>(config.bits_per_pixel),
                config.rectified ? "rectified" : "raw", config.imager_enable_mask,
                static_cast<unsigned>(config.post_proc_mask.value_or(0)));
}

void ThermalPublisher::handle_packet(const multisense::SecondaryApplicationData &packet)
{
    if (packet.application.name != thermal::APPLICATION_NAME || packet.output_index != 0)
    {
        return;
    }

    auto frame = thermal::deserialize_frame_group(packet.payload);
    if (!frame)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Ignoring an invalid thermal frame group");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (shutdown_)
        {
            return;
        }
        pending_frame_ = std::move(frame.value());
    }
    frame_condition_.notify_one();
}

void ThermalPublisher::run()
{
    while (true)
    {
        std::optional<FrameGroup> frame;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_condition_.wait(lock, [this]() { return shutdown_ || pending_frame_.has_value(); });
            if (shutdown_)
            {
                return;
            }
            frame = std::move(pending_frame_);
            pending_frame_.reset();
        }

        const auto stamp = timestamp_factory_(frame.value());
        if (!stamp)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Thermal frame has no valid timestamp for the configured time source");
            continue;
        }

        for (const auto &thermal_image : frame->images)
        {
            const size_t id = thermal_image.imager_id;
            if (id >= imagers_.size() || !imagers_[id])
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Ignoring thermal image from disabled or invalid imager %zu", id);
                continue;
            }

            auto &imager = imagers_[id].value();
            if (imager.publisher->get_subscription_count() == 0)
            {
                continue;
            }

            auto image = std::make_unique<sensor_msgs::msg::Image>();
            if (!populate_image(thermal_image.image, imager.frame_id, stamp.value(), *image))
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Ignoring invalid thermal image from imager %zu", id);
                continue;
            }

            imager.camera_info.header = image->header;
            imager.camera_info.width = image->width;
            imager.camera_info.height = image->height;
            imager.publisher->publish(
                std::move(image),
                std::make_unique<sensor_msgs::msg::CameraInfo>(imager.camera_info));
        }
    }
}

} // namespace multisense_ros
