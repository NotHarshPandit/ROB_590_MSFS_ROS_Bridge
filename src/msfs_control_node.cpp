#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <windows.h>
#include "SimConnect.h"

using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

// Client event IDs
enum EVENT_ID
{
  EVT_AXIS_AILERONS_SET = 1,
  EVT_AXIS_ELEVATOR_SET,
  EVT_AXIS_RUDDER_SET,
  EVT_AXIS_THROTTLE1_SET,
  EVT_AXIS_FLAPS_SET,
  EVT_AXIS_ELEV_TRIM_SET,
  EVT_AXIS_LEFT_BRAKE_SET,
  EVT_AXIS_RIGHT_BRAKE_SET,
  EVT_AXIS_MIXTURE1_SET,
};

// Data definition IDs
enum DATA_DEFINE_ID
{
  DEFINITION_BARO = 100,
  DEFINITION_DG   = 101,
};

// Structs for SetDataOnSimObject
struct BaroData
{
  double kohlsman_inhg;   // "KOHLSMAN SETTING HG:1", inHg
};

struct GyroData
{
  double heading_gyro_rad;  // "PLANE HEADING DEGREES GYRO", radians
};

class MsfsControlNode : public rclcpp::Node
{
public:
  MsfsControlNode()
  : Node("msfs_control_node"),
    sim_connected_(false)
  {
    RCLCPP_INFO(get_logger(), "Starting MSFS control node ...");

    // Connect to SimConnect
    HRESULT hr = SimConnect_Open(
      &hSimConnect_,
      "ROS2 MSFS Control Node",
      nullptr,
      0,
      0,
      0);

    if (hr != S_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "SimConnect_Open failed (hr=0x%08lx). Is MSFS running and in a flight?",
        hr);
      return;
    }

    sim_connected_ = true;
    RCLCPP_INFO(get_logger(), "Connected to SimConnect.");

    map_events();
    define_settable_simvars();

    // --- ROS subscribers ---

    // Roll / pitch / yaw in [-1,1]
        // Roll / pitch / yaw in [-1,1]
    sub_attitude_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/msfs/cmd/attitude", 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg)
      {
        this->attitude_callback(msg);
      });

    // Throttle (0..1)
    sub_throttle_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/throttle", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->throttle_callback(msg);
      });

    // Mixture (0..1)
    sub_mixture_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/mixture", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->mixture_callback(msg);
      });

    // Flaps (0..1)
    sub_flaps_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/flaps", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->flaps_callback(msg);
      });

    // Trim (-1..1 or 0..1)
    sub_trim_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/trim", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->trim_callback(msg);
      });

    // Brakes (0..1)
    sub_brake_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/brake", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->brake_callback(msg);
      });

    // Baro setting (inHg)
    sub_baro_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/baro_inhg", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->baro_callback(msg);
      });

    // Directional gyro heading (deg)
    sub_dg_ = this->create_subscription<std_msgs::msg::Float32>(
      "/msfs/cmd/dg_deg", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        this->dg_callback(msg);
      });

    // Timer to poll SimConnect
    timer_ = this->create_wall_timer(
      50ms,
      [this]()
      {
        this->poll_simconnect();
      });

    RCLCPP_INFO(get_logger(), "MSFS control node ready.");
  }

  ~MsfsControlNode() override
  {
    if (sim_connected_ && hSimConnect_ != nullptr) {
      SimConnect_Close(hSimConnect_);
    }
  }

private:
  // Map our client events to SimConnect event names
  void map_events()
  {
    if (!sim_connected_) return;

    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_AILERONS_SET, "AXIS_AILERONS_SET");
    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_ELEVATOR_SET, "AXIS_ELEVATOR_SET");
    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_RUDDER_SET,   "AXIS_RUDDER_SET");

    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_THROTTLE1_SET, "AXIS_THROTTLE1_SET");
    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_FLAPS_SET,     "AXIS_FLAPS_SET");

    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_ELEV_TRIM_SET, "AXIS_ELEV_TRIM_SET");

    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_LEFT_BRAKE_SET,  "AXIS_LEFT_BRAKE_SET");
    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_RIGHT_BRAKE_SET, "AXIS_RIGHT_BRAKE_SET");

    SimConnect_MapClientEventToSimEvent(
      hSimConnect_, EVT_AXIS_MIXTURE1_SET, "AXIS_MIXTURE1_SET");
  }

  // Define settable SimVars for baro and gyro via SetDataOnSimObject
  void define_settable_simvars()
  {
    if (!sim_connected_) return;

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_BARO,
      "KOHLSMAN SETTING HG:1", "inHg");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_DG,
      "PLANE HEADING DEGREES GYRO", "radians");
  }

  // Helpers for axis scaling
  static DWORD norm_to_axis_dword(float value)
  {
    // Clamp to [-1, 1] without std::max/min (Windows macros conflict)
    float clamped = value;
    if (clamped < -1.0f) clamped = -1.0f;
    if (clamped >  1.0f) clamped =  1.0f;

    int32_t raw = static_cast<int32_t>(clamped * 16383.0f);
    return static_cast<DWORD>(raw);
  }

  static DWORD zero_one_to_axis_dword(float value)
  {
    // Clamp to [0, 1] manually
    float clamped = value;
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;

    int32_t raw = static_cast<int32_t>(clamped * 16383.0f);
    return static_cast<DWORD>(raw);
  }
  void send_axis_event(EVENT_ID evt, float normalized)
  {
    if (!sim_connected_) return;

    DWORD dw = norm_to_axis_dword(normalized);

    HRESULT hr = SimConnect_TransmitClientEvent(
      hSimConnect_,
      SIMCONNECT_OBJECT_ID_USER,
      evt,
      dw,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST,
      SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

    if (hr != S_OK) {
      RCLCPP_WARN(get_logger(),
        "SimConnect_TransmitClientEvent (evt=%d) failed (hr=0x%08lx)", evt, hr);
    }
  }

  void send_axis01_event(EVENT_ID evt, float value)
  {
    if (!sim_connected_) return;

    DWORD dw = zero_one_to_axis_dword(value);

    HRESULT hr = SimConnect_TransmitClientEvent(
      hSimConnect_,
      SIMCONNECT_OBJECT_ID_USER,
      evt,
      dw,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST,
      SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

    if (hr != S_OK) {
      RCLCPP_WARN(get_logger(),
        "SimConnect_TransmitClientEvent (evt=%d) failed (hr=0x%08lx)", evt, hr);
    }
  }

  // ======================
  // ROS callbacks
  // ======================

  void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    if (!sim_connected_) return;

    send_axis_event(EVT_AXIS_AILERONS_SET, msg->x);
    send_axis_event(EVT_AXIS_ELEVATOR_SET, msg->y);
    send_axis_event(EVT_AXIS_RUDDER_SET,   msg->z);
  }

  void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;
    RCLCPP_INFO(get_logger(),"Throttle command: %.3f", msg->data);
    send_axis01_event(EVT_AXIS_THROTTLE1_SET, msg->data);
  }

  void mixture_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;
    send_axis01_event(EVT_AXIS_MIXTURE1_SET, msg->data);
  }

  void flaps_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;
    send_axis01_event(EVT_AXIS_FLAPS_SET, msg->data);
  }

  void trim_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;
    send_axis_event(EVT_AXIS_ELEV_TRIM_SET, msg->data);
  }

  void brake_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;

    float v = msg->data;
    DWORD dw = zero_one_to_axis_dword(v);

    HRESULT hr1 = SimConnect_TransmitClientEvent(
      hSimConnect_,
      SIMCONNECT_OBJECT_ID_USER,
      EVT_AXIS_LEFT_BRAKE_SET,
      dw,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST,
      SIMCONNECT_EVENT_FLAG_DEFAULT);

    HRESULT hr2 = SimConnect_TransmitClientEvent(
      hSimConnect_,
      SIMCONNECT_OBJECT_ID_USER,
      EVT_AXIS_RIGHT_BRAKE_SET,
      dw,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST,
      SIMCONNECT_EVENT_FLAG_DEFAULT);

    if (hr1 != S_OK || hr2 != S_OK) {
      RCLCPP_WARN(get_logger(),
        "Brake events failed (hr1=0x%08lx, hr2=0x%08lx)", hr1, hr2);
    }
  }

  void baro_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;

    BaroData data;
    data.kohlsman_inhg = static_cast<double>(msg->data);

    HRESULT hr = SimConnect_SetDataOnSimObject(
      hSimConnect_,
      DEFINITION_BARO,
      SIMCONNECT_OBJECT_ID_USER,
      0, 0,
      sizeof(BaroData),
      &data);

    if (hr != S_OK) {
      RCLCPP_WARN(get_logger(),
        "SimConnect_SetDataOnSimObject (BARO) failed (hr=0x%08lx)", hr);
    }
  }

  void dg_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!sim_connected_) return;

    double deg = static_cast<double>(msg->data);
    double rad = deg * kPi / 180.0;

    GyroData data;
    data.heading_gyro_rad = rad;

    HRESULT hr = SimConnect_SetDataOnSimObject(
      hSimConnect_,
      DEFINITION_DG,
      SIMCONNECT_OBJECT_ID_USER,
      0, 0,
      sizeof(GyroData),
      &data);

    if (hr != S_OK) {
      RCLCPP_WARN(get_logger(),
        "SimConnect_SetDataOnSimObject (DG) failed (hr=0x%08lx)", hr);
    }
  }

  void poll_simconnect()
  {
    if (!sim_connected_) return;

    SIMCONNECT_RECV* pData = nullptr;
    DWORD cbData = 0;

    while (SUCCEEDED(SimConnect_GetNextDispatch(hSimConnect_, &pData, &cbData))) {
      switch (pData->dwID) {
        case SIMCONNECT_RECV_ID_EXCEPTION:
        {
          auto *exc = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(pData);
          RCLCPP_WARN(get_logger(),
            "SimConnect exception: %u", exc->dwException);
          break;
        }
        case SIMCONNECT_RECV_ID_QUIT:
          RCLCPP_WARN(get_logger(), "SimConnect QUIT received, disconnecting.");
          sim_connected_ = false;
          break;
        default:
          break;
      }
    }
  }

  HANDLE hSimConnect_{nullptr};
  bool sim_connected_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_attitude_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_throttle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_mixture_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_flaps_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_trim_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_brake_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_baro_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       sub_dg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MsfsControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}