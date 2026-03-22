#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <windows.h>
#include "SimConnect.h"

using namespace std::chrono_literals;

enum DATA_DEFINE_ID {
  DEFINITION_AIRCRAFT_DATA = 1,
};

enum DATA_REQUEST_ID {
  REQUEST_AIRCRAFT_DATA = 1,
};

// Order MUST match SimConnect_AddToDataDefinition calls
struct AircraftData
{
  // Basic flight data
  double airspeed_indicated_kts;
  double altitude_ft;
  double roll_deg;
  double pitch_deg;
  double heading_mag_deg;
  double dir_gyro_deg;
  double vertical_speed_fpm;
  double side_slip_deg;
  double baro_setting_inhg;

  // GPS position
  double gps_lat_deg;
  double gps_lon_deg;
  double gps_alt_ft;

  // Engine
  double eng1_rpm;

  // Controls / inputs
  double yoke_aileron;      // [-1..1]
  double yoke_elevator;     // [-1..1]
  double rudder;            // [-1..1]
  double parking_brake;     // 0 = off, >0 = on (from BRAKE PARKING POSITION, 0..32767)
  double flaps_handle_pct;  // 0..100
  double trim_pct;          // 0..100
  double throttle1_pct;     // 0..100
  double mixture1_pct;      // 0..100
};

class MsfsSensorNode : public rclcpp::Node
{
public:
  MsfsSensorNode()
  : Node("msfs_sensor_node"),
    sim_connected_(false)
  {
    RCLCPP_INFO(get_logger(), "Starting MSFS sensor node...");

    // Human-readable debug publisher
    sensor_string_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/msfs/sensor_readings", 10);

    // Replayable command/state publishers
    attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/msfs/cmd/attitude", 10);

    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/throttle", 10);

    flaps_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/flaps", 10);

    mixture_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/mixture", 10);

    trim_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/trim", 10);

    parking_brake_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/parking_brake", 10);

    baro_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/baro", 10);

    dg_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/msfs/cmd/dg_deg", 10);

    // Try initial SimConnect connection
    HRESULT hr = SimConnect_Open(
      &hSimConnect_,
      "ROS2 MSFS Sensor Node",
      nullptr,
      0,
      0,
      0);

    if (hr != S_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "SimConnect_Open failed (hr=0x%08lx). Is MSFS running and in a flight?",
        hr);
    } else {
      RCLCPP_INFO(get_logger(), "Connected to SimConnect.");
      sim_connected_ = true;
      define_data();
      request_data();
    }

    // Poll SimConnect periodically
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MsfsSensorNode::timer_callback, this));
  }

  ~MsfsSensorNode() override
  {
    if (sim_connected_ && hSimConnect_ != nullptr) {
      SimConnect_Close(hSimConnect_);
    }
  }

private:
  void define_data()
  {
    // ---- Basic flight data ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "AIRSPEED INDICATED", "knots");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "INDICATED ALTITUDE", "feet");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE BANK DEGREES", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE PITCH DEGREES", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE HEADING DEGREES MAGNETIC", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "HEADING INDICATOR", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "VERTICAL SPEED", "feet per minute");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "INCIDENCE BETA", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "KOHLSMAN SETTING HG", "inHg");

    // ---- GPS coordinates ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION LAT", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION LON", "degrees");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION ALT", "feet");

    // ---- Engine ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG RPM:1", "rpm");

    // ---- Controls / inputs ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "AILERON POSITION", "position");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "ELEVATOR POSITION", "position");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "RUDDER POSITION", "position");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "BRAKE PARKING POSITION", "position");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "FLAPS HANDLE PERCENT", "percent over 100");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "ELEVATOR TRIM PCT", "percent over 100");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG THROTTLE LEVER POSITION:1", "percent");

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG MIXTURE LEVER POSITION:1", "percent");
  }

  void request_data()
  {
    HRESULT hr = SimConnect_RequestDataOnSimObject(
      hSimConnect_,
      REQUEST_AIRCRAFT_DATA,
      DEFINITION_AIRCRAFT_DATA,
      SIMCONNECT_OBJECT_ID_USER,
      SIMCONNECT_PERIOD_SIM_FRAME,
      SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT,
      0, 0, 0);

    if (hr != S_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "SimConnect_RequestDataOnSimObject failed (hr=0x%08lx)", hr);
    } else {
      RCLCPP_INFO(get_logger(), "Requested aircraft data each sim frame.");
    }
  }

  void handle_simobject_data(SIMCONNECT_RECV_SIMOBJECT_DATA *pObjData)
  {
    if (pObjData->dwRequestID != REQUEST_AIRCRAFT_DATA) {
      return;
    }

    const AircraftData *data =
      reinterpret_cast<const AircraftData*>(&pObjData->dwData);

    // -----------------------------
    // 1) Publish human-readable debug string
    // -----------------------------
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss << "Airspeed: " << data->airspeed_indicated_kts << " kts\n";
    oss << "Altitude: " << data->altitude_ft << " ft (indicated)\n";
    oss << "Roll: " << data->roll_deg << " deg\n";
    oss << "Pitch: " << data->pitch_deg << " deg\n";
    oss << "Magnetic Heading: " << data->heading_mag_deg << " deg\n";
    oss << "Directional Gyro: " << data->dir_gyro_deg << " deg\n";
    oss << "Vertical Speed: " << data->vertical_speed_fpm << " ft/min\n";
    oss << "Side Slip: " << data->side_slip_deg << " deg\n";
    oss << "Baro Setting: " << data->baro_setting_inhg << " inHg\n\n";

    oss << "GPS Coordinates:\n";
    oss << "  Lat: " << data->gps_lat_deg << " deg\n";
    oss << "  Lon: " << data->gps_lon_deg << " deg\n";
    oss << "  Alt: " << data->gps_alt_ft << " ft\n\n";

    oss << "Engine 1:\n";
    oss << "  RPM: " << data->eng1_rpm << " rpm\n\n";

    oss << "Controls / Inputs:\n";
    oss << "  Yoke (Aileron):  " << data->yoke_aileron << " [-1..1]\n";
    oss << "  Yoke (Elevator): " << data->yoke_elevator << " [-1..1]\n";
    oss << "  Rudder:          " << data->rudder << " [-1..1]\n";
    oss << "  Parking Brake:   " << (data->parking_brake ? "ON" : "OFF") << "\n";
    oss << "  Flaps Handle:    " << data->flaps_handle_pct << " %\n";
    oss << "  Elevator Trim:   " << data->trim_pct << " %\n";
    oss << "  Throttle 1:      " << data->throttle1_pct << " %\n";
    oss << "  Mixture 1:       " << data->mixture1_pct << " %\n";

    std_msgs::msg::String string_msg;
    string_msg.data = oss.str();
    sensor_string_pub_->publish(string_msg);

    // -----------------------------
    // 2) Publish replayable command topics
    // -----------------------------

    // attitude: x=aileron, y=elevator, z=rudder
    geometry_msgs::msg::Vector3 attitude_msg;
    attitude_msg.x = data->yoke_aileron;
    attitude_msg.y = data->yoke_elevator;
    attitude_msg.z = data->rudder;
    attitude_pub_->publish(attitude_msg);

    // throttle: normalize 0..100 -> 0..1
    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = static_cast<float>(data->throttle1_pct / 100.0);
    throttle_pub_->publish(throttle_msg);

    // flaps: normalize 0..100 -> 0..1
    std_msgs::msg::Float32 flaps_msg;
    flaps_msg.data = static_cast<float>(data->flaps_handle_pct / 100.0);
    flaps_pub_->publish(flaps_msg);

    // mixture: normalize 0..100 -> 0..1
    std_msgs::msg::Float32 mixture_msg;
    mixture_msg.data = static_cast<float>(data->mixture1_pct / 100.0);
    mixture_pub_->publish(mixture_msg);

    // trim: normalize 0..100 -> 0..1
    std_msgs::msg::Float32 trim_msg;
    trim_msg.data = static_cast<float>(data->trim_pct / 100.0);
    trim_pub_->publish(trim_msg);

    // parking brake: 1 = engaged, 0 = disengaged
    std_msgs::msg::Float32 parking_brake_msg;
    parking_brake_msg.data = (data->parking_brake != 0.0) ? 1.0f : 0.0f;
    parking_brake_pub_->publish(parking_brake_msg);

    // baro setting in inHg
    std_msgs::msg::Float32 baro_msg;
    baro_msg.data = static_cast<float>(data->baro_setting_inhg);
    baro_pub_->publish(baro_msg);

    // directional gyro setting in degrees
    std_msgs::msg::Float32 dg_msg;
    dg_msg.data = static_cast<float>(data->dir_gyro_deg);
    dg_pub_->publish(dg_msg);
  }

  void poll_simconnect()
  {
    SIMCONNECT_RECV *pData = nullptr;
    DWORD cbData = 0;

    while (SUCCEEDED(SimConnect_GetNextDispatch(hSimConnect_, &pData, &cbData))) {
      switch (pData->dwID) {
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
        {
          auto *pObjData = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(pData);
          handle_simobject_data(pObjData);
          break;
        }
        case SIMCONNECT_RECV_ID_EXCEPTION:
        {
          auto *exc = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(pData);
          RCLCPP_WARN(get_logger(), "SimConnect exception: %u", exc->dwException);
          break;
        }
        case SIMCONNECT_RECV_ID_QUIT:
          RCLCPP_WARN(get_logger(), "SimConnect QUIT received, stopping.");
          sim_connected_ = false;
          g_running_ = false;
          break;
        default:
          break;
      }
    }
  }

  void timer_callback()
  {
    if (!sim_connected_) {
      static int counter = 0;
      if (++counter % 20 == 0) {
        RCLCPP_INFO(get_logger(), "Trying to reconnect to SimConnect...");
        HRESULT hr = SimConnect_Open(
          &hSimConnect_,
          "ROS2 MSFS Sensor Node",
          nullptr, 0, 0, 0);
        if (hr == S_OK) {
          RCLCPP_INFO(get_logger(), "Reconnected to SimConnect.");
          sim_connected_ = true;
          define_data();
          request_data();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "SimConnect_Open still failing (hr=0x%08lx)", hr);
        }
      }
      return;
    }

    poll_simconnect();
  }

  HANDLE hSimConnect_{nullptr};
  bool sim_connected_;
  bool g_running_{true};

  // Debug publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_string_pub_;

  // Replayable control-state publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flaps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mixture_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trim_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr parking_brake_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr baro_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dg_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MsfsSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}