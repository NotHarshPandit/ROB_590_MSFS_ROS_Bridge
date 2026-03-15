#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>   // <-- needed for std::fixed, std::setprecision

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
  double airspeed_indicated_kts;      // AIRSPEED INDICATED (knots)
  double altitude_ft;                 // INDICATED ALTITUDE (feet)
  double roll_deg;                    // PLANE BANK DEGREES (degrees)
  double pitch_deg;                   // PLANE PITCH DEGREES (degrees)
  double heading_mag_deg;             // PLANE HEADING DEGREES MAGNETIC (degrees)
  double dir_gyro_deg;                // HEADING INDICATOR (degrees)
  double vertical_speed_fpm;          // VERTICAL SPEED (ft/min)
  double side_slip_deg;               // SIDE SLIP ANGLE (degrees)
  double baro_setting_inhg;           // KOHLSMAN SETTING HG (inHg)

  // GPS position
  double gps_lat_deg;                 // GPS POSITION LAT (degrees)
  double gps_lon_deg;                 // GPS POSITION LON (degrees)
  double gps_alt_ft;                  // GPS POSITION ALT (feet)

  // Engine
  double eng1_rpm;                    // GENERAL ENG RPM:1 (rpm)

  // Controls / inputs
  double yoke_aileron;                // AILERON POSITION (normalized -1..1)
  double yoke_elevator;               // ELEVATOR POSITION (normalized -1..1)
  double rudder;                      // RUDDER POSITION (normalized -1..1)
  double brake_left_pct;              // BRAKE LEFT POSITION (percent)
  double brake_right_pct;             // BRAKE RIGHT POSITION (percent)
  double flaps_handle_pct;            // FLAPS HANDLE PERCENT (0..100)
  double trim_pct;                    // ELEVATOR TRIM PCT (0..100)
  double throttle1_pct;               // GENERAL ENG THROTTLE LEVER POSITION:1 (percent)
  double mixture1_pct;                // GENERAL ENG MIXTURE LEVER POSITION:1 (percent)
};

class MsfsSensorNode : public rclcpp::Node
{
public:
  MsfsSensorNode()
  : Node("msfs_sensor_node"),
    sim_connected_(false)
  {
    RCLCPP_INFO(get_logger(), "Starting MSFS sensor node...");

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/msfs/sensor_readings", 10);

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

    // Timer: poll SimConnect ~20 Hz
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
      "AIRSPEED INDICATED", "knots");                 // airspeed_indicated_kts

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "INDICATED ALTITUDE", "feet");                  // altitude_ft

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE BANK DEGREES", "degrees");              // roll_deg (bank)

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE PITCH DEGREES", "degrees");             // pitch_deg

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "PLANE HEADING DEGREES MAGNETIC", "degrees");  // heading_mag_deg

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "HEADING INDICATOR", "degrees");               // dir_gyro_deg

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "VERTICAL SPEED", "feet per minute");          // vertical_speed_fpm

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "SIDE SLIP ANGLE", "degrees");                 // side_slip_deg

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "KOHLSMAN SETTING HG", "inHg");                // baro_setting_inhg :contentReference[oaicite:1]{index=1}

    // ---- GPS coordinates ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION LAT", "degrees");                // gps_lat_deg :contentReference[oaicite:2]{index=2}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION LON", "degrees");                // gps_lon_deg

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GPS POSITION ALT", "feet");                   // gps_alt_ft (SimConnect converts from meters)

    // ---- Engine ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG RPM:1", "rpm");                   // eng1_rpm :contentReference[oaicite:3]{index=3}

    // ---- Controls / inputs ----
    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "AILERON POSITION", "position");               // yoke_aileron [-1..1] :contentReference[oaicite:4]{index=4}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "ELEVATOR POSITION", "position");              // yoke_elevator [-1..1]

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "RUDDER POSITION", "position");                // rudder [-1..1]

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "BRAKE LEFT POSITION", "percent");             // brake_left_pct :contentReference[oaicite:5]{index=5}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "BRAKE RIGHT POSITION", "percent");            // brake_right_pct

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "FLAPS HANDLE PERCENT", "percent over 100");   // flaps_handle_pct :contentReference[oaicite:6]{index=6}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "ELEVATOR TRIM PCT", "percent over 100");      // trim_pct :contentReference[oaicite:7]{index=7}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG THROTTLE LEVER POSITION:1", "percent"); // throttle1_pct :contentReference[oaicite:8]{index=8}

    SimConnect_AddToDataDefinition(
      hSimConnect_, DEFINITION_AIRCRAFT_DATA,
      "GENERAL ENG MIXTURE LEVER POSITION:1", "percent");  // mixture1_pct

    // No SimConnect_RegisterDataDefineStruct<T> helper in the MSFS 2024 headers,
    // so we'll reinterpret the buffer as AircraftData manually.
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
      RCLCPP_ERROR(get_logger(),
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

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    // --- Grouped exactly like your list ---

    // 1) Primary flight instruments
    oss << "Airspeed: " << data->airspeed_indicated_kts << " kts\n";
    oss << "Altitude: " << data->altitude_ft << " ft (indicated)\n";
    oss << "Roll: "    << data->roll_deg    << " deg\n";
    oss << "Pitch: "   << data->pitch_deg   << " deg\n";
    oss << "Magnetic Heading: " << data->heading_mag_deg << " deg\n";
    oss << "Directional Gyro:  " << data->dir_gyro_deg   << " deg\n";
    oss << "Vertical Speed: "   << data->vertical_speed_fpm << " ft/min\n";
    oss << "Side Slip: "        << data->side_slip_deg      << " deg\n";
    oss << "Baro Setting: "     << data->baro_setting_inhg  << " inHg\n\n";

    // 2) GPS
    oss << "GPS Coordinates:\n";
    oss << "  Lat: " << data->gps_lat_deg  << " deg\n";
    oss << "  Lon: " << data->gps_lon_deg  << " deg\n";
    oss << "  Alt: " << data->gps_alt_ft   << " ft\n\n";

    // 3) Engine
    oss << "Engine 1:\n";
    oss << "  RPM: " << data->eng1_rpm << " rpm\n\n";

    // 4) Controls
    oss << "Controls / Inputs:\n";
    oss << "  Yoke (Aileron):  " << data->yoke_aileron   << " [-1..1]\n";
    oss << "  Yoke (Elevator): " << data->yoke_elevator  << " [-1..1]\n";
    oss << "  Rudder:          " << data->rudder         << " [-1..1]\n";
    oss << "  Brakes Left:     " << data->brake_left_pct  << " %\n";
    oss << "  Brakes Right:    " << data->brake_right_pct << " %\n";
    oss << "  Flaps Handle:    " << data->flaps_handle_pct << " %\n";
    oss << "  Elevator Trim:   " << data->trim_pct          << " %\n";
    oss << "  Throttle 1:      " << data->throttle1_pct     << " %\n";
    oss << "  Mixture 1:       " << data->mixture1_pct      << " %\n";

    auto msg = std_msgs::msg::String();
    msg.data = oss.str();
    publisher_->publish(msg);
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
      // Periodically retry connecting
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
          RCLCPP_WARN(get_logger(),
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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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