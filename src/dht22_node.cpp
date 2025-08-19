#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>

using namespace std::chrono_literals;

// Utilidades de tiempo en ns/us
static inline uint64_t now_ns() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch()).count();
}
static inline void busy_sleep_us(uint64_t us) {
  const uint64_t target = now_ns() + us * 1000ULL; // 1 us = 1000 ns
  while (now_ns() < target) { /* busy wait */ }
}

// libgpiod helpers (v1)
static inline int line_get(struct gpiod_line* line) {
  int v = gpiod_line_get_value(line);
  return v < 0 ? -1 : v;
}
static bool wait_level(struct gpiod_line* line, int level, int timeout_us) {
  const uint64_t limit = now_ns() + (uint64_t)timeout_us * 1000ULL;
  while (now_ns() < limit) {
    int v = line_get(line);
    if (v < 0) return false;
    if (v == level) return true;
  }
  return false;
}
// Medir HIGH: llamar justo tras subir HIGH; devuelve us del HIGH o <0 si timeout
static double measure_high_pulse_us(struct gpiod_line* line, int max_us) {
  const uint64_t start = now_ns();
  const uint64_t limit = start + (uint64_t)max_us * 1000ULL;
  while (now_ns() < limit) {
    int v = line_get(line);
    if (v < 0) return -1.0;
    if (v == 0) {
      uint64_t dur_ns = now_ns() - start;
      return (double)dur_ns / 1000.0;
    }
  }
  return -1.0;
}
static bool acquire_line_output(struct gpiod_chip* chip, int offset, struct gpiod_line** out, int value) {
  *out = gpiod_chip_get_line(chip, offset);
  if (!*out) return false;
  gpiod_line_release(*out);
  if (gpiod_line_request_output(*out, "dht22", value) < 0) return false;
  return true;
}
static bool acquire_line_input(struct gpiod_chip* chip, int offset, struct gpiod_line** out) {
  *out = gpiod_chip_get_line(chip, offset);
  if (!*out) return false;
  gpiod_line_release(*out);
  if (gpiod_line_request_input(*out, "dht22") < 0) return false;
  return true;
}

class Dht22Node : public rclcpp::Node {
public:
  Dht22Node() : Node("dht22_node") {
    chip_path_ = this->declare_parameter<std::string>("chip_path", "/dev/gpiochip4");
    gpio_line_ = this->declare_parameter<int>("gpio_line", 12);
    rate_hz_   = this->declare_parameter<double>("rate_hz", 0.5);
    tries_per_sample_ = this->declare_parameter<int>("tries_per_sample", 4);
    frame_id_  = this->declare_parameter<std::string>("frame_id", "dht22_link");

    // Enforce mínimo 0.5 Hz (>= 2 s)
    if (rate_hz_ > 0.5) {
      RCLCPP_WARN(this->get_logger(),
                  "rate_hz=%.3f excede lo permitido por DHT22; forzando a 0.5 Hz (2 s).",
                  rate_hz_);
      rate_hz_ = 0.5;
    }
    if (rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "rate_hz inválido; usando 0.5 Hz.");
      rate_hz_ = 0.5;
    }

    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/dht22/temperature", 10);
    hum_pub_  = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("/dht22/humidity", 10);

    period_ = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(period_, std::bind(&Dht22Node::tick_, this));

    RCLCPP_INFO(this->get_logger(),
                "DHT22 en chip=%s line=%d, rate=%.3f Hz, tries=%d",
                chip_path_.c_str(), gpio_line_, rate_hz_, tries_per_sample_);
  }

private:
  void tick_() {
    double t_c = 0.0, rh = 0.0;
    bool ok = false;

    for (int i = 0; i < std::max(1, tries_per_sample_); ++i) {
      ok = read_once_on_chip_(chip_path_, gpio_line_, t_c, rh);
      if (ok) break;
      std::this_thread::sleep_for(400ms);
    }

    auto stamp = this->now();
    if (ok) {
      sensor_msgs::msg::Temperature tmsg;
      tmsg.header.stamp = stamp;
      tmsg.header.frame_id = frame_id_;
      tmsg.temperature = t_c;
      tmsg.variance = 0.0;
      temp_pub_->publish(tmsg);

      sensor_msgs::msg::RelativeHumidity hmsg;
      hmsg.header.stamp = stamp;
      hmsg.header.frame_id = frame_id_;
      hmsg.relative_humidity = rh; // fracción 0..1
      hmsg.variance = 0.0;
      hum_pub_->publish(hmsg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Lectura DHT22 fallida (se publicará en el próximo tick).");
    }
  }

  bool read_once_on_chip_(const std::string& chip_path, int offset, double& temp_c, double& rh_frac) {
    struct gpiod_chip* chip = gpiod_chip_open(chip_path.c_str());
    if (!chip) return false;

    struct gpiod_line* line = nullptr;

    // Start: LOW ~1.2 ms
    if (!acquire_line_output(chip, offset, &line, 0)) { gpiod_chip_close(chip); return false; }
    busy_sleep_us(1200);

    // Entrada (pull-up mantiene HIGH)
    if (!acquire_line_input(chip, offset, &line)) { gpiod_chip_close(chip); return false; }

    // Handshake: LOW (~80us), HIGH (~80us), LOW (inicio bits)
    if (!wait_level(line, 0, 5000)) { gpiod_chip_close(chip); return false; }
    if (!wait_level(line, 1, 500))  { gpiod_chip_close(chip); return false; }
    if (!wait_level(line, 0, 500))  { gpiod_chip_close(chip); return false; }

    // 40 bits
    std::vector<int> bits; bits.reserve(40);
    for (int i = 0; i < 40; ++i) {
      if (!wait_level(line, 1, 300)) { gpiod_chip_close(chip); return false; }
      double high_us = measure_high_pulse_us(line, 200);
      if (high_us < 0.0) { gpiod_chip_close(chip); return false; }
      bits.push_back(high_us > 50.0 ? 1 : 0);
    }

    gpiod_chip_close(chip);

    if ((int)bits.size() != 40) return false;

    auto bits_to_byte = [&](int start) {
      int v = 0;
      for (int i = start; i < start + 8; ++i) v = (v << 1) | bits[i];
      return v;
    };

    int b0 = bits_to_byte(0);
    int b1 = bits_to_byte(8);
    int b2 = bits_to_byte(16);
    int b3 = bits_to_byte(24);
    int b4 = bits_to_byte(32);

    int sum = (b0 + b1 + b2 + b3) & 0xFF;
    if (sum != b4) return false;

    double rh_pct = ((b0 << 8) | b1) / 10.0;
    int t_raw = ((b2 & 0x7F) << 8) | b3;
    double t_c = t_raw / 10.0;
    if (b2 & 0x80) t_c = -t_c;

    temp_c = t_c;
    rh_frac = std::clamp(rh_pct / 100.0, 0.0, 1.0);
    return true;
  }

  // Parámetros
  std::string chip_path_;
  int gpio_line_;
  double rate_hz_;
  int tries_per_sample_;
  std::string frame_id_;

  // Pub & timer
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr hum_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::duration<double> period_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dht22Node>());
  rclcpp::shutdown();
  return 0;
}
