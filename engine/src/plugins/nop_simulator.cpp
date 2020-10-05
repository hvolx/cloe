/*
 * Copyright 2020 Robert Bosch GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * \file plugins/nop_simulator.cpp
 * \see  plugins/nop_simulator.hpp
 */

#include "plugins/nop_simulator.hpp"

#include <functional>  // for function<>
#include <memory>      // for unique_ptr<>
#include <string>      // for string
#include <vector>      // for vector<>

#include <cloe/component/ego_sensor.hpp>        // for NopEgoSensor
#include <cloe/component/latlong_actuator.hpp>  // for LatLongActuator
#include <cloe/component/object_sensor.hpp>     // for NopObjectSensor
#include <cloe/handler.hpp>                     // for ToJson
#include <cloe/models.hpp>                      // for CloeComponent
#include <cloe/registrar.hpp>                   // for Registrar
#include <cloe/simulator.hpp>                   // for Simulator
#include <cloe/sync.hpp>                        // for Sync
#include <cloe/vehicle.hpp>                     // for Vehicle

namespace cloe {
namespace simulator {

struct NopVehicle : public Vehicle {
  NopVehicle(uint64_t id, const std::string& name) : Vehicle(id, name) {
    // clang-format off
    this->new_component(new NopEgoSensor(),
                        CloeComponent::GROUNDTRUTH_EGO_SENSOR,
                        CloeComponent::DEFAULT_EGO_SENSOR);
    this->new_component(new NopObjectSensor(),
                        CloeComponent::GROUNDTRUTH_WORLD_SENSOR,
                        CloeComponent::DEFAULT_WORLD_SENSOR);
    this->new_component(new LatLongActuator(),
                        CloeComponent::DEFAULT_LATLONG_ACTUATOR);
    // clang-format on
  }
};

class NopSimulator : public Simulator {
 public:
  explicit NopSimulator(const std::string& name, const NopConfiguration& c)
      : Simulator(name), config_(c) {}
  virtual ~NopSimulator() noexcept = default;

  void connect() override {
    Simulator::connect();
    for (size_t i = 0; i < config_.vehicles.size(); i++) {
      vehicles_.push_back(std::make_shared<NopVehicle>(i, config_.vehicles[i]));
    }
  }

  void reset() override {
    vehicles_.clear();
    disconnect();
    connect();
  }

  void abort() override {
    // Nothing to do here.
  }

  void enroll(Registrar& r) override {
    r.register_api_handler("/state", HandlerType::BUFFERED, handler::ToJson<NopSimulator>(this));
    r.register_api_handler(
        "/configuration", HandlerType::BUFFERED, handler::ToJson<NopConfiguration>(&config_));
  }

  size_t num_vehicles() const override {
    assert(is_connected());
    return vehicles_.size();
  }

  std::shared_ptr<Vehicle> get_vehicle(size_t i) const override {
    assert(i < num_vehicles());
    return vehicles_[i];
  }

  std::shared_ptr<Vehicle> get_vehicle(const std::string& key) const override {
    for (const auto& v : vehicles_) {
      if (v->name() == key) {
        return v;
      }
    }
    return nullptr;
  }

  Duration process(const Sync& sync) override {
    assert(is_connected());
    assert(is_operational());
    if (finfunc_) {
      operational_ = finfunc_(sync);
    }
    return sync.time();
  }

  friend void to_json(Json& j, const NopSimulator& b) {
    // clang-format off
    j = Json{
        {"connected", b.connected_},
        {"operational", b.operational_},
        {"running", nullptr},
        {"num_vehicles", b.num_vehicles()},
        {"vehicles", b.vehicles_},
    };
    // clang-format on
  }

 private:
  NopConfiguration config_;
  std::vector<std::shared_ptr<Vehicle>> vehicles_;
  std::function<bool(const Sync&)> finfunc_;
};

DEFINE_SIMULATOR_FACTORY_MAKE(NopFactory, NopSimulator)

}  // namespace simulator
}  // namespace cloe
