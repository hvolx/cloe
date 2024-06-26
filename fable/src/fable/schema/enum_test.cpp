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
 * \file fable/schema/enum_test.cpp
 * \see  fable/schema/enum.hpp
 */

#include <gtest/gtest.h>

#include <fable/confable.hpp>       // for Confable
#include <fable/schema.hpp>         // for Struct, ...
#include <fable/utility/gtest.hpp>  // for assert_to_json, ...

namespace logger {
enum LogLevel {
  nada = 0,
  info = 1,
  debug = 2,
};
}

// clang-format off
FABLE_ENUM_SERIALIZATION(logger::LogLevel, ({
    {logger::LogLevel::nada, "unknown"},
    {logger::LogLevel::info, "info"},
    {logger::LogLevel::debug, "debug"},
}))
// clang-format on

using LogLevel = logger::LogLevel;

struct LoggerStruct : public fable::Confable {
  std::optional<LogLevel> level;

  CONFABLE_SCHEMA(LoggerStruct) {
    using namespace fable::schema;  // NOLINT(build/namespaces)
    return Struct{
        {"level", make_schema(&level, "")},
    };
  }
};

TEST(fable_schema_enum, struct_enum) {
  LoggerStruct l;
  fable::assert_to_json(l, "{}");
  fable::assert_from_eq_to(l, fable::Json{{"level", "info"}});
}

TEST(fable_schema_enum, vector_enum_ok) {
  using namespace fable::schema;  // NOLINT(build/namespaces)
  std::vector<LogLevel> xs;
  Vector<LogLevel, Enum<LogLevel>> s{&xs, ""};

  fable::assert_to_json(s, "[]");
  fable::assert_from_eq_to(s, R"([
    "info"
  ])");
  ASSERT_EQ(xs.size(), 1);
  ASSERT_EQ(xs[0], LogLevel::info);
}

TEST(fable_schema_enum, vector_struct_enum) {
  using namespace fable::schema;  // NOLINT(build/namespaces)
  std::vector<LoggerStruct> xs;
  Vector<LoggerStruct, FromConfable<LoggerStruct>> s{&xs, ""};

  fable::assert_to_json(s, "[]");
  fable::assert_from_eq_to(s, R"([
    {"level": "info"}
  ])");

  ASSERT_EQ(xs.size(), 1);
  ASSERT_EQ(*(xs[0].level), LogLevel::info);
}
