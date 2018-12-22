// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <rosidl_typesupport_cpp/action_type_support.hpp>
#include "test_msgs/action/fibonacci.hpp"


TEST(ActionCppTypeSupport, can_get_type_support)
{
  const rosidl_action_type_support_t * ts =
    rosidl_typesupport_cpp::get_action_type_support_handle<test_msgs::action::Fibonacci>();
  ASSERT_NE(nullptr, ts);
}
