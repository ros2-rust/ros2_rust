// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally ported from ROS1:
// https://github.com/ros/common_msgs/blob/89069bc/sensor_msgs/include/sensor_msgs/fill_image.h

#ifndef SENSOR_MSGS__FILL_IMAGE_HPP_
#define SENSOR_MSGS__FILL_IMAGE_HPP_

#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace sensor_msgs
{
/// Fill an image message.
/**
 * \param[out] image Image to be filled.
 * \param[in] encoding_arg Encoding type, such as sensor_msgs::image_encodings::RGB8.
 * \param[in] rows_arg Number of rows.
 * \param[in] cols_arg Number of columns.
 * \param[in] step_arg Step size.
 * \param[in] data_arg Data to fill image with.
 * \return True if successful.
 */
static inline bool fillImage(
  msg::Image & image,
  const std::string & encoding_arg,
  uint32_t rows_arg,
  uint32_t cols_arg,
  uint32_t step_arg,
  const void * data_arg)
{
  image.encoding = encoding_arg;
  image.height = rows_arg;
  image.width = cols_arg;
  image.step = step_arg;
  size_t st0 = (step_arg * rows_arg);
  image.data.resize(st0);
  memcpy(&image.data[0], data_arg, st0);

  image.is_bigendian = 0;
  return true;
}

/// Clear the data of an image message.
/**
 * \details All fields but `data` are kept the same.
 * \param[out]image Image to be cleared.
 */
static inline void clearImage(msg::Image & image)
{
  image.data.resize(0);
}
}  // namespace sensor_msgs

#endif  // SENSOR_MSGS__FILL_IMAGE_HPP_
