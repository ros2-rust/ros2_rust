// Copyright (c) 2009, Willow Garage, Inc.
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

// This file is originally from:
// https://github.com/ros/common_msgs/blob/846bfcb/sensor_msgs/include/sensor_msgs/image_encodings.h

#ifndef SENSOR_MSGS__IMAGE_ENCODINGS_HPP_
#define SENSOR_MSGS__IMAGE_ENCODINGS_HPP_

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace sensor_msgs
{
namespace image_encodings
{
const char RGB8[] = "rgb8";
const char RGBA8[] = "rgba8";
const char RGB16[] = "rgb16";
const char RGBA16[] = "rgba16";
const char BGR8[] = "bgr8";
const char BGRA8[] = "bgra8";
const char BGR16[] = "bgr16";
const char BGRA16[] = "bgra16";
const char MONO8[] = "mono8";
const char MONO16[] = "mono16";

// OpenCV CvMat types
const char TYPE_8UC1[] = "8UC1";
const char TYPE_8UC2[] = "8UC2";
const char TYPE_8UC3[] = "8UC3";
const char TYPE_8UC4[] = "8UC4";
const char TYPE_8SC1[] = "8SC1";
const char TYPE_8SC2[] = "8SC2";
const char TYPE_8SC3[] = "8SC3";
const char TYPE_8SC4[] = "8SC4";
const char TYPE_16UC1[] = "16UC1";
const char TYPE_16UC2[] = "16UC2";
const char TYPE_16UC3[] = "16UC3";
const char TYPE_16UC4[] = "16UC4";
const char TYPE_16SC1[] = "16SC1";
const char TYPE_16SC2[] = "16SC2";
const char TYPE_16SC3[] = "16SC3";
const char TYPE_16SC4[] = "16SC4";
const char TYPE_32SC1[] = "32SC1";
const char TYPE_32SC2[] = "32SC2";
const char TYPE_32SC3[] = "32SC3";
const char TYPE_32SC4[] = "32SC4";
const char TYPE_32FC1[] = "32FC1";
const char TYPE_32FC2[] = "32FC2";
const char TYPE_32FC3[] = "32FC3";
const char TYPE_32FC4[] = "32FC4";
const char TYPE_64FC1[] = "64FC1";
const char TYPE_64FC2[] = "64FC2";
const char TYPE_64FC3[] = "64FC3";
const char TYPE_64FC4[] = "64FC4";

// Bayer encodings
const char BAYER_RGGB8[] = "bayer_rggb8";
const char BAYER_BGGR8[] = "bayer_bggr8";
const char BAYER_GBRG8[] = "bayer_gbrg8";
const char BAYER_GRBG8[] = "bayer_grbg8";
const char BAYER_RGGB16[] = "bayer_rggb16";
const char BAYER_BGGR16[] = "bayer_bggr16";
const char BAYER_GBRG16[] = "bayer_gbrg16";
const char BAYER_GRBG16[] = "bayer_grbg16";

// Miscellaneous
// This is the UYVY version of YUV422 codec http://www.fourcc.org/yuv.php#UYVY
// with an 8-bit depth
const char YUV422[] = "yuv422";

// Prefixes for abstract image encodings
const char ABSTRACT_ENCODING_PREFIXES[][5] = {
  "8UC", "8SC", "16UC", "16SC", "32SC", "32FC", "64FC"
};

// Utility functions for inspecting an encoding string
static inline bool isColor(const std::string & encoding)
{
  return encoding == RGB8 || encoding == BGR8 ||
         encoding == RGBA8 || encoding == BGRA8 ||
         encoding == RGB16 || encoding == BGR16 ||
         encoding == RGBA16 || encoding == BGRA16;
}

static inline bool isMono(const std::string & encoding)
{
  return encoding == MONO8 || encoding == MONO16;
}

static inline bool isBayer(const std::string & encoding)
{
  return encoding == BAYER_RGGB8 || encoding == BAYER_BGGR8 ||
         encoding == BAYER_GBRG8 || encoding == BAYER_GRBG8 ||
         encoding == BAYER_RGGB16 || encoding == BAYER_BGGR16 ||
         encoding == BAYER_GBRG16 || encoding == BAYER_GRBG16;
}

static inline bool hasAlpha(const std::string & encoding)
{
  return encoding == RGBA8 || encoding == BGRA8 ||
         encoding == RGBA16 || encoding == BGRA16;
}

static inline int numChannels(const std::string & encoding)
{
  // First do the common-case encodings
  if (encoding == MONO8 ||
    encoding == MONO16)
  {
    return 1;
  }
  if (encoding == BGR8 ||
    encoding == RGB8 ||
    encoding == BGR16 ||
    encoding == RGB16)
  {
    return 3;
  }
  if (encoding == BGRA8 ||
    encoding == RGBA8 ||
    encoding == BGRA16 ||
    encoding == RGBA16)
  {
    return 4;
  }
  if (encoding == BAYER_RGGB8 ||
    encoding == BAYER_BGGR8 ||
    encoding == BAYER_GBRG8 ||
    encoding == BAYER_GRBG8 ||
    encoding == BAYER_RGGB16 ||
    encoding == BAYER_BGGR16 ||
    encoding == BAYER_GBRG16 ||
    encoding == BAYER_GRBG16)
  {
    return 1;
  }

  // Now all the generic content encodings
  // TODO(clalancette): Rewrite with regex when ROS supports C++11
  for (size_t i = 0; i < sizeof(ABSTRACT_ENCODING_PREFIXES) / sizeof(*ABSTRACT_ENCODING_PREFIXES);
    i++)
  {
    std::string prefix = ABSTRACT_ENCODING_PREFIXES[i];
    if (encoding.substr(0, prefix.size()) != prefix) {
      continue;
    }
    if (encoding.size() == prefix.size()) {
      return 1;  // ex. 8UC -> 1
    }
    int n_channel = atoi(encoding.substr(prefix.size(),
        encoding.size() - prefix.size()).c_str());  // ex. 8UC5 -> 5
    if (n_channel != 0) {
      return n_channel;  // valid encoding string
    }
  }

  if (encoding == YUV422) {
    return 2;
  }

  throw std::runtime_error("Unknown encoding " + encoding);
  return -1;
}

static inline int bitDepth(const std::string & encoding)
{
  if (encoding == MONO16) {
    return 16;
  }
  if (encoding == MONO8 ||
    encoding == BGR8 ||
    encoding == RGB8 ||
    encoding == BGRA8 ||
    encoding == RGBA8 ||
    encoding == BAYER_RGGB8 ||
    encoding == BAYER_BGGR8 ||
    encoding == BAYER_GBRG8 ||
    encoding == BAYER_GRBG8)
  {
    return 8;
  }

  if (encoding == MONO16 ||
    encoding == BGR16 ||
    encoding == RGB16 ||
    encoding == BGRA16 ||
    encoding == RGBA16 ||
    encoding == BAYER_RGGB16 ||
    encoding == BAYER_BGGR16 ||
    encoding == BAYER_GBRG16 ||
    encoding == BAYER_GRBG16)
  {
    return 16;
  }

  // Now all the generic content encodings
  // TODO(clalancette): Rewrite with regex when ROS supports C++11
  for (size_t i = 0; i < sizeof(ABSTRACT_ENCODING_PREFIXES) / sizeof(*ABSTRACT_ENCODING_PREFIXES);
    i++)
  {
    std::string prefix = ABSTRACT_ENCODING_PREFIXES[i];
    if (encoding.substr(0, prefix.size()) != prefix) {
      continue;
    }
    if (encoding.size() == prefix.size()) {
      return atoi(prefix.c_str());  // ex. 8UC -> 8
    }
    int n_channel = atoi(encoding.substr(prefix.size(),
        encoding.size() - prefix.size()).c_str());  // ex. 8UC10 -> 10
    if (n_channel != 0) {
      return atoi(prefix.c_str());  // valid encoding string
    }
  }

  if (encoding == YUV422) {
    return 8;
  }

  throw std::runtime_error("Unknown encoding " + encoding);
  return -1;
}
}  // namespace image_encodings
}  // namespace sensor_msgs

#endif  // SENSOR_MSGS__IMAGE_ENCODINGS_HPP_
