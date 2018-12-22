# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from test_msgs.srv import Empty
from test_msgs.srv import Primitives


def get_msg_empty():
    req = Empty.Request()
    resp = Empty.Response()

    return [[req, resp]]


def get_msg_primitives():
    srvs = []
    req = Primitives.Request()
    req.bool_value = False
    req.byte_value = bytes([0])
    req.char_value = '\x00'
    req.float32_value = float(0.0)
    req.float64_value = float(0.0)
    req.int8_value = 0
    req.uint8_value = 0
    req.int16_value = 0
    req.uint16_value = 0
    req.int32_value = 0
    req.uint32_value = 0
    req.int64_value = 0
    req.uint64_value = 0
    req.string_value = 'request'
    resp = Primitives.Response()
    resp.bool_value = False
    resp.byte_value = bytes([0])
    resp.char_value = '\x00'
    resp.float32_value = float(0.0)
    resp.float64_value = float(0.0)
    resp.int8_value = 0
    resp.uint8_value = 0
    resp.int16_value = 0
    resp.uint16_value = 0
    resp.int32_value = 0
    resp.uint32_value = 0
    resp.int64_value = 0
    resp.uint64_value = 0
    resp.string_value = 'reply'
    srvs.append([req, resp])

    req = Primitives.Request()
    req.bool_value = True
    req.byte_value = bytes([1])
    req.char_value = '\x01'
    req.float32_value = float(1.125)
    req.float64_value = float(1.11)
    req.int8_value = 1
    req.uint8_value = 2
    req.int16_value = 3
    req.uint16_value = 4
    req.int32_value = 5
    req.uint32_value = 6
    req.int64_value = 7
    req.uint64_value = 8
    # check strings longer then 256 characters
    req.string_value = ''
    for i in range(20000):
        req.string_value += str(i % 10)
    resp = Primitives.Response()
    resp.bool_value = True
    resp.byte_value = bytes([11])
    resp.char_value = '\x11'
    resp.float32_value = float(11.125)
    resp.float64_value = float(11.11)
    resp.int8_value = 11
    resp.uint8_value = 22
    resp.int16_value = 33
    resp.uint16_value = 44
    resp.int32_value = 55
    resp.uint32_value = 66
    resp.int64_value = 77
    resp.uint64_value = 88
    # check strings longer then 256 characters
    resp.string_value = ''
    for i in range(20000):
        resp.string_value += str(i % 10)
    srvs.append([req, resp])
    return srvs


def get_test_srv(service_name):
    if 'Empty' == service_name:
        srv = get_msg_empty()
    elif 'Primitives' == service_name:
        srv = get_msg_primitives()
    else:
        raise NotImplementedError('%s service is not part of the test suite', service_name)
    return srv
