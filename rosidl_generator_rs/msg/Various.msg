float32 float32_value 1.125
float64 float64_value 2.4
uint64 uint64_value
string string_value "bar"
float32[10] float32_arr
float64[3] float64_arr [8.5, 1.2, 3.4]
float64[2] float64_arr2 [99.0, 2.3]
string[3] string_arr
# TODO(clalancette): default values for strings aren't currently implemented;
# see https://github.com/ros2/rosidl/issues/206
#string[3] string_def_arr ["hello", "there", "world"]
float32[] unbounded [1, 2]
float32[<=3] bounded_no_def
float32[<=3] bounded_def [3, 4]
FieldsWithSameTypeSomeDefaults vec3
FieldsWithSameTypeSomeDefaults[2] vec3_fixed
FieldsWithSameTypeSomeDefaults[] vec3_unbounded
FieldsWithSameTypeSomeDefaults[<=2] vec3_bounded
