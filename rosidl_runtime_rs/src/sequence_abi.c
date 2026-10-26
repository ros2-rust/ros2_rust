// Reports the layout of the C structs in header files installed with the ROS 2 distribution.
// rosidl_runtime_rs casts its types to those C structs so we need to make sure the sizes and
// alignments match.
//
// Every sequence type, `String` and `U16String` included, is declared through
// the ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE macro, so one primitive sequence
// stands in for the whole family, and only the string sequences need measuring
// separately because their element structs have a size of their own.

#include <stddef.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/u16string.h>
#include <builtin_interfaces/msg/detail/time__struct.h>

typedef rosidl_runtime_c__double__Sequence primitive_sequence;

// A member placed after a char sits at that member's alignment, which reports
// the alignment without _Alignof and the C11 that it needs.
struct alignment_probe
{
  char before;
  primitive_sequence sequence;
};

// The sizes of the structs a Sequence<T> is cast to.
const size_t rosidl_rs_primitive_sequence_size = sizeof(primitive_sequence);
const size_t rosidl_rs_string_sequence_size = sizeof(rosidl_runtime_c__String__Sequence);
const size_t rosidl_rs_u16string_sequence_size = sizeof(rosidl_runtime_c__U16String__Sequence);
// Message-element sequences were not extended in rosidl#942; they stay
// pointer/size/capacity on every distro, including Lyrical+.
const size_t rosidl_rs_message_sequence_size = sizeof(builtin_interfaces__msg__Time__Sequence);

// Where C keeps the three fields both sides read and write, and how it aligns
// the struct that holds them.
const size_t rosidl_rs_sequence_data_offset = offsetof(primitive_sequence, data);
const size_t rosidl_rs_sequence_size_offset = offsetof(primitive_sequence, size);
const size_t rosidl_rs_sequence_capacity_offset = offsetof(primitive_sequence, capacity);
const size_t rosidl_rs_sequence_align = offsetof(struct alignment_probe, sequence);

// The flags belong to the sequence structs, so the string structs themselves
// keep the same size on every distro.
const size_t rosidl_rs_string_size = sizeof(rosidl_runtime_c__String);
const size_t rosidl_rs_u16string_size = sizeof(rosidl_runtime_c__U16String);
