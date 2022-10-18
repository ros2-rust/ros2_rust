@{
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import NamedType
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import UnboundedString
from rosidl_parser.definition import UnboundedWString
}@
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
}@

#[link(name = "@(package_name)__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() -> *const std::os::raw::c_void;
}

#[link(name = "@(package_name)__rosidl_generator_c")]
extern "C" {
    fn @(package_name)__@(subfolder)__@(type_name)__init(msg: *mut @(type_name)) -> bool;
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>, size: usize) -> bool;
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>);
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<@(type_name)>, out_seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>) -> bool;
}

@# Drop is not needed, since the default drop glue does the same as fini here:
@# it just calls the drop/fini functions of all fields
// Corresponds to @(package_name)__@(subfolder)__@(type_name)
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct @(type_name) {
@[for member in msg_spec.structure.members]@
    pub @(get_rs_name(member.name)): @(get_rmw_rs_type(member.type)),
@[end for]@
}

@[if msg_spec.constants]@
impl @(type_name) {
@[for constant in msg_spec.constants]@
@{
comments = getattr(constant, 'get_comment_lines', lambda: [])()
}@
@[  for line in comments]@
@[    if line]@
    /// @(line)
@[    else]@
    ///
@[    end if]@
@[  end for]@
@[  if isinstance(constant.type, BasicType)]@
    pub const @(get_rs_name(constant.name)): @(get_rmw_rs_type(constant.type)) = @(constant_value_to_rs(constant.type, constant.value));
@[  elif isinstance(constant.type, AbstractGenericString)]@
    pub const @(get_rs_name(constant.name)): &'static str = @(constant_value_to_rs(constant.type, constant.value));
@[  else]@
@{assert False, 'Unhandled constant type: ' + str(constant.type)}@
@[  end if]@
@[end for]@
}
@[end if]

impl Default for @(type_name) {
  fn default() -> Self {
    unsafe {
@#    // SAFETY: This is safe since a zeroed bit pattern always forms a valid message.
      let mut msg = std::mem::zeroed();
@#    // SAFETY: This is safe since the precondititons for init() are fulfilled by giving it a zeroed message.
      if !@(package_name)__@(subfolder)__@(type_name)__init(&mut msg as *mut _) {
        panic!("Call to @(package_name)__@(subfolder)__@(type_name)__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for @(type_name) {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for @(type_name) {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for @(type_name) where Self: Sized {
  const TYPE_NAME: &'static str = "@(package_name)/@(subfolder)/@(type_name)";
  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() }
  }
}

@[end for]
