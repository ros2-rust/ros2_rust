@{
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
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

pub mod rmw {
@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
}@

#[link(name = "@(package_name)__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() -> libc::uintptr_t;
}

#[link(name = "@(package_name)__rosidl_generator_c")]
extern "C" {
    fn @(package_name)__@(subfolder)__@(type_name)__init(msg: *mut @(type_name)) -> bool;
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>, size: libc::size_t) -> bool;
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>) -> ();
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__copy(in_seq: *const rosidl_runtime_rs::Sequence<@(type_name)>, out_seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>) -> bool;
}

@# Drop is not needed, since the default drop glue does the same as fini here:
@# it just calls the drop/fini functions of all fields
// Corresponds to @(package_name)__@(subfolder)__@(type_name)
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct @(type_name) {
@[for member in msg_spec.structure.members]@
    pub @(get_rs_name(member.name)): @(get_rmw_rs_type(member.type)),
@[end for]@
}

impl Default for @(type_name) {
  fn default() -> Self {
    unsafe {
@#    // This is safe since a zeroed bit pattern always forms a valid message.
      let mut msg = std::mem::zeroed();
@#    // This is safe since the precondititons for inti() are fulfilled by giving it a zeroed message.
      if !@(package_name)__@(subfolder)__@(type_name)__init(&mut msg as *mut _) {
        panic!("Call to @(package_name)__@(subfolder)__@(type_name)__init() failed");
      }
      msg
    }
  }
}



impl rosidl_runtime_rs::SequenceAlloc for @(type_name) {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: libc::size_t) -> bool {
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    unsafe { @(package_name)__@(subfolder)__@(type_name)__Sequence__copy(in_seq as *const _, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for @(type_name) {
  type RmwMsg = Self;
  fn into_rmw_message(self) -> Self::RmwMsg { self }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for @(type_name) where Self: Sized {
  fn get_type_support() -> libc::uintptr_t {
    return unsafe { rosidl_typesupport_c__get_message_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() };
  }
}

@[end for]
}

@# #################################################
@# ############ Idiomatic message types ############
@# #################################################
@# These types use standard Rust containers where possible.
#[allow(unused_imports)]
use rosidl_runtime_rs::{Message, Sequence};
@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
}@

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct @(type_name) {
@[for member in msg_spec.structure.members]@
    pub @(get_rs_name(member.name)): @(get_idiomatic_rs_type(member.type)),
@[end for]@
}

impl Default for @(type_name) {
  fn default() -> Self {
@#  This has the benefit of automatically setting the right default values
    <Self as Message>::from_rmw_message(crate::msg::rmw::@(type_name)::default())
  }
}

impl Message for @(type_name) {
  type RmwMsg = crate::msg::rmw::@(type_name);

  fn into_rmw_message(self) -> Self::RmwMsg {
    Self::RmwMsg {
@[for member in msg_spec.structure.members]@
@#
@#
@#    == Array ==
@[    if isinstance(member.type, Array)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name))
        .map(|elem| elem.as_str().into()),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name))
        .map(|elem| elem.into_rmw_message()),
@[        else]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name)).clone(),
@[        end if]@
@#
@#
@#    == UnboundedString + UnboundedWString ==
@[    elif isinstance(member.type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name)).as_str().into(),
@#
@#
@#    == UnboundedSequence ==
@[    elif isinstance(member.type, UnboundedSequence)]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name))
        .into_iter()
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
        .map(|elem| elem.as_str().into())
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
        .map(|elem| elem.into_rmw_message())
@[        end if]@
        .collect(),
@#
@#
@#    == NamedType + NamespacedType ==
@[    elif isinstance(member.type, NamedType) or isinstance(member.type, NamespacedType)]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name)).into_rmw_message(),
@#
@#
@#    == Bounded and basic types ==
@[    else]@
      @(get_rs_name(member.name)): self.@(get_rs_name(member.name)),
@[    end if]@
@[end for]@
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
@[for member in msg_spec.structure.members]@
@#
@#
@#    == Array ==
@[    if isinstance(member.type, Array)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
        .map(|elem| elem.to_string()),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
        .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::from_rmw_message(elem)),
@[        else]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@[        end if]@
@#
@#
@#    == UnboundedSequence ==
@[    elif isinstance(member.type, UnboundedSequence)]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .into_iter()
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
          .map(|elem| elem.to_string())
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
          .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::from_rmw_message(elem))
@[        end if]@
          .collect(),
@#
@#
@#    == UnboundedString + UnboundedWString ==
@[    elif isinstance(member.type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).to_string(),
@#
@#
@#    == NamedType + NamespacedType ==
@[    elif isinstance(member.type, NamedType) or isinstance(member.type, NamespacedType)]@
      @(get_rs_name(member.name)): @(get_idiomatic_rs_type(member.type))::from_rmw_message(msg.@(get_rs_name(member.name))),
@#
@#
@#    == Bounded and basic types ==
@[    else]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@[    end if]@
@[end for]@
    }
  }
}

@[end for]
