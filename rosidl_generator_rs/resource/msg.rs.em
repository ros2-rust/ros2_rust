@{
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
    fn @(package_name)__@(subfolder)__@(type_name)__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<@(type_name)>);
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
      out_seq.resize_to_at_least(in_seq.len());
      out_seq.clone_from_slice(in_seq.as_slice());
      true
  }
}

impl rosidl_runtime_rs::Message for @(type_name) {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for @(type_name) where Self: Sized {
  fn get_type_support() -> libc::uintptr_t {
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() }
  }
}

@[end for]
}  // mod rmw

@# #################################################
@# ############ Idiomatic message types ############
@# #################################################
@# These types use standard Rust containers where possible.
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::@(type_name)::default())
  }
}

impl rosidl_runtime_rs::Message for @(type_name) {
  type RmwMsg = crate::msg::rmw::@(type_name);

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
@[for member in msg_spec.structure.members]@
@#
@#
@#    == Array ==
@[    if isinstance(member.type, Array)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .map(|elem| elem.as_str().into()),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned()),
@[        elif isinstance(member.type.value_type, BasicType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@[        else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).clone(),
@[        end if]@
@#
@#
@#    == UnboundedString + UnboundedWString ==
@[    elif isinstance(member.type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).as_str().into(),
@#
@#
@#    == UnboundedSequence ==
@[    elif isinstance(member.type, UnboundedSequence)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .into_iter()
          .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
@[        else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).into(),
@[        end if]@
@#
@#
@#    == NamedType + NamespacedType ==
@[    elif isinstance(member.type, NamedType) or isinstance(member.type, NamespacedType)]@
        @(get_rs_name(member.name)): @(get_idiomatic_rs_type(member.type))::into_rmw_message(std::borrow::Cow::Owned(msg.@(get_rs_name(member.name)))).into_owned(),
@#
@#
@#    == Bounded and basic types ==
@[    else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@[    end if]@
@[end for]@
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
@[for member in msg_spec.structure.members]@
@#
@#
@#    == Array ==
@[    if isinstance(member.type, Array)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .iter()
          .map(|elem| elem.as_str().into())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .iter()
          .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
@[        elif isinstance(member.type.value_type, BasicType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@[        else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).clone(),
@[        end if]@
@#
@#
@#    == UnboundedString + UnboundedWString ==
@[    elif isinstance(member.type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).as_str().into(),
@#
@#
@#    == UnboundedSequence ==
@[    elif isinstance(member.type, UnboundedSequence)]@
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type, UnboundedWString)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
@[        elif isinstance(member.type.value_type, NamedType) or isinstance(member.type.value_type, NamespacedType)]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name))
          .iter()
          .map(|elem| @(get_idiomatic_rs_type(member.type.value_type))::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
@[        else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).as_slice().into(),
@[        end if]@
@#
@#
@#    == NamedType + NamespacedType ==
@[    elif isinstance(member.type, NamedType) or isinstance(member.type, NamespacedType)]@
        @(get_rs_name(member.name)): @(get_idiomatic_rs_type(member.type))::into_rmw_message(std::borrow::Cow::Borrowed(&msg.@(get_rs_name(member.name)))).into_owned(),
@#
@#
@#    == BasicType ==
@[    elif isinstance(member.type, BasicType)]@
      @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)),
@#
@#
@#    == Bounded types ==
@[    else]@
        @(get_rs_name(member.name)): msg.@(get_rs_name(member.name)).clone(),
@[    end if]@
@[end for]@
      })
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
        .map(@(get_idiomatic_rs_type(member.type.value_type))::from_rmw_message),
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
          .map(@(get_idiomatic_rs_type(member.type.value_type))::from_rmw_message)
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
