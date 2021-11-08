use libc::c_char;
use libc::uintptr_t;
use rclrs_msg_utilities;
use std::ffi::CString;
use std::ffi::CStr;

@{
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import Array
}@

@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
}@

#[derive(Default)]
pub struct @(type_name) {
@[for member in msg_spec.structure.members]@
    pub @(get_rs_name(member.name)): @(get_rs_type(member.type).replace(package_name, 'crate')),
@[end for]@
}

#[link(name = "@(package_name)__rosidl_typesupport_c__rsext")]
extern "C" {
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() -> uintptr_t;

    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
@[for member in msg_spec.structure.members]@
@[    if isinstance(member.type, AbstractGenericString)]@
    @(get_rs_name(member.name)): *const c_char,
@[    elif isinstance(member.type, BasicType)]@
    @(get_rs_name(member.name)): @(get_rs_type(member.type)),
@[    end if]@
@[end for]@
    ) -> uintptr_t;

    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle: uintptr_t) -> ();

@[for member in msg_spec.structure.members]@
@[    if isinstance(member.type, Array)]@
@[    elif isinstance(member.type, AbstractGenericString)]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(member.name)_read_handle(message_handle: uintptr_t) -> *const c_char;
@[    elif isinstance(member.type, BasicType)]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(member.name)_read_handle(message_handle: uintptr_t) -> @(get_rs_type(member.type));
@[    end if]@
@[end for]@
}

impl @(type_name) {
  fn get_native_message(&self) -> uintptr_t {
    return unsafe { @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
@[for member in msg_spec.structure.members]@
@[    if isinstance(member.type, Array)]@
@[    elif isinstance(member.type, AbstractGenericString)]@
    {let s = CString::new(self.@(get_rs_name(member.name)).clone()).unwrap();
    let p = s.as_ptr();
    std::mem::forget(s);
    p},
@[    elif isinstance(member.type, BasicType)]@
    self.@(get_rs_name(member.name)),
@[    end if]@
@[end for]@
    ) };
  }

  fn destroy_native_message(&self, message_handle: uintptr_t) -> () {
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle);
    }
  }

  #[allow(unused_unsafe)]
  fn read_handle(&mut self, _message_handle: uintptr_t) -> () {
    unsafe {
      {
@[for member in msg_spec.structure.members]@
@[    if isinstance(member.type, Array)]@
@[    elif isinstance(member.type, AbstractGenericString)]@
      let ptr = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(member.name)_read_handle(_message_handle);
      self.@(get_rs_name(member.name)) = CStr::from_ptr(ptr).to_string_lossy().into_owned();
@[    elif isinstance(member.type, BasicType)]@
      self.@(get_rs_name(member.name)) = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(member.name)_read_handle(_message_handle);
@[    elif isinstance(member.type, AbstractSequence)]@
@[    end if]@
@[end for]@
      }
    }
  }
}

impl rclrs_msg_utilities::traits::Message for @(type_name) {
  fn get_native_message(&self) -> uintptr_t {
    return self.get_native_message();
  }

  fn destroy_native_message(&self, message_handle: uintptr_t) -> () {
    self.destroy_native_message(message_handle);
  }

  fn read_handle(&mut self, message_handle: uintptr_t) -> () {
    self.read_handle(message_handle);
  }
}

impl rclrs_msg_utilities::traits::MessageDefinition<@(type_name)> for @(type_name) {
  fn get_type_support() -> uintptr_t {
    return unsafe { @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() };
  }

  fn static_get_native_message(message: &@(type_name)) -> uintptr_t {
    return message.get_native_message();
  }

  fn static_destroy_native_message(message_handle: uintptr_t) -> () {
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle);
    }
  }
}

@[end for]
