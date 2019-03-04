use std;
use libc::c_char;
use libc::uintptr_t;
use rclrs_common;
use std::ffi::CString;
use std::ffi::CStr;

@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.base_type.type
}@

#[derive(Default)]
pub struct @(type_name) {
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
  // TODO(esteve): arrays are not supported yet
@[    else]@
@[        if field.type.is_primitive_type()]@
  pub @(field.name): @(get_rs_type(field.type)),
@[        else]@
  // TODO(esteve): nested types are not supported yet
@[        end if]@
@[    end if]@
@[end for]@
}

#[link(name = "@(package_name)__rosidl_typesupport_c__rsext")]
extern "C" {
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() -> uintptr_t;
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
  @(field.name): *const c_char,
@[            else]@
  @(field.name): @(get_rs_type(field.type)),
@[            end if]@
@[        else]@
@[        end if]@
@[    end if]@
@[end for]@
    ) -> uintptr_t;

    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle: uintptr_t) -> ();

@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t) -> *const c_char;
@[            else]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t) -> @(get_rs_type(field.type));
@[            end if]@
@[        else]@
@[        end if]@
@[    end if]@
@[end for]@
}

// impl @(type_name) {
//   pub fn new() -> @(type_name) {
//     @(type_name) {}
//   }
// }

impl @(type_name) {
  fn get_native_message(&self) -> uintptr_t {
    return unsafe { @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
  CString::new(self.@(field.name).clone()).unwrap().as_ptr(),
@[            else]@
  self.@(field.name),
@[            end if]@
@[        else]@
@[        end if]@
@[    end if]@
@[end for]@
    ) };
  }

  fn destroy_native_message(&self, message_handle: uintptr_t) -> () {
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle);
    }
  }

  fn read_handle(&mut self, message_handle: uintptr_t) -> () {
    unsafe {
      {
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
      let ptr = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
      self.@(field.name) = CStr::from_ptr(ptr).to_string_lossy().into_owned();
@[            else]@
      self.@(field.name) = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
@[            end if]@
@[        else]@
@[        end if]@
@[    end if]@
@[end for]@
      }
    }
  }
}

impl rclrs_common::traits::Message for @(type_name) {
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

impl rclrs_common::traits::MessageDefinition<@(type_name)> for @(type_name) {
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

// impl Default for @(type_name) {
//     fn default() -> @(type_name) {
//         @(type_name) {
// 
//         }
//    }
// }

@[end for]
