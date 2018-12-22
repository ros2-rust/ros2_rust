use std;
#[allow(unused_imports)]
use libc::*;
use rclrs_common;
#[allow(unused_imports)]
use std::ffi::CString;
#[allow(unused_imports)]
use std::ffi::CStr;
#[allow(unused_imports)]
use rclrs_common::traits::Message;

@{
includes = {}
}@
@
@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.base_type.type
}@
@
@{
for field in msg_spec.fields:
    if not field.type.is_primitive_type():
        key = field.type.pkg_name
        if key == package_name:
          continue
        if key not in includes:
            includes[key] = set([])
        includes[key].add(type_name + "." + field.name)
}@

#[derive(Default, Debug, Clone)]
pub struct @(type_name) {
@[for field in msg_spec.fields]@
  pub @(sanitize_identifier(field.name)): @(get_rs_type(field.type, package_name)),
@[end for]@
}

#[link(name = "@(package_name)__rosidl_typesupport_c__rsext")]
extern "C" {
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() -> uintptr_t;
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
@[for field in msg_spec.fields]@
      @(sanitize_identifier(field.name)): @(get_ffi_type(field.type, package_name)),
@[    if field.type.is_array ]@
      @(sanitize_identifier(field.name))__len: size_t,
@[    end if]@
@[end for]@
    ) -> uintptr_t;

    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle: uintptr_t) -> ();

@[for field in msg_spec.fields]@
@[    if field.type.is_array ]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t, size: *mut size_t) -> @(get_ffi_return_type(field.type, package_name));
@[    else]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t) -> @(get_ffi_return_type(field.type, package_name));
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
@[    if field.type.is_array ]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
      self.@(sanitize_identifier(field.name)).iter().map(|s| CString::new(s.clone()).unwrap().as_ptr()).collect::<Vec<_>>().as_ptr(),
@[            else]@
      self.@(sanitize_identifier(field.name)).as_ptr(),
@[            end if]@
@[        else]@
      self.@(sanitize_identifier(field.name)).as_ptr() as *const libc::c_void,
@[        end if]@
      self.@(sanitize_identifier(field.name)).len() as size_t,
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
      CString::new(self.@(sanitize_identifier(field.name)).clone()).unwrap().as_ptr(),
@[            else]@
      self.@(sanitize_identifier(field.name)),
@[            end if]@
@[        else]@
      self.@(sanitize_identifier(field.name)).get_native_message() as *const _,
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
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[        if field.type.is_primitive_type() and field.type.array_size]@
      let mut @(field.name)_size = 0usize;
      self.@(sanitize_identifier(field.name)).copy_from_slice(std::slice::from_raw_parts(
          @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(
              message_handle, &mut @(field.name)_size as *mut _) as *const _, @(field.type.array_size)));
@[        else]@
      let mut @(field.name)_size = 0usize;
      let @(field.name)_array = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle, &mut @(field.name)_size as *mut _);
      self.@(sanitize_identifier(field.name)) = std::slice::from_raw_parts(@(field.name)_array as *const _, @(field.name)_size).to_vec();
@[        end if]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
      let ptr = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
      self.@(sanitize_identifier(field.name)) = CStr::from_ptr(ptr).to_string_lossy().into_owned();
@[            else]@
      self.@(sanitize_identifier(field.name)) = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
@[            end if]@
@[        else]@
      self.@(sanitize_identifier(field.name)).read_handle(@(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle));
@[        end if]@
@[    end if]@
@[end for]@
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
@
@{
for key in sorted(includes.keys()):
    print('use %s;  // %s' % (key, ', '.join(includes[key])))
}@