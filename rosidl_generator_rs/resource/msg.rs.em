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
@
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

#[allow(non_camel_case_types)]
#[derive(Default, Debug, Clone, PartialEq)]
pub struct @(type_name) {
@[for field in msg_spec.fields]@
  pub @(sanitize_identifier(field.name)): @(get_rs_type(field.type, package_name)),
@[end for]@
}

#[link(name = "@(package_name)__rosidl_typesupport_c__rsext")]
extern "C" {
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() -> uintptr_t;
@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_create_native_message() -> uintptr_t;
@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message_at(
      message_handle: uintptr_t,
@[for field in msg_spec.fields]@
@[    if is_dynamic_array(field)]@
      @(sanitize_identifier(field.name))__len: size_t,
@[    end if]@
@[    if not is_nested(field)]@
      @(sanitize_identifier(field.name)): @(get_ffi_type(field, package_name)),
@[    end if]@
@[end for]@
    );
@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle: uintptr_t);
@
@[for field in msg_spec.fields]@
@[    if is_dynamic_array(field)]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_array_size(message_handle: uintptr_t) -> size_t;
@[    end if]@
@
@[    if is_nested_array(field) or is_string_array(field)]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t, item_handles: *mut uintptr_t);
@[    else]@
    fn @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle: uintptr_t) -> @(get_ffi_return_type(field, package_name));
@[    end if]@
@[end for]@
}

#[allow(non_snake_case)]
impl @(type_name) {
  fn get_native_message(&self) -> uintptr_t {
@[for field in msg_spec.fields]@
@[    if is_string_array(field)]@
    let @(sanitize_identifier(field.name))__c_strings = self.@(sanitize_identifier(field.name)).iter().map(|s| CString::new(s.clone()).unwrap()).collect::<Vec<_>>();
@[    elif is_single_string(field)]@
    let @(sanitize_identifier(field.name))__c_string = CString::new(self.@(sanitize_identifier(field.name)).clone()).unwrap();
@[    end if]@
@[end for]@
@
    unsafe {
      let native_message = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_create_native_message();
      self.get_native_message_at(native_message);
      native_message
    }
  }

  pub fn get_native_message_at(&self, native_message: uintptr_t) {
@[for field in msg_spec.fields]@
@[    if is_string_array(field)]@
    let @(sanitize_identifier(field.name))__c_strings = self.@(sanitize_identifier(field.name)).iter().map(|s| CString::new(s.clone()).unwrap()).collect::<Vec<_>>();
@[    elif is_single_string(field)]@
    let @(sanitize_identifier(field.name))__c_string = CString::new(self.@(sanitize_identifier(field.name)).clone()).unwrap();
@[    end if]@
@[end for]@
@
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message_at(
        native_message@("," if len(msg_spec.fields) else "")
@[for field in msg_spec.fields]@
@
@[    if is_dynamic_array(field)]@
        self.@(sanitize_identifier(field.name)).len() as size_t,
@[    end if]@
@
@[    if is_string_array(field)]@
        @(sanitize_identifier(field.name))__c_strings.iter().map(|c_string| c_string.as_ptr()).collect::<Vec<_>>().as_ptr(),
@[    elif is_primitive_array(field)]@
        self.@(sanitize_identifier(field.name)).as_ptr(),
@[    elif is_single_string(field)]@
        @(sanitize_identifier(field.name))__c_string.as_ptr(),
@[    elif is_primitive(field)]@
        self.@(sanitize_identifier(field.name)),
@[    end if]@
@[end for]@
      );
@

@[for field in msg_spec.fields]@
@[    if is_static_nested_array(field)]@
      let mut @(sanitize_identifier(field.name))__item_handles : [uintptr_t; @(field.type.array_size)] = [0; @(field.type.array_size)];
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(native_message, @(sanitize_identifier(field.name))__item_handles.as_mut_ptr());
@[        for i in range(field.type.array_size)]@
      self.@(sanitize_identifier(field.name))[@(i)].get_native_message_at(@(sanitize_identifier(field.name))__item_handles[@(i)]);
@[        end for]@
@[    elif is_dynamic_nested_array(field)]@
      let mut @(sanitize_identifier(field.name))__item_handles = Vec::with_capacity(self.@(sanitize_identifier(field.name)).len());
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(native_message, @(sanitize_identifier(field.name))__item_handles.as_mut_ptr());
      @(sanitize_identifier(field.name))__item_handles.set_len(self.@(sanitize_identifier(field.name)).len());
      for i in 0..self.@(sanitize_identifier(field.name)).len() {
        self.@(sanitize_identifier(field.name))[i].get_native_message_at(@(sanitize_identifier(field.name))__item_handles[i]);
      }
@[    elif is_single_nested(field)]@
      let @(sanitize_identifier(field.name))__handle = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(native_message);
      self.@(sanitize_identifier(field.name)).get_native_message_at(@(sanitize_identifier(field.name))__handle);
@[    end if]@
@[end for]@
    }
  }

  fn destroy_native_message(&self, message_handle: uintptr_t) {
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle);
    }
  }

@[if msg_spec.fields]@
  fn read_handle(&mut self, message_handle: uintptr_t) {
    unsafe {
@[    for field in msg_spec.fields]@
@
@[        if is_static_string_array(field)]@
      let mut item_handles : [uintptr_t; @(field.type.array_size)] = [0; @(field.type.array_size)];
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle, item_handles.as_mut_ptr());
@[            for i in range(field.type.array_size)]@
      self.@(sanitize_identifier(field.name))[@(i)] = CStr::from_ptr(item_handles[@(i)] as *const c_char).to_string_lossy().into_owned();
@[            end for]
@
@[        elif is_static_primitive_array(field)]@
      let @(field.name)_array = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
      self.@(sanitize_identifier(field.name)).clone_from_slice(std::slice::from_raw_parts(@(field.name)_array as *const _, @(field.type.array_size)));
@
@[        elif is_static_nested_array(field)]@
      let mut item_handles : [uintptr_t; @(field.type.array_size)] = [0; @(field.type.array_size)];
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle, item_handles.as_mut_ptr());
@[            for i in range(field.type.array_size)]@
      self.@(sanitize_identifier(field.name))[@(i)].read_handle(item_handles[@(i)]);
@[            end for]
@
@[        elif is_dynamic_string_array(field)]@
      let @(field.name)_size = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_array_size(message_handle);
      let mut item_handles = Vec::with_capacity(@(field.name)_size);
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle, item_handles.as_mut_ptr());
      item_handles.set_len(@(field.name)_size);
      for item_handle in item_handles.into_iter() {
        self.@(sanitize_identifier(field.name)).push(CStr::from_ptr(item_handle as *const c_char).to_string_lossy().into_owned());
      }
@
@[        elif is_dynamic_primitive_array(field)]@
      let @(field.name)_size = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_array_size(message_handle);
      let @(field.name)_array = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
      self.@(sanitize_identifier(field.name)) = Vec::with_capacity(@(field.name)_size);
      self.@(sanitize_identifier(field.name)).set_len(@(field.name)_size);
      self.@(sanitize_identifier(field.name)).clone_from_slice(std::slice::from_raw_parts(@(field.name)_array, @(field.name)_size));
@
@[        elif is_dynamic_nested_array(field)]@
      let @(field.name)_size = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_array_size(message_handle);
      let mut item_handles = Vec::with_capacity(@(field.name)_size);
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle, item_handles.as_mut_ptr());
      item_handles.set_len(@(field.name)_size);
      for item_handle in item_handles.into_iter() {
        let mut item = @get_non_array_rs_type(field.type, package_name)::default();
        item.read_handle(item_handle);
        self.@(sanitize_identifier(field.name)).push(item);
      }
@
@[        elif is_single_string(field)]@
      let ptr = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
      self.@(sanitize_identifier(field.name)) = CStr::from_ptr(ptr).to_string_lossy().into_owned();
@
@[        elif is_primitive(field)]@
      self.@(sanitize_identifier(field.name)) = @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle);
@
@[        else]@
      self.@(sanitize_identifier(field.name)).read_handle(@(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(message_handle));
@[        end if]@
@[    end for]@
    }
@[else]@
  fn read_handle(&mut self, _message_handle: uintptr_t) {
@[end if]@
  }
}

impl rclrs_common::traits::Message for @(type_name) {
  fn get_native_message(&self) -> uintptr_t {
    self.get_native_message()
  }

  fn destroy_native_message(&self, message_handle: uintptr_t) {
    self.destroy_native_message(message_handle);
  }

  fn read_handle(&mut self, message_handle: uintptr_t) {
    self.read_handle(message_handle);
  }
}

impl rclrs_common::traits::MessageDefinition<@(type_name)> for @(type_name) {
  fn get_type_support() -> uintptr_t {
    unsafe { @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() }
  }

  fn static_get_native_message(message: &@(type_name)) -> uintptr_t {
    message.get_native_message()
  }

  fn static_destroy_native_message(message_handle: uintptr_t) {
    unsafe {
      @(package_name)_@(subfolder)_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(message_handle);
    }
  }
}

@[end for]
@
@{
for key in sorted(includes.keys()):
    print('use %s;  // %s' % (key, ', '.join(includes[key])))
}@