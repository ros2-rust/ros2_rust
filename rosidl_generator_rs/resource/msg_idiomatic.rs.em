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

@# #################################################
@# ############ Idiomatic message types ############
@# #################################################
@# These types use standard Rust containers where possible.
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
}@

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct @(type_name) {
@[for member in msg_spec.structure.members]@
    @(pre_field_serde(member.type))pub @(get_rs_name(member.name)): @(get_idiomatic_rs_type(member.type)),
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
@#  This has the benefit of automatically setting the right default values
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::@(subfolder)::rmw::@(type_name)::default())
  }
}

impl rosidl_runtime_rs::Message for @(type_name) {
  type RmwMsg = crate::@(subfolder)::rmw::@(type_name);

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
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
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
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
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
@[        if isinstance(member.type.value_type, UnboundedString) or isinstance(member.type.value_type, UnboundedWString)]@
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
