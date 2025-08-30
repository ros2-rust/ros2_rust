use std::default;

use proc_macro2::TokenStream;
use quote::quote;
use syn::{Data, DeriveInput, Expr, Lit, Meta};

pub(crate) fn derive_struct_parameters(input: DeriveInput) -> syn::Result<TokenStream> {
    let ident = input.ident;

    let fields = match input.data {
        syn::Data::Struct(ref s) => &s.fields,
        _ => {
            return syn::Result::Err(syn::Error::new_spanned(
                ident,
                "StrucutredParameter trait can only be derived for structs",
            ));
        }
    };

    let field_types: Vec<_> = fields.iter().map(|f| &f.ty).collect();

    let mut args = Vec::new();
    for f in fields {
        let ident = f.ident.as_ref().unwrap();
        let ident_str = syn::LitStr::new(&f.ident.as_ref().unwrap().to_string(), ident.span());

        let mut default: Option<Expr> = None;

        for attr in &f.attrs {
            if attr.path().is_ident("param") {
                attr.parse_nested_meta(|meta| {
                    if meta.path.is_ident("default") {
                        default = Some(meta.value()?.parse()?);
                        Ok(())
                    } else {
                        syn::Result::Err(syn::Error::new_spanned(meta.path, "Unknown key."))
                    }
                })?;
            }
        }

        let default = match default {
            Some(expr) => quote! {Some(#expr)},
            None => quote! {None},
        };

        let field_type = match &f.ty {
            syn::Type::Path(p) => {
                let mut p = p.path.clone();
                for segment in &mut p.segments {
                    segment.arguments = syn::PathArguments::None;
                }
                p
            }
            e => {
                return syn::Result::Err(syn::Error::new_spanned(
                    e,
                    "only PathType attributes are supported.",
                ));
            }
        };
        let r = quote! {
           #ident : #field_type::declare_structured(
              node,
              rclrs::parameter::structured::ParameterOptions {
                name: &{match options.name {
                    "" => #ident_str.to_string(),
                    prefix => [prefix, ".", #ident_str].concat(),
                }},
                default: #default,
              }
          )?,
        };
        args.push(r);
    }

    let result = quote!(
      impl #ident {
            const _ASSERT_PARAMETER: fn() = || {
                fn assert_parameter<T: rclrs::parameter::structured::StructuredParameters>() {}
                #(
                  assert_parameter::<#field_types>();
                )*
            };
      }
      impl rclrs::parameter::structured::StructuredParametersMeta<rclrs::parameter::structured::DefaultForbidden> for #ident {
        fn declare_structured_(node: &rclrs::NodeState, options: rclrs::parameter::structured::ParameterOptions<rclrs::parameter::structured::DefaultForbidden>)
          -> core::result::Result<Self, crate::DeclarationError> {
              core::result::Result::Ok(Self{ #(#args)*})
          }

      }
      impl rclrs::StructuredParameters for #ident {}
    );
    syn::Result::Ok(result)
}
