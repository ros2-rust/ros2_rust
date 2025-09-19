use proc_macro2::TokenStream;
use quote::quote;
use syn::{token::Token, DeriveInput, Expr};

pub(crate) fn derive_structured_parameters(input: DeriveInput) -> syn::Result<TokenStream> {
    let ident = input.ident;

    match input.data {
        syn::Data::Struct(ref s) => derive_structured_parameters_struct(ident, s),
        _ => {
            return syn::Result::Err(syn::Error::new_spanned(
                ident,
                "StructuredParameters trait can only be derived for structs",
            ));
        }
    }
}

fn derive_structured_parameters_struct(
    ident: proc_macro2::Ident,
    struct_: &syn::DataStruct,
) -> syn::Result<TokenStream> {
    let fields = &struct_.fields;

    let field_types: Vec<_> = fields.iter().map(|f| &f.ty).collect();
    let mut args = Vec::new();
    for f in fields {
        let ident = f.ident.as_ref().unwrap();
        let ident_str = syn::LitStr::new(&f.ident.as_ref().unwrap().to_string(), ident.span());

        let mut default: Option<Expr> = None;
        let mut description: Option<Expr> = None;
        let mut constraints: Option<Expr> = None;
        let mut ignore_override = false;
        let mut discard_mismatching_prior_value = false;
        let mut discriminate: Option<Expr> = None;

        for attr in &f.attrs {
            if attr.path().is_ident("param") {
                attr.parse_nested_meta(|meta| {
                    if meta.path.is_ident("default") {
                        default = Some(meta.value()?.parse()?);
                        Ok(())
                    } else if meta.path.is_ident("description") {
                        description = Some(meta.value()?.parse()?);
                        Ok(())
                    } else if meta.path.is_ident("constraints") {
                        constraints = Some(meta.value()?.parse()?);
                        Ok(())
                    } else if meta.path.is_ident("ignore_override") {
                        ignore_override = true;
                        Ok(())
                    } else if meta.path.is_ident("discard_mismatching_prior_value") {
                        discard_mismatching_prior_value = true;
                        Ok(())
                    } else if meta.path.is_ident("discriminate") {
                        discriminate = Some(meta.value()?.parse()?);
                        Ok(())
                    } else {
                        let err = format!("Unknown key: {:?}", &meta.path.get_ident());
                        syn::Result::Err(syn::Error::new_spanned(meta.path, err))
                    }
                })?;
            }
        }

        let default = match default {
            Some(expr) => quote! {Some(#expr)},
            None => quote! {None},
        };
        let description = match description {
            Some(expr) => quote! {#expr},
            None => quote! {""},
        };
        let constraints = match constraints {
            Some(expr) => quote! {#expr},
            None => quote! {""},
        };
        let discriminate = match discriminate {
            Some(expr) => quote! {
                core::option::Option::Some(Box::new(#expr))
            },
            None => quote! {core::option::Option::None},
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
                    "Only PathType attributes are supported",
                ));
            }
        };
        let r = quote! {
           #ident : #field_type::declare_structured(
              node,
              &{match name {
                    "" => #ident_str.to_string(),
                    prefix => [prefix, ".", #ident_str].concat(),
              }},
              #default,
              #description,
              #constraints,
              #ignore_override,
              #discard_mismatching_prior_value,
              #discriminate,

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
        fn declare_structured_(
          node: &rclrs::NodeState,
          name: &str,
          default: core::option::Option<rclrs::structured::DefaultForbidden>,
          description: impl core::convert::Into<std::sync::Arc<str>>,
          constraints: impl core::convert::Into<std::sync::Arc<str>>,
          ignore_override: bool,
          discard_mismatching_prior_value: bool,
          discriminate: core::option::Option<Box<dyn core::ops::function::FnOnce(rclrs::AvailableValues<T>) -> core::option::Option<T>>>,
        )
          -> core::result::Result<Self, rclrs::DeclarationError> {
              core::result::Result::Ok(Self{ #(#args)*})
          }

      }
      impl rclrs::StructuredParameters for #ident {}
    );
    syn::Result::Ok(result)
}
