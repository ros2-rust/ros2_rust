use proc_macro2::TokenStream;
use quote::quote;
use syn::DeriveInput;

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
                    "attribute can only be path type",
                ));
            }
        };
        let r = quote! {
           #ident : #field_type::declare_structured(
              node, &{match name {
                "" => #ident_str.to_string(),
                prefix => [prefix, ".", #ident_str].concat(),
              }
           })?,
        };
        args.push(r);
    }

    let result = quote!(
      impl #ident {
            const _ASSERT_PARAMETER: fn() = || {
                fn assert_parameter<T: rclrs::StructuredParameters>() {}
                #(
                  assert_parameter::<#field_types>();
                )*
            };
        }

      impl rclrs::StructuredParameters for #ident {
        fn declare_structured(node: &rclrs::NodeState, name: &str) -> core::result::Result<Self, rclrs::DeclarationError> {
          core::result::Result::Ok(Self{ #(#args)*})
        }
      }
    );
    syn::Result::Ok(result)
}
