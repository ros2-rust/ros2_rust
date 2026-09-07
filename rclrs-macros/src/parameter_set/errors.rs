//! Collecting more than one error per expansion.
//!
//! A struct with several mistakes in it should report all of them, rather than making the user
//! fix them one compile at a time.

/// Accumulates errors, combining them into a single [`syn::Error`] that expands to one
/// `compile_error!` per problem.
#[derive(Default)]
pub(crate) struct Errors(Option<syn::Error>);

impl Errors {
    /// Records an error against `spanned`'s source location.
    pub fn at(&mut self, spanned: impl syn::spanned::Spanned, message: impl std::fmt::Display) {
        self.push(syn::Error::new(spanned.span(), message));
    }

    /// Records an already-constructed error.
    pub fn push(&mut self, error: syn::Error) {
        match &mut self.0 {
            Some(existing) => existing.combine(error),
            none => *none = Some(error),
        }
    }

    /// Records an error if `result` is one, and returns whatever value it held.
    pub fn handle<T>(&mut self, result: syn::Result<T>) -> Option<T> {
        match result {
            Ok(value) => Some(value),
            Err(error) => {
                self.push(error);
                None
            }
        }
    }

    /// Returns the accumulated errors, if any.
    pub fn into_result(self) -> syn::Result<()> {
        match self.0 {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }
}
