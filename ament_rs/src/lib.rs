//! `ament_rs` provides utilities for querying the ament resource index used by ROS 2 packages.
//!
//! # Examples
//!
//! ```
//! # fn main() -> Result<(), std::env::VarError> {
//! # std::env::set_var("AMENT_PREFIX_PATH", "");
//! println!("{:#?}", ament_rs::packages()?);
//! # Ok(())
//! # }
//! ```
//!
//! This snippet will print a list of packages with the prefixes they were found in, depending of the value of the `AMENT_PREFIX_PATH` environment variable on your system.
//!
//! ```text
//! {
//!     "ros_core": [
//!         "/opt/ros/rolling",
//!     ],
//!     "rcl_interfaces": [
//!         "/your/workspace/install/rcl_interfaces",
//!         "/opt/ros/rolling",
//!     ],
//!     ...
//! }
//! ```
//!
//! For more information:
//! - Design doc: <https://github.com/ament/ament_cmake/blob/2366f15479e37d552d4e225f09ccef1c6ccc8c4e/ament_cmake_core/doc/resource_index.md>

use std::{
    collections::HashMap,
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
};

/// This constant defines the name of the environment variable containing the list of ament resource index prefixes, which is `AMENT_PREFIX_PATH`
pub const AMENT_PREFIX_PATH_ENV_VAR: &str = "AMENT_PREFIX_PATH";

/// Returns the list of prefixes defined in the `AMENT_PREFIX_PATH` environment variable.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn prefixes() -> Result<Vec<PathBuf>, std::env::VarError> {
    prefixes_from_env_var(AMENT_PREFIX_PATH_ENV_VAR)
}

/// Returns the list of prefixes defined in the given environment variable.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the given environment variable is not set.
pub fn prefixes_from_env_var(env_var: &str) -> Result<Vec<PathBuf>, std::env::VarError> {
    Ok(std::env::split_paths(&std::env::var(env_var)?).collect())
}

/// Returns all resources of the given type found under the provided ament prefixes.
///
/// Each resource name is mapped to the list of prefixes that contain a matching
/// resource marker file.
pub fn resources_in(resource_type: &str, prefixes: &[PathBuf]) -> HashMap<String, Vec<PathBuf>> {
    prefixes.iter().fold(
        HashMap::new(),
        |mut map: HashMap<String, Vec<PathBuf>>, path| {
            let resource_index_path = path
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join(resource_type);

            if let Ok(dirs) = fs::read_dir(resource_index_path) {
                dirs.into_iter()
                    .flatten()
                    .filter(|entry| entry.path().is_file())
                    .for_each(|e| {
                        map.entry(e.file_name().to_string_lossy().into_owned())
                            .or_default()
                            .push(path.to_owned())
                    });
            }

            map
        },
    )
}

/// Returns all resources of the given type found in the prefixes from `AMENT_PREFIX_PATH`.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn resources(resource_type: &str) -> Result<HashMap<String, Vec<PathBuf>>, std::env::VarError> {
    Ok(resources_in(resource_type, &prefixes()?))
}

/// Finds a resource by name and type under the provided ament prefixes.
///
/// Returns the list of prefixes containing the resource, or `None` if no matching
/// resource marker file is found.
pub fn find_resource_in(
    resource_name: &str,
    resource_type: &str,
    prefixes: &[PathBuf],
) -> Option<Vec<PathBuf>> {
    resources_in(resource_type, prefixes)
        .get(resource_name)
        .cloned()
}

/// Finds a resource by name and type using the prefixes from `AMENT_PREFIX_PATH`.
///
/// Returns the list of prefixes containing the resource, or `None` if no matching
/// resource marker file is found.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn find_resource(
    resource_name: &str,
    resource_type: &str,
) -> Result<Option<Vec<PathBuf>>, std::env::VarError> {
    Ok(find_resource_in(resource_name, resource_type, &prefixes()?))
}

/// Returns all packages found under the provided ament prefixes.
pub fn packages_in(prefixes: &[PathBuf]) -> HashMap<String, Vec<PathBuf>> {
    resources_in("packages", prefixes)
}

/// Returns all packages found in the prefixes from `AMENT_PREFIX_PATH`.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn packages() -> Result<HashMap<String, Vec<PathBuf>>, std::env::VarError> {
    Ok(packages_in(&prefixes()?))
}

/// Finds a package under the provided ament prefixes.
///
/// Returns the list of prefixes containing the package, or `None` if the package is not found.
pub fn find_package_in(package: &str, prefixes: &[PathBuf]) -> Option<Vec<PathBuf>> {
    find_resource_in(package, "packages", prefixes)
}

/// Finds a package using the prefixes from `AMENT_PREFIX_PATH`.
///
/// Returns `None` if the package is not found.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn find_package(package: &str) -> Result<Option<Vec<PathBuf>>, std::env::VarError> {
    Ok(find_package_in(package, &prefixes()?))
}

/// Returns the share directories for a package under the provided ament prefixes.
///
/// The returned paths are formed by appending `share/<package>` to each prefix
/// where the package was found.
pub fn package_share_dirs_in(package: &str, prefixes: &[PathBuf]) -> Option<Vec<PathBuf>> {
    find_package_in(package, prefixes).map(|paths| {
        paths
            .iter()
            .map(|path| path.join("share").join(package))
            .collect()
    })
}

/// Returns the share directories for a package using prefixes from `AMENT_PREFIX_PATH`.
///
/// Returns `None` if the package is not found.
///
/// # Errors
///
/// A `std::env::VarError` is returned if the `AMENT_PREFIX_PATH` environment variable is not set.
pub fn package_share_dirs(package: &str) -> Result<Option<Vec<PathBuf>>, std::env::VarError> {
    Ok(package_share_dirs_in(package, &prefixes()?))
}

/// Register a package resource of a specific type with the ament index.
///
/// # Errors
///
/// A `io::Error` if the resource file cannot be created
pub fn register_resource(
    install_base: impl AsRef<Path>,
    resource_type: &str,
    package_name: &str,
    content: &str,
) -> Result<(), io::Error> {
    let resource_dir = install_base
        .as_ref()
        .join("share")
        .join("ament_index")
        .join("resource_index")
        .join(resource_type);
    fs::create_dir_all(&resource_dir)?;
    let resource_file_path = resource_dir.join(package_name);
    let mut file = File::create(&resource_file_path)?;
    file.write_all(content.as_bytes())?;
    Ok(())
}

/// Register a package name with the ament index's `packages` resource type. Equivalent of CMake's `ament_index_register_package()`.
///
/// # Errors
///
/// A `io::Error` if the resource file cannot be created
pub fn register_package(
    install_base: impl AsRef<Path>,
    package_name: &str,
) -> Result<(), io::Error> {
    register_resource(install_base, "packages", package_name, "")
}

#[cfg(test)]
mod tests {
    use super::*;
    use itertools::Itertools;

    struct AmentIndexTestFixture<'a> {
        manifest: Vec<(&'a str, Vec<&'a str>)>,
        pub fs: tempfile::TempDir,
    }

    impl<'a> AmentIndexTestFixture<'a> {
        pub fn new(manifest: Vec<(&'a str, Vec<&'a str>)>) -> Self {
            let fs = tempfile::TempDir::new().unwrap();

            for (package, paths) in &manifest {
                for path in paths {
                    std::fs::create_dir_all(
                        fs.path()
                            .join(path)
                            .join("share")
                            .join("ament_index")
                            .join("resource_index")
                            .join("packages"),
                    )
                    .unwrap();

                    std::fs::File::create(
                        fs.path()
                            .join(path)
                            .join("share")
                            .join("ament_index")
                            .join("resource_index")
                            .join("packages")
                            .join(package),
                    )
                    .unwrap();
                }
            }

            Self { manifest, fs }
        }

        pub fn prefixes(&self) -> Vec<PathBuf> {
            self.manifest
                .iter()
                .flat_map(|(_, paths)| paths)
                .map(|path| self.fs.path().join(path))
                .unique()
                .collect()
        }

        pub fn expected(&self) -> HashMap<String, Vec<PathBuf>> {
            self.manifest
                .iter()
                .map(|(package, paths)| {
                    (
                        package.to_string(),
                        paths.iter().map(|path| self.fs.path().join(path)).collect(),
                    )
                })
                .collect()
        }
    }

    #[test]
    fn empty_prefixes() {
        assert!(packages_in(&[]).is_empty());
    }

    #[test]
    fn multiple_packages_and_prefixes() {
        let fixture = AmentIndexTestFixture::new(vec![
            (
                "common_interfaces",
                vec!["my/workspace/install/common_interfaces", "opt/ros/rolling"],
            ),
            ("ros_core", vec!["opt/ros/rolling"]),
        ]);

        assert_eq!(packages_in(&fixture.prefixes()), fixture.expected());
    }

    #[test]
    fn package_prefixes_preserve_input_prefix_order() {
        let fixture = AmentIndexTestFixture::new(vec![(
            "shared_package",
            vec!["second_prefix", "first_prefix", "third_prefix"],
        )]);

        let packages = packages_in(&fixture.prefixes());

        let shared_package = packages.get("shared_package").unwrap();
        assert_eq!(
            shared_package[0].iter().next_back().unwrap().to_str(),
            Some("second_prefix")
        );
        assert_eq!(
            shared_package[1].iter().next_back().unwrap().to_str(),
            Some("first_prefix")
        );
        assert_eq!(
            shared_package[2].iter().next_back().unwrap().to_str(),
            Some("third_prefix")
        );
    }

    #[test]
    fn find_nonexistent_package() {
        assert_eq!(find_package_in("nonexistent_package", &[]), None);
    }

    #[test]
    fn find_real_package() {
        let fixture = AmentIndexTestFixture::new(vec![
            (
                "common_interfaces",
                vec!["my/workspace/install/common_interfaces", "opt/ros/rolling"],
            ),
            ("ros_core", vec!["opt/ros/rolling"]),
        ]);

        let packages = find_package_in("common_interfaces", &fixture.prefixes()).unwrap();

        let expected = fixture.expected().get("common_interfaces").unwrap().clone();

        assert_eq!(packages, expected);
    }

    #[test]
    fn find_nonexistent_package_share_dirs() {
        let fixture = AmentIndexTestFixture::new(vec![]);

        let share_dirs = package_share_dirs_in("common_interfaces", &fixture.prefixes());

        assert_eq!(share_dirs, None);
    }

    #[test]
    fn test_package_share_dirs() {
        let fixture = AmentIndexTestFixture::new(vec![(
            "common_interfaces",
            vec!["my/workspace/install/common_interfaces", "opt/ros/rolling"],
        )]);

        let share_dirs = package_share_dirs_in("common_interfaces", &fixture.prefixes()).unwrap();

        assert_eq!(
            share_dirs,
            vec![
                fixture
                    .fs
                    .path()
                    .join("my/workspace/install/common_interfaces/share/common_interfaces"),
                fixture
                    .fs
                    .path()
                    .join("opt/ros/rolling/share/common_interfaces"),
            ]
        );
    }

    #[test]
    fn find_real_resources() {
        let fixture = AmentIndexTestFixture::new(vec![
            (
                "common_interfaces",
                vec!["my/workspace/install/common_interfaces", "opt/ros/rolling"],
            ),
            ("ros_core", vec!["opt/ros/rolling"]),
        ]);

        let resources = resources_in("packages", &fixture.prefixes());

        assert_eq!(resources, fixture.expected());

        let common_interfaces =
            find_resource_in("common_interfaces", "packages", &fixture.prefixes()).unwrap();

        assert_eq!(
            common_interfaces,
            vec![
                fixture
                    .fs
                    .path()
                    .join("my/workspace/install/common_interfaces"),
                fixture.fs.path().join("opt/ros/rolling"),
            ]
        );
    }

    #[test]
    fn find_package_matches_find_package_in_with_prefixes_from_env() {
        let fixture = AmentIndexTestFixture::new(vec![(
            "rcl_interfaces",
            vec!["my/workspace/install/rcl_interfaces", "opt/ros/rolling"],
        )]);

        let prefixes = fixture.prefixes();
        std::env::set_var(
            AMENT_PREFIX_PATH_ENV_VAR,
            std::env::join_paths(&prefixes).unwrap(),
        );

        assert_eq!(
            find_package_in("rcl_interfaces", &prefixes),
            find_package("rcl_interfaces").unwrap()
        );
    }

    #[test]
    fn resources_ignore_directories() {
        let fs = tempfile::TempDir::new().unwrap();

        std::fs::create_dir_all(
            fs.path()
                .join("prefix")
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("packages")
                .join("not_a_package"),
        )
        .unwrap();

        let prefixes = vec![fs.path().join("prefix")];

        assert!(packages_in(&prefixes).is_empty());
        assert_eq!(find_package_in("not_a_package", &prefixes), None);
    }

    #[test]
    fn resources_support_arbitrary_resource_types() {
        let fs = tempfile::TempDir::new().unwrap();

        std::fs::create_dir_all(
            fs.path()
                .join("prefix")
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("plugins"),
        )
        .unwrap();

        std::fs::File::create(
            fs.path()
                .join("prefix")
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("plugins")
                .join("my_plugin"),
        )
        .unwrap();

        let prefixes = vec![fs.path().join("prefix")];

        let mut expected = HashMap::new();
        expected.insert("my_plugin".to_string(), vec![fs.path().join("prefix")]);

        assert_eq!(resources_in("plugins", &prefixes), expected);
        assert_eq!(
            find_resource_in("my_plugin", "plugins", &prefixes),
            Some(vec![fs.path().join("prefix")])
        );
    }

    #[test]
    fn test_register_resource() -> Result<(), Box<dyn std::error::Error>> {
        let tmp = tempfile::TempDir::new()?;
        let install_base = tmp.path();

        register_resource(install_base, "packages", "test_package", "")?;

        let marker_path =
            install_base.join("share/ament_index/resource_index/packages/test_package");

        assert!(marker_path.exists());
        assert!(marker_path.is_file());
        assert_eq!(fs::read_to_string(&marker_path)?, "");
        Ok(())
    }

    #[test]
    fn test_register_resource_with_content() -> Result<(), Box<dyn std::error::Error>> {
        let tmp = tempfile::TempDir::new()?;
        let install_base = tmp.path();

        register_resource(
            install_base,
            "test_resource",
            "test_package",
            "test_resource/foo.yaml;test_resource/bar.yaml",
        )?;

        let marker_path =
            install_base.join("share/ament_index/resource_index/test_resource/test_package");

        assert_eq!(
            fs::read_to_string(&marker_path)?,
            "test_resource/foo.yaml;test_resource/bar.yaml"
        );
        Ok(())
    }

    #[test]
    fn test_register_package() -> Result<(), Box<dyn std::error::Error>> {
        let tmp = tempfile::TempDir::new()?;
        let install_base = tmp.path();

        register_package(install_base, "test_package")?;

        let marker_path =
            install_base.join("share/ament_index/resource_index/packages/test_package");

        assert_eq!(fs::read_to_string(&marker_path)?, "");
        Ok(())
    }
}
