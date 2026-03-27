{
  description = "ros flake";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    { self, nixpkgs, rust-overlay, flake-utils, nix-ros-overlay, ... }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (
      system:
      let
        overlays = [ (import rust-overlay) nix-ros-overlay.overlays.default ];

        pkgs = import nixpkgs {
          inherit system overlays;
        };

        rosDistro = pkgs.rosPackages.jazzy;
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" ];
        };

        rosPrefixes = "${rosDistro.ament-cmake}:${rosDistro.ament-cmake-core}:${rosDistro.python-cmake-module}:${rosDistro.rmw}:${rosDistro.rosidl-default-generators}:${rosDistro.rosidl-runtime-c}:${rosDistro.rosidl-typesupport-c}:${rosDistro.rosidl-typesupport-interface}:${rosDistro.std-msgs}:${rosDistro.test-msgs}";
      in
      {
        devShells.default = pkgs.mkShell {
          packages =
            [
              rustToolchain
              pkgs.gcc
              pkgs.clang
              pkgs.cmake
              pkgs.mold
              pkgs.pkg-config
              pkgs.colcon
              (with rosDistro; buildEnv {
                paths = [
                  ros-core
                  ros-base
                  cyclonedds
                  rmw-cyclonedds-cpp
                  test-msgs
                ];
              })
            ];

          RUST_SRC_PATH = "${rustToolchain}/lib/rustlib/src/rust/library";

          shellHook = ''
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            export CC=clang
            export CXX=clang++

            dedup_flags() {
                local var_name="$1"
                local current_value
                current_value="$(printenv "$var_name" 2>/dev/null || true)"

                if [ -z "$current_value" ]; then
                    return
                fi

                export "$var_name=$(
                    printf '%s' "$current_value" \
                        | tr ' ' '\n' \
                        | awk '
                            function flush_entry(entry_key, entry_value) {
                                if (!(entry_key in seen)) {
                                    seen[entry_key] = 1
                                    print entry_value
                                }
                            }

                            BEGIN {
                                expects_value["-I"] = 1
                                expects_value["-L"] = 1
                                expects_value["-B"] = 1
                                expects_value["-include"] = 1
                                expects_value["-iquote"] = 1
                                expects_value["-idirafter"] = 1
                                expects_value["-isystem"] = 1
                                expects_value["-isysroot"] = 1
                                expects_value["-iframework"] = 1
                            }

                            NF == 0 {
                                next
                            }

                            pending_option != "" {
                                flush_entry(pending_option SUBSEP $0, pending_option " " $0)
                                pending_option = ""
                                next
                            }

                            {
                                if ($0 in expects_value) {
                                    pending_option = $0
                                    next
                                }

                                flush_entry($0, $0)
                            }

                            END {
                                if (pending_option != "") {
                                    flush_entry(pending_option, pending_option)
                                }
                            }
                        ' \
                        | paste -sd ' ' -
                )"
            }

            dedup_flags NIX_CFLAGS_COMPILE
            dedup_flags NIX_CXXFLAGS_COMPILE
            dedup_flags NIX_LDFLAGS

            prepend_prefixes() {
                local var_name="$1"
                local existing_value

                existing_value="$(printenv "$var_name" 2>/dev/null || true)"
                if [ -n "$existing_value" ]; then
                    export "$var_name=${rosPrefixes}:$existing_value"
                else
                    export "$var_name=${rosPrefixes}"
                fi
            }

            prepend_prefixes CMAKE_PREFIX_PATH

            setup_synthetic_ament_prefix() {
                local base_dir
                local synthetic_prefix
                local source_prefixes
                local old_ifs="$IFS"
                local prefix
                local category_dir
                local resource
                local existing_value
                local filtered=""

                if [ -n "''${XDG_CACHE_HOME:-}" ]; then
                    base_dir="$XDG_CACHE_HOME"
                elif [ -n "''${TMPDIR:-}" ]; then
                    base_dir="$TMPDIR"
                else
                    base_dir="/tmp"
                fi

                synthetic_prefix="$base_dir/ros2-rust-nix-ament-prefix-${system}"
                mkdir -p "$synthetic_prefix/share/ament_index/resource_index/packages"

                printf '%s\n' '#!/usr/bin/env sh' > "$synthetic_prefix/local_setup.sh"
                printf '%s\n' '#!/usr/bin/env bash' > "$synthetic_prefix/local_setup.bash"
                printf '%s\n' '#!/usr/bin/env zsh' > "$synthetic_prefix/local_setup.zsh"
                chmod +x \
                    "$synthetic_prefix/local_setup.sh" \
                    "$synthetic_prefix/local_setup.bash" \
                    "$synthetic_prefix/local_setup.zsh"

                find "$synthetic_prefix/share/ament_index/resource_index" -mindepth 1 -delete

                source_prefixes="$(printenv CMAKE_PREFIX_PATH 2>/dev/null || true)"
                IFS=:
                for prefix in $source_prefixes; do
                    if [ ! -d "$prefix/share/ament_index/resource_index" ]; then
                        continue
                    fi

                    for category_dir in "$prefix"/share/ament_index/resource_index/*; do
                        if [ ! -d "$category_dir" ]; then
                            continue
                        fi

                        mkdir -p "$synthetic_prefix/share/ament_index/resource_index/$(basename "$category_dir")"
                        for resource in "$category_dir"/*; do
                            if [ ! -f "$resource" ]; then
                                continue
                            fi
                            ln -sf "$resource" \
                                "$synthetic_prefix/share/ament_index/resource_index/$(basename "$category_dir")/$(basename "$resource")"
                        done
                    done
                done
                IFS="$old_ifs"

                existing_value="$(printenv AMENT_PREFIX_PATH 2>/dev/null || true)"
                if [ -n "$existing_value" ]; then
                    IFS=:
                    for prefix in $existing_value; do
                        if [ -f "$prefix/local_setup.sh" ] || [ -f "$prefix/local_setup.bash" ] || [ -f "$prefix/local_setup.zsh" ]; then
                            if [ -n "$filtered" ]; then
                                filtered="$filtered:$prefix"
                            else
                                filtered="$prefix"
                            fi
                        fi
                    done
                    IFS="$old_ifs"
                fi

                if [ -n "$filtered" ]; then
                    export AMENT_PREFIX_PATH="$synthetic_prefix:$filtered"
                else
                    export AMENT_PREFIX_PATH="$synthetic_prefix"
                fi
            }

            setup_synthetic_ament_prefix
          '';
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
