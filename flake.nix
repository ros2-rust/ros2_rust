{
  description = "ROS 2 Rust client library (rclrs)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, rust-overlay, ... }:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;

      pkgsFor = system: import nixpkgs {
        inherit system;
        overlays = [ rust-overlay.overlays.default ];
      };
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = pkgsFor system;

          rustToolchain = pkgs.rust-bin.stable.latest.default.override {
            extensions = [ "rust-src" "rust-analyzer" "clippy" ];
          };
        in
        {
          default = let
            # Wrapper script that adds --features use_ros_shim to cargo commands
            cargoWrapper = pkgs.writeShellScriptBin "cargo" ''
              args=("$@")
              cmd="''${args[0]:-}"

              case "$cmd" in
                build|test|doc|check|clippy|run)
                  exec ${rustToolchain}/bin/cargo "$@" --features use_ros_shim
                  ;;
                *)
                  exec ${rustToolchain}/bin/cargo "$@"
                  ;;
              esac
            '';
          in pkgs.mkShell {
            name = "ros2-rust";

            nativeBuildInputs = [
              cargoWrapper
              rustToolchain
              pkgs.pkg-config
              pkgs.llvmPackages.libclang
            ];

            LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
            RUSTFLAGS = "--cfg ros_distro=\"humble\"";

            shellHook = ''
              echo "═══════════════════════════════════════════════════════"
              echo "  ROS 2 Rust Development Environment"
              echo "═══════════════════════════════════════════════════════"
              echo "  Rust:   $(${rustToolchain}/bin/rustc --version)"
              echo ""
              echo "  cargo build/test/doc automatically use --features use_ros_shim"
              echo "═══════════════════════════════════════════════════════"
            '';
          };
        });

      checks = forAllSystems (system:
        let
          pkgs = pkgsFor system;
        in
        {
          fmt = pkgs.runCommand "check-fmt" {
            nativeBuildInputs = [ pkgs.rust-bin.stable.latest.default ];
            src = self;
          } ''
            cd $src
            cargo fmt -- --check
            touch $out
          '';
        });
    };
}
