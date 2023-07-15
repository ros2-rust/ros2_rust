# This script produces the `vendor` module inside `rclrs` by copying
# the generated code for the `rcl_interfaces` package and its dependency
# `builtin_interfaces` and adjusting the submodule paths in the code.
# If these packages, or the `rosidl_generator_rs`, get changed, you can
# update the `vendor` module by running this script.
# The purpose is to avoid an external dependency on `rcl_interfaces`, which
# is not published on crates.io.

import argparse
from pathlib import Path
import shutil
import subprocess

def get_args():
  parser = argparse.ArgumentParser(description='Vendor the rcl_interfaces and builtin_interfaces packages into rclrs')
  parser.add_argument('install_base', metavar='install_base', type=Path,
                      help='the install base (must have non-merged layout)')
  return parser.parse_args()

def adjust(pkg, text):
  text = text.replace('builtin_interfaces::', 'crate::vendor::builtin_interfaces::')
  text = text.replace('rcl_interfaces::', 'crate::vendor::rcl_interfaces::')
  text = text.replace('crate::msg', f'crate::vendor::{pkg}::msg')
  text = text.replace('crate::srv', f'crate::vendor::{pkg}::srv')
  return text

def copy_adjusted(pkg, src, dst):
  dst.write_text(adjust(pkg, src.read_text()))
  subprocess.check_call(['rustfmt', str(dst)])

mod_contents = """//! Created by {}
#![allow(dead_code)]
#![allow(clippy::derive_partial_eq_without_eq)]

pub mod builtin_interfaces;
pub mod rcl_interfaces;
""".format(Path(__file__).name)

def main():
  args = get_args()
  assert args.install_base.is_dir(), "Install base does not exist"
  assert (args.install_base / 'builtin_interfaces').is_dir(), "Install base does not contain builtin_interfaces"
  assert (args.install_base / 'rcl_interfaces').is_dir(), "Install base does not contain rcl_interfaces"
  rclrs_root = Path(__file__).parent
  vendor_dir = rclrs_root / 'src' / 'vendor'
  if vendor_dir.exists():
    shutil.rmtree(vendor_dir)
  for pkg in ['builtin_interfaces', 'rcl_interfaces']:
    src = args.install_base / pkg / 'share' / pkg / 'rust' / 'src'
    dst = vendor_dir / pkg
    dst.mkdir(parents=True)
    copy_adjusted(pkg, src / 'msg.rs', dst / 'msg.rs')
    if (src / 'srv.rs').is_file():
      copy_adjusted(pkg, src / 'srv.rs', dst / 'srv.rs')
    copy_adjusted(pkg, src / 'lib.rs', dst / 'mod.rs')  # Rename lib.rs to mod.rs
  (vendor_dir / 'mod.rs').write_text(mod_contents)
    


if __name__ == '__main__':
  main()
