#!/usr/bin/env python3
# This script produces the `vendor` module inside `rclrs` by copying the
# generated code for the `rosgraph_msgs`, `rcl_interfaces`, and `action_msgs`
# packages and their dependencies `builtin_interfaces` and
# `unique_identifier_msgs` and adjusting the submodule paths in the code.
# If these packages, or the `rosidl_generator_rs`, get changed, you can
# update the `vendor` module by running this script.
# The purpose is to avoid an external dependency on these message packages,
# which are not published on crates.io.

import argparse
from pathlib import Path
import shutil
import subprocess

vendored_packages = [
  "action_msgs",
  "builtin_interfaces",
  "rcl_interfaces",
  "rosgraph_msgs",
  "unique_identifier_msgs",
]

def get_args():
  parser = argparse.ArgumentParser(description='Vendor interface packages into rclrs')
  parser.add_argument('install_base', metavar='install_base', type=Path,
                      help='the install base (must have non-merged layout)')
  return parser.parse_args()

def adjust(current_package, text):
  for pkg in vendored_packages:
    text = text.replace(f'{pkg}::', f'crate::vendor::{pkg}::')
  text = text.replace('crate::msg', f'crate::vendor::{current_package}::msg')
  text = text.replace('crate::srv', f'crate::vendor::{current_package}::srv')
  text = text.replace('crate::action', f'crate::vendor::{current_package}::action')
  return text

def copy_adjusted(pkg, src, dst):
  dst.write_text(adjust(pkg, src.read_text()))
  subprocess.check_call(['rustfmt', str(dst)])

def main():
  args = get_args()
  assert args.install_base.is_dir(), "Install base does not exist"
  for pkg in vendored_packages:
    assert (args.install_base / pkg).is_dir(), f"Install base does not contain {pkg}"
  rclrs_root = Path(__file__).parent
  vendor_dir = rclrs_root / 'src' / 'vendor'
  if vendor_dir.exists():
    shutil.rmtree(vendor_dir)
  for pkg in vendored_packages:
    src = args.install_base / pkg / 'share' / pkg / 'rust' / 'src'
    dst = vendor_dir / pkg
    dst.mkdir(parents=True)
    copy_adjusted(pkg, src / 'msg.rs', dst / 'msg.rs')
    if (src / 'srv.rs').is_file():
      copy_adjusted(pkg, src / 'srv.rs', dst / 'srv.rs')
    if (src / 'action.rs').is_file():
      copy_adjusted(pkg, src / 'action.rs', dst / 'action.rs')
    copy_adjusted(pkg, src / 'lib.rs', dst / 'mod.rs')  # Rename lib.rs to mod.rs

  mod_contents = "//! Created by {}\n".format(Path(__file__).name)
  mod_contents += "#![allow(dead_code)]\n"
  mod_contents += "\n"
  for pkg in vendored_packages:
    mod_contents += f"pub mod {pkg};\n"
  (vendor_dir / 'mod.rs').write_text(mod_contents)

if __name__ == '__main__':
  main()
