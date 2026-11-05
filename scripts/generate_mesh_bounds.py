# SPDX-License-Identifier: Apache-2.0
"""Export reproducible, hash-pinned mesh bounds using the native Gazebo loader."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    """Keep mesh conversion offline; running Python nodes need only this small catalog."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loader', required=True)
    parser.add_argument('--mesh-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    revision = '55ff151632a0220bf3ae0c0772392615d86da7a8'
    source = (args.mesh_dir/'trunk.dae').resolve().parent
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                                     text=True).strip()
    names = ['trunk', 'hip', 'thigh', 'thigh_mirror', 'calf', 'L1_4D_Lidar']
    dirty = subprocess.check_output(
        ['git', '-C', str(source), 'status', '--porcelain', '--']
        + [name+'.dae' for name in names], text=True)
    if actual != revision or dirty:
        raise ValueError('expected_clean_pinned_mesh_sources')
    catalog = {'schema_version': 1, 'loader': 'gz::common::MeshManager',
               'dependency_revision': actual, 'meshes': {}}
    for name in names:
        path = args.mesh_dir / (name+'.dae')
        if path.resolve().parent != source:
            raise ValueError('mesh_sources_do_not_share_pinned_directory')
        result = subprocess.run([args.loader, str(path)], capture_output=True, text=True,
                                timeout=20, check=True)
        rows = [line.removeprefix('GUIDEDOG_BOUNDS ') for line in result.stdout.splitlines()
                if line.startswith('GUIDEDOG_BOUNDS ')]
        if len(rows) != 1:
            raise ValueError('native_loader_did_not_produce_one_bounds_record')
        record = json.loads(rows[0])
        record['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        catalog['meshes'][path.name] = record
    args.output.write_text(json.dumps(catalog, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
