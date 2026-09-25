#!/usr/bin/env python3
"""Read-only presence, version and effective-overlay audit. Never install or repair."""
import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET
sys.dont_write_bytecode = True
import yaml

ROOT = Path(__file__).resolve().parents[1]


def compare(expected, actual):
    if actual is None:
        return 'missing'
    if expected is None:
        return 'unverified'
    return 'match' if expected == actual else 'mismatch'


def apt_version(name):
    result = subprocess.run(['dpkg-query', '-W', '-f=${Version}\t${Architecture}\t${db:Status-Status}', name], text=True, capture_output=True)
    fields = result.stdout.split('\t')
    return (fields[0], fields[1]) if result.returncode == 0 and len(fields) == 3 and fields[2] == 'installed' else (None, None)


def upstream(version):
    return version.split(':')[-1].split('-')[0].split('+')[0] if version else None


def loaded_module(name):
    # Resolve without importing the package: imports may write caches or run hooks.
    import importlib.util
    spec = importlib.util.find_spec(name)
    if spec is None or not spec.origin:
        raise ImportError('Module not found: ' + name)
    origin = Path(spec.origin)
    search = origin.parent.parent if spec.submodule_search_locations else origin.parent
    distribution = {'PIL': 'Pillow', 'yaml': 'PyYAML', 'colcon_core': 'colcon-core'}.get(name, name)
    normalized = lambda value: value.lower().replace('_', '-').replace('.', '-')
    version = None
    for item in importlib.metadata.distributions(path=[str(search)]):
        if normalized(item.metadata.get('Name', '')) == normalized(distribution):
            version = item.version
            break
    return {'actual_version': version, 'loaded_from': str(origin)}


def loaded_ros(name):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory(name))
    return {'actual_version': ET.parse(share / 'package.xml').findtext('version'), 'loaded_from': str(share)}


def check(row, role, arch):
    version, found_arch = apt_version(row['package'])
    source = 'pc_installed' if role == 'pc' else 'pi_historical'
    record = row['version_sources'].get(source)
    expected = record.get('version') if record else None
    status = compare(expected, version)
    if record and record['architecture'] not in (arch, 'all'):
        status = 'inapplicable_architecture'
    result = {'package': row['package'], 'source': source, 'expected_version': expected,
              'actual_version': version, 'expected_architecture': record.get('architecture') if record else arch,
              'actual_architecture': found_arch, 'status': status}
    records = {k: v['version'] for k, v in row['version_sources'].items() if v}
    result['recorded_source_differences'] = records if len(set(records.values())) > 1 else {}
    result['difference_scope'] = 'Separate host/platform observations; not an automatic compatibility failure'
    effective = None
    try:
        if row.get('python_module'):
            effective = loaded_module(row['python_module'])
        elif row.get('ros_package'):
            effective = loaded_ros(row['ros_package'])
    except (ImportError, OSError, ValueError, LookupError) as error:
        effective = {'actual_version': None, 'loaded_from': None, 'error': str(error)}
    if effective is not None:
        # Report comparisons against every applicable same-platform observation.
        checks = []
        for kind, observation in row['version_sources'].items():
            if not observation or (role == 'pc') != kind.startswith('pc_'):
                continue
            if observation['architecture'] not in (arch, 'all'):
                continue
            wanted = upstream(observation['version'])
            checks.append({'source': kind, 'expected_version': wanted, 'actual_version': effective['actual_version'],
                           'status': compare(wanted, effective['actual_version'])})
        effective['version_checks'] = checks
        effective['status'] = ('missing' if effective['actual_version'] is None else
                               'unverified' if not checks else
                               'mismatch' if any(x['status'] != 'match' for x in checks) else 'match')
        result['effective_environment'] = effective
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['pc', 'pi'], required=True)
    parser.add_argument('--requirements', default=str(ROOT / 'requirements.yaml'))
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    data = yaml.safe_load(Path(args.requirements).read_text())
    arch = subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip()
    rows = [check(r, args.role, arch) for r in data['dependencies'] if r['role'] in ('common', args.role)]
    os_release = platform.freedesktop_os_release()
    platform_check = {'os_version': os_release.get('VERSION_ID'), 'expected_os_version': '24.04',
                      'python_version': platform.python_version(), 'expected_python_series': '3.12',
                      'architecture': arch, 'expected_architecture': data['platforms'][args.role]['architecture'],
                      'ros_distro': os.environ.get('ROS_DISTRO'), 'expected_ros_distro': 'jazzy'}
    platform_ok = (platform_check['os_version'] == '24.04' and platform_check['python_version'].startswith('3.12.') and
                   arch == platform_check['expected_architecture'] and platform_check['ros_distro'] == 'jazzy')
    bad = [r for r in rows if r['status'] != 'match' or r.get('effective_environment', {}).get('status', 'match') != 'match']
    report = {'platform': platform_check, 'platform_matches': platform_ok, 'dependencies': rows,
              'unresolved_count': len(bad), 'modified_system': False,
              'installation_guidance': 'Review mismatches explicitly. Use apt/rosdep for missing packages; no automatic fixes were attempted.'}
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(json.dumps(platform_check))
        for r in rows:
            print(f"{r['package']}: {r['status']} | {r['source']} expected={r['expected_version']} found={r['actual_version']} arch={r['actual_architecture']}")
            if r['recorded_source_differences']:
                print('  Recorded source differences:', json.dumps(r['recorded_source_differences']))
            if 'effective_environment' in r:
                print('  Loaded environment:', json.dumps(r['effective_environment']))
        print(report['installation_guidance'])
    return 1 if bad or not platform_ok else 0

if __name__ == '__main__':
    sys.exit(main())
