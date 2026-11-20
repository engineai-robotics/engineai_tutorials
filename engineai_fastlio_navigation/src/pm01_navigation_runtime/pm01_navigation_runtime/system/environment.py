"""Resolve one user deployment file into process environment, never shell code."""
import argparse
from pathlib import Path
import yaml
from .network import write_profile


def environment(filename):
    path = Path(filename).expanduser().resolve(strict=True)
    data = yaml.safe_load(path.read_text())
    domain = int(data['ros_domain_id'])
    profile = data.get('profile', 'hardware')
    if not 0 <= domain <= 232 or profile not in ('hardware', 'sim'):
        raise ValueError('Invalid deployment domain/profile')
    if 'local_addresses' in data:
        dds = write_profile(path.parent/'fastdds-desktop.xml', data['local_addresses'], data['peers'])
    else:
        dds = str(Path(data['dds_profile']).expanduser().resolve(strict=True))
    values = {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DOMAIN_ID': str(domain),
              'ROS_LOCALHOST_ONLY': '0', 'PM01_USE_SIM_TIME': str(profile == 'sim').lower(),
              'FASTRTPS_DEFAULT_PROFILES_FILE': dds}
    if any('\n' in value or '\r' in value for value in values.values()):
        raise ValueError('Environment paths must not contain newlines')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config')
    args = parser.parse_args()
    for key, value in environment(args.config).items():
        print(key+'='+value)


if __name__ == '__main__':
    main()
