"""Read the running motion interface's ROS domain without changing services."""
import json
from pathlib import Path


def inspect_motion_domain(proc=Path('/proc')):
    found, denied = [], False
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            words = (path/'cmdline').read_bytes().split(b'\0')
            if not words or Path(words[0].decode(errors='replace')).name not in (
                    'src_executor', 'locomotion_interface_node'):
                continue
            try:
                environment = (path/'environ').read_bytes().split(b'\0')
            except PermissionError:
                denied = True
                continue
            allowed = {'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION'}
            values = {}
            for item in environment:
                key, _, value = item.partition(b'=')
                name = key.decode(errors='replace')
                if name in allowed:
                    values[name] = value.decode(errors='replace')
            domain = int(values.get('ROS_DOMAIN_ID') or '0')
            if not 0 <= domain <= 232:
                return {'state': 'invalid', 'reason': 'motion_domain_out_of_range'}
            interfaces, current = [], None
            try:
                for line in (path/'net/igmp').read_text().splitlines():
                    fields = line.split()
                    if len(fields) > 2 and fields[0].isdigit() and fields[2] == ':':
                        current = fields[1]
                    if fields and fields[0].upper() == '0100FFEF' and current:
                        interfaces.append(current)
            except OSError:
                pass
            found.append(dict(pid=int(path.name), domain=domain,
                localhost_only=values.get('ROS_LOCALHOST_ONLY') == '1',
                middleware=values.get('RMW_IMPLEMENTATION', ''),
                dds_multicast_interfaces=interfaces))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (OSError, ValueError):
            return {'state': 'unavailable', 'reason': 'motion_process_unreadable'}
    if denied:
        return {'state': 'needs_privilege'}
    if not found:
        return {'state': 'not_running'}
    domains = {item['domain'] for item in found}
    if len(domains) != 1:
        return {'state': 'ambiguous', 'domains': sorted(domains)}
    return dict(state='detected', domain=domains.pop(), processes=found)


if __name__ == '__main__':
    print(json.dumps(inspect_motion_domain()))
