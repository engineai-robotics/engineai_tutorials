"""Fast DDS profiles for robot networking and single-host simulation."""
import ipaddress
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def fastdds_xml(addresses, peers, participant_range=64, *, shm_id=None):
    def checked(values):
        return list(dict.fromkeys(str(ipaddress.IPv4Address(v)) for v in values))
    addresses, peers = checked(addresses), checked(peers)
    if not addresses or not 1 <= participant_range <= 120:
        raise ValueError('Supply local IPv4 addresses and participant range 1..120')
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    transports = ET.SubElement(root, 'transport_descriptors')
    transport = ET.SubElement(transports, 'transport_descriptor')
    ET.SubElement(transport, 'transport_id').text = 'pm01_udp'
    ET.SubElement(transport, 'type').text = 'UDPv4'
    interfaces = ET.SubElement(transport, 'interfaceWhiteList')
    for address in addresses:
        ET.SubElement(interfaces, 'address').text = address
    ET.SubElement(transport, 'maxInitialPeersRange').text = str(participant_range)
    rtps = ET.SubElement(ET.SubElement(root, 'participant', profile_name='pm01', is_default_profile='true'), 'rtps')
    builtin = ET.SubElement(rtps, 'builtin')
    initial = ET.SubElement(builtin, 'initialPeersList')
    for peer in checked(['127.0.0.1'] + addresses + peers):
        udp = ET.SubElement(ET.SubElement(initial, 'locator'), 'udpv4')
        ET.SubElement(udp, 'address').text = peer
    # Preserve default discovery listeners, including multicast bootstrap.
    # A nonempty unicast listener list suppresses Fast DDS default multicast setup.
    ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
    active = ET.SubElement(rtps, 'userTransports')
    ET.SubElement(active, 'transport_id').text = 'pm01_udp'
    if shm_id:
        add_shm_transport(transports, active, shm_id)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def add_shm_transport(descriptors, active, name, ns=''):
    """Keep existing transport tuning and enable one SHM transport per participant."""
    by_id = {d.findtext(ns+'transport_id'): d for d in descriptors.findall(ns+'transport_descriptor')}
    if name in by_id and by_id[name].findtext(ns+'type') != 'SHM':
        raise ValueError('Conflicting DDS transport name: '+name)
    for entry in active.findall(ns+'transport_id'):
        if entry.text in by_id and by_id[entry.text].findtext(ns+'type') == 'SHM':
            return False
    if name not in by_id:
        shm = ET.SubElement(descriptors, ns+'transport_descriptor')
        ET.SubElement(shm, ns+'transport_id').text = name
        ET.SubElement(shm, ns+'type').text = 'SHM'
        ET.SubElement(shm, ns+'segment_size').text = str(16 * 1024 * 1024)
    ET.SubElement(active, ns+'transport_id').text = name
    return True


def write_profile(path, addresses, peers, *, robot=False):
    return _write_xml(path, fastdds_xml(addresses, peers, shm_id='pm01_robot_shm' if robot else None))


def write_sim_profile(path):
    return _write_xml(path, fastdds_xml(['127.0.0.1'], [], shm_id='pm01_sim_shm'))


def _write_xml(path, content):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != content:
        # Readers may create DDS participants while another terminal loads the
        # same configuration. Never expose a partly written XML document.
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
        try:
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
    return str(path.resolve())


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Write the single-host simulation DDS profile')
    parser.add_argument('--loopback', action='store_true', required=True)
    parser.parse_args()
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))
    print(write_sim_profile(state/'pm01_navigation/fastdds/sim-local.xml'))


if __name__ == '__main__':
    main()
