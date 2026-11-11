"""Process-local Fast DDS UDP configuration; no network/system mutations."""
import ipaddress
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def fastdds_xml(addresses, peers, participant_range=64):
    def checked(values):
        return list(dict.fromkeys(str(ipaddress.IPv4Address(v)) for v in values))
    addresses, peers = checked(addresses), checked(peers)
    if not addresses or not 1 <= participant_range <= 120:
        raise ValueError('Supply local IPv4 addresses and participant range 1..120')
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    transport = ET.SubElement(ET.SubElement(root, 'transport_descriptors'), 'transport_descriptor')
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
    ET.SubElement(ET.SubElement(rtps, 'userTransports'), 'transport_id').text = 'pm01_udp'
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def write_profile(path, addresses, peers):
    path = Path(path).expanduser()
    content = fastdds_xml(addresses, peers)
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
    parser = argparse.ArgumentParser(description='Write project-local loopback UDP configuration')
    parser.add_argument('--loopback', action='store_true', required=True)
    parser.parse_args()
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))
    print(write_profile(state/'pm01_navigation/fastdds/loopback.xml', ['127.0.0.1'], []))


if __name__ == '__main__':
    main()
