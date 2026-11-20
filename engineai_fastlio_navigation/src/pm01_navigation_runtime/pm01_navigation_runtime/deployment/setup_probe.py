"""Identifiers for bounded deployment connectivity probes (no ROS imports)."""
import re

def namespace(nonce):
    if not re.fullmatch("[a-f0-9]{12}", nonce):
        raise ValueError("Invalid probe identifier")
    return "/pm01_setup_probe/p_"+nonce
