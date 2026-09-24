#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[1]
PROFILE = WORKSPACE / "config" / "fastdds_control.xml"
XML_NAMESPACE = {"f": "http://www.eprosima.com/XMLProfiles"}


class FastDDSControlProfileTests(unittest.TestCase):
    def test_profile_excludes_the_dedicated_robot_network(self):
        root = ET.parse(PROFILE).getroot()
        addresses = {
            element.text
            for element in root.findall(
                ".//f:interfaceWhiteList/f:address", XML_NAMESPACE
            )
        }

        self.assertEqual(addresses, {"127.0.0.1", "10.10.10.1"})
        self.assertNotIn("172.16.0.1", addresses)

    def test_profile_uses_only_the_whitelisted_udp_transport(self):
        root = ET.parse(PROFILE).getroot()
        self.assertEqual(
            root.findtext(".//f:useBuiltinTransports", namespaces=XML_NAMESPACE),
            "false",
        )
        self.assertEqual(
            root.findtext(".//f:userTransports/f:transport_id", namespaces=XML_NAMESPACE),
            "udp_control_transport",
        )


if __name__ == "__main__":
    unittest.main()
