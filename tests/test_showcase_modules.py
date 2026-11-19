"""Check the published whole-robot scene, including each module's moving frames."""
import json
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class InstalledModules(unittest.TestCase):
    def test_all_corners_steer_and_roll_independently(self):
        receipt = json.loads((ROOT / "docs/media/swerve-r7.json").read_bytes())
        with (ROOT / receipt["assets"]["model"]["path"]).open("rb") as stream:
            stream.seek(12)
            length, kind = struct.unpack("<I4s", stream.read(8))
            self.assertEqual(kind, b"JSON")
            gltf = json.loads(stream.read(length))
        nodes = gltf["nodes"]
        roots = gltf["scenes"][0]["nodes"]
        clip, = (a for a in gltf["animations"] if a["name"] == receipt["clip"])
        rotations = {c["target"]["node"] for c in clip["channels"] if c["target"]["path"] == "rotation"}
        layout = json.loads((ROOT / "cad/drivetrain/maxswerve-layout.json").read_bytes())
        for corner, station in layout["stations"].items():
            with self.subTest(corner=corner):
                def find(role, candidates):
                    idx, = (i for i in candidates if nodes[i].get("name") == corner+" "+role)
                    return idx
                fixed = find("module housing", roots)
                steer = find("steering", roots)
                wheel = find("tire envelope", nodes[steer]["children"])
                x, y, z = station["wheelCenterMm"]
                for idx in (fixed, steer):
                    self.assertEqual([round(v*1000, 3) for v in nodes[idx]["translation"]], [x, z, -y])
                self.assertNotIn(fixed, rotations)
                self.assertTrue({steer, wheel} <= rotations)
                for parent, roles in ((fixed, ("NEO drive motor","NEO 550 steering motor","lower housing","upper housing")),
                                      (steer, ("azimuth carrier","left fork plate","right fork plate")),
                                      (wheel, ("wheel hub",))):
                    for role in roles:
                        self.assertIn("mesh", nodes[find(role, nodes[parent]["children"])])

    def test_first_floor_note_film_reaches_its_recorded_counter_event(self):
        receipt = json.loads((ROOT / "docs/media/swerve-r7.json").read_bytes())
        route = json.loads((ROOT / "sim/pressure/evidence/maxswerve-six-note-D1.json").read_bytes())
        first = route["records"][0]
        self.assertEqual(receipt["plantSha256"], route["plantArchiveSha256"])
        self.assertEqual(receipt["sourceSha256"], route["displayArchiveSha256"])
        self.assertEqual(receipt["traceSha256"]["pickup"], first["pickupTrace"])
        self.assertEqual(receipt["traceSha256"]["field"], first["fieldTrace"])
        data = (ROOT / receipt["assets"]["model"]["path"]).read_bytes()
        length = struct.unpack_from("<I", data, 12)[0]
        gltf = json.loads(data[20:20+length])
        clip, = (a for a in gltf["animations"] if a["name"] == receipt["clip"])
        extras = clip["extras"]
        self.assertEqual(extras["sourceStartS"], first["pickupStartS"])
        self.assertEqual(extras["sensorS"], first["pairedSensorS"])
        duration = gltf["accessors"][clip["samplers"][0]["input"]]["max"][0]
        self.assertAlmostEqual(duration, extras["sensorS"] - extras["sourceStartS"], places=5)
        film = receipt["film"]
        expected = (film["slowFromMatchS"] - extras["sourceStartS"]
                    + (extras["openingS"] - film["slowFromMatchS"]) / film["slowRate"]
                    + extras["sensorS"] - extras["openingS"] + film["holdS"])
        self.assertAlmostEqual((film["frames"] - 1) / film["framesPerSecond"], expected, delta=.05)
