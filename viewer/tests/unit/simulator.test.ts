import { HOME, exampleMission } from "../../apps/simulator/missions/default";
import test from "node:test";
import assert from "node:assert/strict";
import { MAVLink } from "@microagu/sim-browser/ardupilot/mavlink";
import { parseMission, validateMission } from "@microagu/contracts/mission";
test("MAVLink streams survive fragmentation, truncation and corrupt CRC", () => {
  const codec = new MAVLink(),
    packet = codec.encode("HEARTBEAT", {
      custom_mode: 7,
      type: 1,
      autopilot: 3,
      base_mode: 129,
      system_status: 4,
      mavlink_version: 3,
    });
  assert.deepEqual(codec.feed(packet.slice(0, 8)), []);
  assert.equal(codec.feed(packet.slice(8))[0].values.custom_mode, 7);
  const corrupt = packet.slice();
  corrupt[10] ^= 8;
  assert.deepEqual(codec.feed(corrupt), []);
  assert.equal(codec.feed(packet)[0].values.base_mode, 129);
});
test("WPL home is excluded and relative waypoints retain order", () => {
  const rows =
    "QGC WPL 110\n0 1 0 16 0 0 0 0 45 -121 600 1\n1 0 3 16 0 22 0 0 45.001 -121.001 60 1";
  const mission = parseMission(rows);
  assert.equal(mission.length, 1);
  assert.equal(mission[0].alt, 60);
  assert.equal(mission[0].frame, 3);
  assert.throws(() => parseMission(rows.replace("1 0 3", "2 0 3")), /sequence/);
});
test("QGC accepts null yaw but rejects complex and terrain-relative items", () => {
  const data = {
    fileType: "Plan",
    mission: {
      items: [
        {
          type: "SimpleItem",
          command: 16,
          frame: 3,
          params: [0, 22, 0, null, 45, -121, 60],
        },
      ],
    },
  };
  assert.equal(parseMission(JSON.stringify(data))[0].params[3], 0);
  data.mission.items[0].frame = 10;
  assert.throws(() => parseMission(JSON.stringify(data)), /altitude/);
  data.mission.items[0].type = "ComplexItem";
  assert.throws(() => parseMission(JSON.stringify(data)), /SimpleItem/);
});
test("example is valid and unsupported mission commands are rejected", () => {
  assert.equal(validateMission(exampleMission()).length, 11);
  assert.throws(
    () => validateMission([{ ...exampleMission()[0], command: 999 }]),
    /unsupported/,
  );
});
