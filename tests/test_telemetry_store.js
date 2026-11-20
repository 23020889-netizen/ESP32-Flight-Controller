/**
 * test_telemetry_store.js
 * Kiểm thử luồng dữ liệu IMU từ ESP32 lên Web (TelemetryDataStore):
 *  - WebSocket JSON: giữ đủ gx/gy/gz, tune, setpoint, tau... cho panel Auto-Tune
 *  - JSON hỏng: được đếm, không làm treo
 *  - WebSerial: parse đúng định dạng dòng [PX4_EKF] / [MPU6050] / [ESC_QUAD] firmware in ra
 * Chạy: node tests/test_telemetry_store.js
 */
const assert = require("assert");
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const src = fs.readFileSync(
  path.join(__dirname, "../tools/hitl_monitor_web/monitor_Web_resource/js/telemetry_data_store.js"), "utf8");

const timers = [];
const sandbox = {
  console: { log() {}, warn() {}, error() {} },
  performance: { now: () => 1000 },
  setInterval: (fn) => { timers.push(fn); return timers.length; },
  setTimeout: () => 0,
  document: { getElementById: () => null },
  window: {},
  navigator: {},
};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(src, sandbox);
const Store = sandbox.window.TelemetryDataStore;

let passed = 0;
function test(name, fn) { fn(); passed++; console.log("✓ " + name); }

function fakeWsStore() {
  const store = new Store();
  store.connectionMode = "ws";
  class FakeWS { constructor() { FakeWS.last = this; this.readyState = 1; } send() {} close() {} }
  FakeWS.OPEN = 1;
  sandbox.WebSocket = FakeWS;
  store.connectWebSocket("ws://test:81");
  return { store, ws: FakeWS.last };
}

test("WebSocket JSON giữ gx/gy/gz + khối tune + setpoint/tau", () => {
  const { store, ws } = fakeWsStore();
  const pkt = {
    t: 1000, roll: 1.5, pitch: -2.25, yaw: 90, gx: 12.5, gy: -7.25, gz: 3,
    ax: 0.1, ay: 0.2, az: 9.8, m: [10, 20, 30, 40], us: [1100, 1200, 1300, 1400], state: "ARMED",
    sp_r: 5, sp_p: -5, sp_y: 90, sp_t: 40, rsp: [30, -30, 0], pid_i: [0.01, -0.02, 0],
    tau: [0.1, -0.1, 0.01], u: [0.4, 0.5, 0.4, 0.5], yaw_ok: 1, imu_src: 1, loop_hz: 200, loop_max_us: 5200,
    tune: { active: true, axis: "roll", cycle: 3, min: 8, state: 1, err: 0 }
  };
  ws.onmessage({ data: JSON.stringify(pkt) });
  const s = store.sampleBuffer[store.sampleBuffer.length - 1];
  assert.strictEqual(s.gx, 12.5);
  assert.strictEqual(s.gy, -7.25);
  assert.strictEqual(s.gz, 3);
  assert.strictEqual(s.tune.axis, "roll");
  assert.strictEqual(s.tune.cycle, 3);
  assert.deepStrictEqual(Array.from(s.tau), [0.1, -0.1, 0.01]);
  assert.deepStrictEqual(Array.from(s.rsp), [30, -30, 0]);
  assert.strictEqual(s.loop_hz, 200);
  assert.strictEqual(s.imu_src, 1);
});

test("Phát hiện mất gói theo đồng hồ ESP32 (t) kể cả khi tràn uint32", () => {
  const { store, ws } = fakeWsStore();
  ws.onmessage({ data: JSON.stringify({ t: 4294967290, roll: 0, m: [0, 0, 0, 0] }) });
  ws.onmessage({ data: JSON.stringify({ t: 14, roll: 0, m: [0, 0, 0, 0] }) });   // gap 20 ms qua tràn
  assert.strictEqual(store.espGapDropCount, 0);
  ws.onmessage({ data: JSON.stringify({ t: 214, roll: 0, m: [0, 0, 0, 0] }) });  // gap 200 ms
  assert.strictEqual(store.espGapDropCount, 1);
});

test("JSON hỏng (bị cắt / 'nan') được đếm, không đẩy mẫu rác", () => {
  const { store, ws } = fakeWsStore();
  const before = store.sampleBuffer.length;
  ws.onmessage({ data: '{"roll":nan,"pitch":1}' });
  ws.onmessage({ data: '{"roll":1.0,"pitch":2.0,"ya' });
  assert.strictEqual(store.jsonParseErrors, 2);
  assert.strictEqual(store.sampleBuffer.length, before);
});

test("Bản tin log {\"log\":..} không ghi đè góc", () => {
  const { store, ws } = fakeWsStore();
  ws.onmessage({ data: JSON.stringify({ roll: 7, pitch: 1, yaw: 2, m: [0, 0, 0, 0] }) });
  const n = store.sampleBuffer.length;
  ws.onmessage({ data: JSON.stringify({ log: "[WIFI] Quad Armed" }) });
  assert.strictEqual(store.sampleBuffer.length, n);
});

test("WebSerial: parse đúng các dòng firmware in ra ([PX4_EKF] -> [ESC_QUAD])", () => {
  const store = new Store();
  // Đúng định dạng snprintf trong dual_imu_esp32c6.ino::printTelemetry()
  store.parseSerialLine("[PX4_EKF] R:  12.3° P:  -4.5° Y:  80.0° | Acc:[  0.10, -0.20,  9.80] | Gyr:[  15.2,  -3.1,   0.4] (SRC:XSENS YAW:OK)");
  store.parseSerialLine("[MPU6050] R:  12.0° P:  -4.0° Y:   1.0° | Acc:[  0.10, -0.20,  9.80] | Gyr:[  15.0,  -3.0,   0.5]");
  store.parseSerialLine("[Xsens  ] R:  12.4° P:  -4.6° Y:  80.1° | Acc:[  0.10, -0.20,  9.80] | Gyr:[  15.3,  -3.2,   0.3] age=5ms err=0");
  store.parseSerialLine("[ESC_QUAD] State:ARMED    | M1(P7-CCW):1400us(40.0%) | M2(P6-CCW):1500us(50.0%) | M3(P5-CW):1400us(40.0%) | M4(P4-CW):1500us(50.0%) @ 50Hz");
  const s = store.sampleBuffer[store.sampleBuffer.length - 1];
  assert.strictEqual(s.roll, 12.3);
  assert.strictEqual(s.pitch, -4.5);
  assert.strictEqual(s.yaw, 80);
  assert.strictEqual(s.gx, 15.2);
  assert.strictEqual(s.gy, -3.1);
  assert.strictEqual(s.mpu_r, 12.0);
  assert.strictEqual(s.xs_y, 80.1);
  assert.strictEqual(store.lastEspState, "ARMED");
  assert.deepStrictEqual(Array.from(s.m), [400, 500, 400, 500]);
});

test("Firmware cũ ([ATTITUDE]) vẫn được hỗ trợ", () => {
  const store = new Store();
  store.parseSerialLine("[ATTITUDE] R:   3.0° P:   2.0° Y:   1.0° (Yaw:VALID) | Gyr:[   1.0,   2.0,   3.0] (0% CPU)");
  store.parseSerialLine("[ESC_QUAD] State:DISARMED | M1(P7-CCW):1000us( 0.0%) | M2(P6-CCW):1000us( 0.0%) | M3(P5-CW):1000us( 0.0%) | M4(P4-CW):1000us( 0.0%) @ 50Hz");
  const s = store.sampleBuffer[store.sampleBuffer.length - 1];
  assert.strictEqual(s.roll, 3);
  assert.strictEqual(s.gz, 3);
});

console.log(`\nALL ${passed} TELEMETRY STORE TESTS PASSED`);
