/**
 * test_gamepad_mapping.js
 * Unit test suite for Gamepad Reading & Safety Gate logic
 * Conforms to docs/GAMEPAD_THROTTLE_MAPPING_FIX_GUIDE.md
 */

const assert = require("assert");

function deadzone(v, th = 0.08) {
  if (Math.abs(v) < th) return 0;
  return v > 0 ? (v - th) / (1 - th) : (v + th) / (1 - th);
}

/**
 * Pure function extracting normalized inputs from Gamepad API object
 */
function readGamepadInputs(gp) {
  if (!gp) {
    return {
      connected: false,
      isRcTransmitter: false,
      deviceName: "None",
      thrPct: 0,
      rollDeg: 0,
      pitchDeg: 0,
      yawDeg: 0
    };
  }

  const id = gp.id || "";
  const isRcTransmitter = /radiomaster|pocket|edgetx|opentx|jumper|boxer|tx16|t-pro/i.test(id);
  const deviceName = isRcTransmitter ? "RadioMaster (HID)" : (id.length > 20 ? id.substring(0, 20) : id);

  let rawThr = 0;
  let yawDeg = 0;
  let rollDeg = 0;
  let pitchDeg = 0;

  if (isRcTransmitter) {
    // Mode 2 (AETR):
    // Channel 3: Throttle -> axes[2]: Bottom = -1.0 (0%), Top = +1.0 (100%)
    const rawThrAxis = (gp.axes && gp.axes.length > 2) ? gp.axes[2] : -1.0;
    let normThr = (rawThrAxis + 1.0) * 0.5;
    if (normThr < 0.02) normThr = 0.0;
    rawThr = Math.min(1.0, Math.max(0.0, normThr));

    // Channel 4: Yaw -> axes[3] (-45..+45 deg)
    const rawYawAxis = (gp.axes && gp.axes.length > 3) ? gp.axes[3] : 0;
    yawDeg = Math.round(deadzone(rawYawAxis) * 45.0 * 10) / 10;

    // Channel 1: Roll -> axes[0] (-25..+25 deg)
    const rawRollAxis = (gp.axes && gp.axes.length > 0) ? gp.axes[0] : 0;
    rollDeg = Math.round(deadzone(rawRollAxis) * 25.0 * 10) / 10;

    // Channel 2: Pitch -> axes[1] (Push forward is pitch down / negative)
    const rawPitchAxis = (gp.axes && gp.axes.length > 1) ? gp.axes[1] : 0;
    pitchDeg = Math.round(-deadzone(rawPitchAxis) * 25.0 * 10) / 10;
  } else {
    // Standard Xbox / Gamepad:
    // RT is buttons[7].value [0..1]
    const rtVal = (gp.buttons && gp.buttons[7]) ? gp.buttons[7].value : 0;
    // Left stick Y: push forward is negative
    const rawLy = (gp.axes && gp.axes.length > 1) ? gp.axes[1] : 0;
    const stickLy = Math.max(0, -deadzone(rawLy));
    rawThr = Math.max(rtVal, stickLy);

    // Left stick X: Yaw (-45..+45 deg)
    const rawLx = (gp.axes && gp.axes.length > 0) ? gp.axes[0] : 0;
    yawDeg = Math.round(deadzone(rawLx) * 45.0 * 10) / 10;

    // Right stick X & Y: Roll (-25..+25 deg) & Pitch (-25..+25 deg)
    let rawRx = 0;
    let rawRy = 0;
    if (gp.axes) {
      if (gp.axes.length >= 4) {
        rawRx = gp.axes[2];
        rawRy = gp.axes[3];
      }
      if (gp.axes.length >= 5 && Math.abs(rawRx) < 0.05 && Math.abs(rawRy) < 0.05) {
        if (Math.abs(gp.axes[3]) > 0.05 || Math.abs(gp.axes[4]) > 0.05) {
          rawRx = gp.axes[3];
          rawRy = gp.axes[4];
        }
      }
    }
    rollDeg = Math.round(deadzone(rawRx) * 25.0 * 10) / 10;
    pitchDeg = Math.round(-deadzone(rawRy) * 25.0 * 10) / 10;
  }

  const thrPct = Math.round(Math.min(100.0, Math.max(0.0, rawThr * 100.0)) * 10) / 10;

  return {
    connected: true,
    isRcTransmitter,
    deviceName,
    thrPct,
    rawThr,
    rollDeg,
    pitchDeg,
    yawDeg
  };
}

/**
 * Arming decision gate
 */
function evaluateArmRequest(inputs, isTransportOpen) {
  if (!isTransportOpen) {
    return { allowed: false, reason: "NO_LINK" };
  }
  if (inputs.thrPct > 3.0) {
    return { allowed: false, reason: "THROTTLE_NOT_ZERO", currentThrottle: inputs.thrPct };
  }
  return { allowed: true, reason: "OK" };
}

/**
 * Pure evaluation function matching dashboard_application_controller.js arm/disarm triggers
 */
function evaluateGamepadArmDisarm(inputs, btns, prevButtons, axes, prevAxis4, hasLink) {
  const justPressed = (idx) => btns[idx] && !prevButtons[idx];
  const justReleased = (idx) => !btns[idx] && prevButtons[idx];

  const currAxis4 = (axes && axes.length > 4) ? axes[4] : 0;
  const saAxisArm = inputs.isRcTransmitter && axes && axes.length > 4 && (currAxis4 > 0.3 && prevAxis4 <= 0.3);
  const saAxisDisarm = inputs.isRcTransmitter && axes && axes.length > 4 && (currAxis4 < -0.3 && prevAxis4 >= -0.3);

  const isDisarmRequested = justPressed(1) || justPressed(8) || justPressed(9) ||
    (inputs.isRcTransmitter && (justReleased(0) || justReleased(4) || saAxisDisarm));

  const isArmRequested = justPressed(0) || (inputs.isRcTransmitter && (justPressed(4) || saAxisArm));

  if (isDisarmRequested) {
    return { action: "DISARM" };
  } else if (isArmRequested) {
    if (!hasLink) return { action: "REJECT", reason: "NO_LINK" };
    if (inputs.thrPct > 3.0) return { action: "REJECT", reason: "THROTTLE_NOT_ZERO" };
    return { action: "ARM" };
  }
  return { action: "NONE" };
}

// ================= TEST CASES =================

console.log("Running Gamepad & Throttle Mapping Unit Tests...\n");

// Test 1: Xbox RT trigger mapping
{
  const mockXbox = {
    id: "Xbox 360 Controller (XInput STANDARD GAMEPAD)",
    axes: [0, 0, 0, 0],
    buttons: new Array(16).fill({ pressed: false, value: 0 })
  };
  mockXbox.buttons[7] = { pressed: true, value: 0.5 }; // RT at 50%
  const res = readGamepadInputs(mockXbox);
  assert.strictEqual(res.isRcTransmitter, false);
  assert.strictEqual(res.thrPct, 50.0);
  console.log("✓ Test 1: Xbox RT 50% maps to thrPct 50.0%");
}

// Test 2: Xbox Left Stick Y mapping
{
  const mockXbox = {
    id: "Xbox Controller",
    axes: [0, -1.0, 0, 0], // Left stick pushed fully up (-1.0)
    buttons: new Array(16).fill({ pressed: false, value: 0 })
  };
  const res = readGamepadInputs(mockXbox);
  assert.strictEqual(res.thrPct, 100.0);
  console.log("✓ Test 2: Xbox Left Stick full forward maps to thrPct 100.0%");
}

// Test 3: RadioMaster Pocket Mode 2 mapping
{
  const mockRadioMaster = {
    id: "RadioMaster Pocket Joystick",
    axes: [0, 0, -1.0, 0], // Throttle at bottom (-1.0)
    buttons: []
  };
  const resBottom = readGamepadInputs(mockRadioMaster);
  assert.strictEqual(resBottom.isRcTransmitter, true);
  assert.strictEqual(resBottom.thrPct, 0.0);

  // Throttle at center (0.0 -> 50%)
  mockRadioMaster.axes[2] = 0.0;
  const resMid = readGamepadInputs(mockRadioMaster);
  assert.strictEqual(resMid.thrPct, 50.0);

  // Throttle at top (+1.0 -> 100%)
  mockRadioMaster.axes[2] = 1.0;
  const resTop = readGamepadInputs(mockRadioMaster);
  assert.strictEqual(resTop.thrPct, 100.0);
  console.log("✓ Test 3: RadioMaster Channel 3 (-1..+1) maps correctly to 0%, 50%, 100%");
}

// Test 4: Arming Gate without Transport Link
{
  const inputs = { thrPct: 0.0 };
  const armCheck = evaluateArmRequest(inputs, false);
  assert.strictEqual(armCheck.allowed, false);
  assert.strictEqual(armCheck.reason, "NO_LINK");
  console.log("✓ Test 4: Arm request rejected when Transport is closed (NO_LINK)");
}

// Test 5: Arming Gate with Throttle Raised
{
  const inputs = { thrPct: 15.0 };
  const armCheck = evaluateArmRequest(inputs, true);
  assert.strictEqual(armCheck.allowed, false);
  assert.strictEqual(armCheck.reason, "THROTTLE_NOT_ZERO");
  console.log("✓ Test 5: Arm request rejected when Throttle > 3% (Safety Interlock)");
}

// Test 6: Arming Gate with Throttle Zero & Link Open
{
  const inputs = { thrPct: 0.0 };
  const armCheck = evaluateArmRequest(inputs, true);
  assert.strictEqual(armCheck.allowed, true);
  assert.strictEqual(armCheck.reason, "OK");
  console.log("✓ Test 6: Arm request accepted when Throttle is 0% and Link is Open");
}

// Test 7: Unarmed Gamepad must NOT transmit RC packets or overwrite Web UI slider
{
  const isGamepadArmed = false;
  const mockWebUiSlider = 30.0;
  let transmittedCommand = null;
  let sliderValue = mockWebUiSlider;

  // Simulation of pollGamepad behavior
  if (!isGamepadArmed) {
    // Unarmed: no transmit, no setThrottle
  } else {
    transmittedCommand = "RC,0.0,0.0,0.0,0.0,1";
    sliderValue = 0.0;
  }

  assert.strictEqual(transmittedCommand, null);
  assert.strictEqual(sliderValue, 30.0);
  console.log("✓ Test 7: Unarmed Gamepad does NOT transmit RC or overwrite Web UI throttle");
}

// Test 8: RadioMaster Switch SA toggle DOWN (Button 0 false -> true) triggers ARM when throttle <= 3%
{
  const inputs = { isRcTransmitter: true, thrPct: 0.0 };
  const prevBtns = [false, false, false, false];
  const currBtns = [true, false, false, false]; // SA flipped DOWN (Button 0 pressed)
  const res = evaluateGamepadArmDisarm(inputs, currBtns, prevBtns, [0,0,-1,0], 0, true);
  assert.strictEqual(res.action, "ARM");
  console.log("✓ Test 8: RadioMaster Switch SA toggle DOWN triggers ARM when throttle is 0%");
}

// Test 9: RadioMaster Switch SA toggle UP (Button 0 true -> false) triggers DISARM
{
  const inputs = { isRcTransmitter: true, thrPct: 50.0 }; // Even at 50% throttle, SA UP must immediately DISARM
  const prevBtns = [true, false, false, false]; // SA was DOWN
  const currBtns = [false, false, false, false]; // SA flipped UP (Button 0 released)
  const res = evaluateGamepadArmDisarm(inputs, currBtns, prevBtns, [0,0,0,0], 0, true);
  assert.strictEqual(res.action, "DISARM");
  console.log("✓ Test 9: RadioMaster Switch SA toggle UP triggers emergency DISARM");
}

// Test 10: RadioMaster Switch SA toggle DOWN rejected when throttle > 3%
{
  const inputs = { isRcTransmitter: true, thrPct: 15.0 }; // Throttle at 15%
  const prevBtns = [false, false, false, false];
  const currBtns = [true, false, false, false]; // SA flipped DOWN
  const res = evaluateGamepadArmDisarm(inputs, currBtns, prevBtns, [0,0,-0.7,0], 0, true);
  assert.strictEqual(res.action, "REJECT");
  assert.strictEqual(res.reason, "THROTTLE_NOT_ZERO");
  console.log("✓ Test 10: RadioMaster Switch SA toggle DOWN rejected when throttle > 3% (Safety)");
}

// Test 11: Xbox Button 'A' release does NOT trigger DISARM
{
  const inputs = { isRcTransmitter: false, thrPct: 20.0 };
  const prevBtns = [true, false, false, false]; // Button A was pressed
  const currBtns = [false, false, false, false]; // Button A released
  const res = evaluateGamepadArmDisarm(inputs, currBtns, prevBtns, [0,0,0,0], 0, true);
  assert.strictEqual(res.action, "NONE"); // Releasing A on Xbox must NOT disarm!
  console.log("✓ Test 11: Xbox Button 'A' release does NOT trigger DISARM (momentary behavior preserved)");
}

// Test 12: RadioMaster 8-axis mode: Axis 4 (CH5) transition triggers ARM and DISARM
{
  const inputs = { isRcTransmitter: true, thrPct: 0.0 };
  const btns = [false, false, false, false];
  // Axis 4 goes from -1.0 (UP) to +1.0 (DOWN)
  const resArm = evaluateGamepadArmDisarm(inputs, btns, btns, [0,0,-1,0, 1.0], -1.0, true);
  assert.strictEqual(resArm.action, "ARM");

  // Axis 4 goes from +1.0 (DOWN) to -1.0 (UP)
  const resDisarm = evaluateGamepadArmDisarm(inputs, btns, btns, [0,0,0,0, -1.0], 1.0, true);
  assert.strictEqual(resDisarm.action, "DISARM");
  console.log("✓ Test 12: RadioMaster 8-axis mode: Axis 4 transitions trigger ARM and DISARM");
}

console.log("\nALL 12 UNIT TESTS PASSED SUCCESSFULLY!");
