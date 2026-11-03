/**
 * Parses and replays recorded IMU CSV telemetry logs in real time.
 */
class CsvTelemetryReplayer {
  /**
   * Initializes the CSV replayer with a target TelemetryDataStore instance.
   * @param {TelemetryDataStore} telemetryStore Data store receiving replayed samples.
   */
  constructor(telemetryStore) {
    this.telemetryStore = telemetryStore;
    this.parsedRecords = [];
    this.currentRecordIndex = 0;
    this.playbackStartTimestampSeconds = 0;
    this.playbackTimerIdentifier = null;
    if (window.DEFAULT_IMU_CSV_LOG) {
      this.parseCsvContent(window.DEFAULT_IMU_CSV_LOG);
    }
  }

  /**
   * Parses CSV text containing IMU columns into normalized telemetry records.
   * @param {string} csvText Raw CSV file content.
   */
  parseCsvContent(csvText) {
    const lines = csvText.trim().split(/\r?\n/);
    if (lines.length < 2) return;
    const indexMap = {};
    lines[0].split(",").forEach((columnName, index) => {
      indexMap[columnName.trim()] = index;
    });
    this.parsedRecords = [];
    for (let lineIndex = 1; lineIndex < lines.length; lineIndex++) {
      const cells = lines[lineIndex].split(",");
      if (cells.length < 6) continue;
      const elapsedSeconds = parseFloat(cells[indexMap.Time_Elapsed_s]) || 0;
      const roll = parseFloat(cells[indexMap.Roll_deg]) || 0;
      const pitch = parseFloat(cells[indexMap.Pitch_deg]) || 0;
      const yaw = parseFloat(cells[indexMap.Yaw_deg]) || 0;
      const accZ = parseFloat(cells[indexMap.Acc_Z_ms2]) || 9.81;
      const gyrX = parseFloat(cells[indexMap.Gyr_X_dps]) || 0;
      const gyrY = parseFloat(cells[indexMap.Gyr_Y_dps]) || 0;
      const gyrZ = parseFloat(cells[indexMap.Gyr_Z_dps]) || 0;
      const baseSpeed = 760;
      const rollMix = roll * 1.4 + gyrX * 0.25;
      const pitchMix = pitch * 1.4 + gyrY * 0.25;
      const yawMix = gyrZ * 0.2;
      const clampSpeed = (value) => Math.max(0, Math.min(1000, value));
      this.parsedRecords.push({
        elapsedSeconds, roll, pitch, yaw,
        z: Math.max(0, 1.5 + (accZ - 9.81) * 0.08),
        vz: (accZ - 9.81) * 0.15,
        m: [
          clampSpeed(baseSpeed - rollMix + pitchMix - yawMix),
          clampSpeed(baseSpeed + rollMix - pitchMix - yawMix),
          clampSpeed(baseSpeed + rollMix + pitchMix + yawMix),
          clampSpeed(baseSpeed - rollMix - pitchMix + yawMix)
        ]
      });
    }
  }

  /**
   * Starts real-time looped playback of the loaded CSV telemetry records.
   */
  startPlayback() {
    this.stopPlayback();
    if (this.parsedRecords.length === 0) return;
    this.telemetryStore.connectionMode = "csv";
    if (this.telemetryStore.webSocketConnection) {
      this.telemetryStore.webSocketConnection.close();
    }
    this.currentRecordIndex = 0;
    this.playbackStartTimestampSeconds = performance.now() / 1000 - this.parsedRecords[0].elapsedSeconds;
    this.playbackTimerIdentifier = setInterval(() => this.advancePlaybackTick(), 15);
  }

  /**
   * Emits all CSV records whose elapsed timestamp has been reached.
   */
  advancePlaybackTick() {
    if (this.telemetryStore.connectionMode !== "csv") {
      this.stopPlayback();
      return;
    }
    const playbackElapsed = performance.now() / 1000 - this.playbackStartTimestampSeconds;
    while (this.currentRecordIndex < this.parsedRecords.length && this.parsedRecords[this.currentRecordIndex].elapsedSeconds <= playbackElapsed) {
      this.telemetryStore.pushSample(this.parsedRecords[this.currentRecordIndex]);
      this.currentRecordIndex += 1;
    }
    if (this.currentRecordIndex >= this.parsedRecords.length) {
      this.currentRecordIndex = 0;
      this.playbackStartTimestampSeconds = performance.now() / 1000 - this.parsedRecords[0].elapsedSeconds;
    }
  }

  /**
   * Stops the active CSV replay timer.
   */
  stopPlayback() {
    if (this.playbackTimerIdentifier) clearInterval(this.playbackTimerIdentifier);
    this.playbackTimerIdentifier = null;
  }
}

window.CsvTelemetryReplayer = CsvTelemetryReplayer;
