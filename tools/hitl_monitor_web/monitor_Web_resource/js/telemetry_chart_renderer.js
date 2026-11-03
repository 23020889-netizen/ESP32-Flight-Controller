/**
 * High-performance 2D time-series telemetry waveforms renderer.
 * Optimized with cached theme styles, batch path drawing, and zero layout thrashing.
 */
class TelemetryChartRenderer {
  static themeCache = {
    "--c1": "#38bdf8",
    "--c2": "#fbbf24",
    "--c3": "#a855f7",
    "--c4": "#22c55e",
    "--mu": "rgba(148, 163, 184, 0.6)",
    "--bd": "rgba(56, 189, 248, 0.2)"
  };

  /**
   * Resolves a CSS custom property with static caching to prevent DOM layout reflow.
   */
  static getThemeVariable(variableName) {
    if (!TelemetryChartRenderer.themeCache[variableName]) {
      const val = getComputedStyle(document.documentElement).getPropertyValue(variableName).trim();
      TelemetryChartRenderer.themeCache[variableName] = val || "#38bdf8";
    }
    return TelemetryChartRenderer.themeCache[variableName];
  }

  /**
   * Draws multi-channel time-series lines onto a target canvas element.
   */
  static renderTimeSeriesChart(canvasElement, sampleBuffer, timeSpanSeconds, seriesDefinitions, minimumValue, maximumValue) {
    if (!canvasElement) return;
    const pixelRatio = window.devicePixelRatio || 1;
    const width = canvasElement.clientWidth;
    const height = canvasElement.clientHeight;
    if (!width || !height) return;

    if (canvasElement.width !== width * pixelRatio || canvasElement.height !== height * pixelRatio) {
      canvasElement.width = width * pixelRatio;
      canvasElement.height = height * pixelRatio;
    }

    const currentTimeSeconds = performance.now() / 1000;
    let lowBound = minimumValue;
    let highBound = maximumValue;

    for (let i = Math.max(0, sampleBuffer.length - 120); i < sampleBuffer.length; i++) {
      const sample = sampleBuffer[i];
      if (currentTimeSeconds - sample.t > timeSpanSeconds) continue;
      for (const seriesItem of seriesDefinitions) {
        const sampleValue = seriesItem.extractValue(sample);
        if (sampleValue < lowBound) lowBound = Math.floor(sampleValue / 10) * 10;
        if (sampleValue > highBound) highBound = Math.ceil(sampleValue / 10) * 10;
      }
    }

    const context = canvasElement.getContext("2d");
    context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
    context.clearRect(0, 0, width, height);

    TelemetryChartRenderer.drawBackgroundGrid(context, width, height, lowBound, highBound);

    const range = Math.max(0.001, highBound - lowBound);
    seriesDefinitions.forEach((seriesItem) => {
      const strokeColor = TelemetryChartRenderer.getThemeVariable(seriesItem.colorVariable);
      context.strokeStyle = strokeColor;
      context.lineWidth = 1.8;
      context.beginPath();

      let isFirstPoint = true;
      for (let i = Math.max(0, sampleBuffer.length - 120); i < sampleBuffer.length; i++) {
        const sample = sampleBuffer[i];
        const ageSeconds = currentTimeSeconds - sample.t;
        if (ageSeconds > timeSpanSeconds) continue;
        const xCoordinate = width * (1 - ageSeconds / timeSpanSeconds);
        const yCoordinate = height - ((seriesItem.extractValue(sample) - lowBound) / range) * height;

        if (isFirstPoint) {
          context.moveTo(xCoordinate, yCoordinate);
          isFirstPoint = false;
        } else {
          context.lineTo(xCoordinate, yCoordinate);
        }
      }
      context.stroke();
    });
  }

  /**
   * Draws light background grid lines and scale indicators.
   */
  static drawBackgroundGrid(context, width, height, lowBound, highBound) {
    context.strokeStyle = "rgba(56, 189, 248, 0.12)";
    context.lineWidth = 1;
    context.beginPath();

    const midY = height / 2;
    context.moveTo(0, midY);
    context.lineTo(width, midY);
    context.stroke();

    context.fillStyle = "rgba(148, 163, 184, 0.55)";
    context.font = "10px monospace";
    context.fillText(String(highBound), 4, 12);
    context.fillText(String(lowBound), 4, height - 4);
  }
}

window.TelemetryChartRenderer = TelemetryChartRenderer;
