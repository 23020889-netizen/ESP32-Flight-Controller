/**
 * Renders the Primary Flight Display (PFD) artificial horizon indicator.
 */
class ArtificialHorizonRenderer {
  /**
   * Draws the artificial horizon for the given Roll and Pitch angles.
   * @param {HTMLCanvasElement} horizonCanvas Target 2D canvas element.
   * @param {number} rollDegrees Roll angle in degrees.
   * @param {number} pitchDegrees Pitch angle in degrees.
   */
  static renderHorizon(horizonCanvas, rollDegrees, pitchDegrees) {
    const context = horizonCanvas.getContext("2d");
    const canvasSize = 124;
    const centerCoordinate = canvasSize / 2;
    const circleRadius = centerCoordinate - 4;
    const pitchVerticalOffset = pitchDegrees * 1.8;

    context.clearRect(0, 0, canvasSize, canvasSize);
    context.save();
    context.beginPath();
    context.arc(centerCoordinate, centerCoordinate, circleRadius, 0, Math.PI * 2);
    context.clip();
    context.translate(centerCoordinate, centerCoordinate);
    context.rotate((-rollDegrees * Math.PI) / 180);

    context.fillStyle = "#1e6fb0";
    context.fillRect(-canvasSize, -canvasSize * 2 + pitchVerticalOffset, canvasSize * 2, canvasSize * 2);
    context.fillStyle = "#6b4423";
    context.fillRect(-canvasSize, pitchVerticalOffset, canvasSize * 2, canvasSize * 2);

    context.strokeStyle = "#ffffff";
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(-canvasSize, pitchVerticalOffset);
    context.lineTo(canvasSize, pitchVerticalOffset);
    context.stroke();

    context.lineWidth = 1;
    [-20, -10, 10, 20].forEach((pitchLadderAngle) => {
      const yMark = pitchVerticalOffset - pitchLadderAngle * 1.8;
      context.beginPath();
      context.moveTo(-12, yMark);
      context.lineTo(12, yMark);
      context.stroke();
    });
    context.restore();

    context.strokeStyle = "#facc15";
    context.lineWidth = 2.5;
    context.beginPath();
    context.moveTo(centerCoordinate - 26, centerCoordinate);
    context.lineTo(centerCoordinate - 8, centerCoordinate);
    context.moveTo(centerCoordinate + 8, centerCoordinate);
    context.lineTo(centerCoordinate + 26, centerCoordinate);
    context.stroke();

    context.strokeStyle = TelemetryChartRenderer.getThemeVariable("--c1");
    context.lineWidth = 2;
    context.beginPath();
    context.arc(centerCoordinate, centerCoordinate, circleRadius, 0, Math.PI * 2);
    context.stroke();
  }
}

window.ArtificialHorizonRenderer = ArtificialHorizonRenderer;

