import {
  WHEEL_TRACK_CM,
  GRAY_FORWARD_OFFSET_CM,
  LEFT_WHEEL_DIAMETER_CM,
} from '../sim/config';
import type { VehicleSimulator } from '../sim/physics';
import type { TrackRenderer } from '../track/track_renderer';

export class CarRenderer {
  private trackRenderer: TrackRenderer;

  constructor(trackRenderer: TrackRenderer) {
    this.trackRenderer = trackRenderer;
  }

  public render(sim: VehicleSimulator, ctx: CanvasRenderingContext2D): void {
    const scale = this.trackRenderer.transform.scale;
    const { x, y } = this.trackRenderer.toScreen(sim.pose.x, sim.pose.y);
    const heading = sim.pose.heading;

    // 1. Render Trajectory Trail
    this.renderTrail(sim, ctx);

    // 2. Render Car Body in Local Coordinate System
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    const s = scale; // 1 cm = s pixels

    // Dimensions in pixels
    const trackWidthPx = WHEEL_TRACK_CM * s;
    const wheelDiaPx = LEFT_WHEEL_DIAMETER_CM * s;
    const wheelWidthPx = 2.0 * s;
    const sensorOffsetPx = GRAY_FORWARD_OFFSET_CM * s;

    // Chassis plate dimensions
    const chassisWidthPx = 17.0 * s;
    const chassisFrontPx = 26.0 * s;
    const chassisRearPx = 6.0 * s;

    // --- A. Chassis Plate (Translucent Dark Acrylic with Tech Border) ---
    ctx.save();
    ctx.fillStyle = 'rgba(12, 28, 54, 0.88)';
    ctx.strokeStyle = '#00e5ff';
    ctx.lineWidth = Math.max(1, 1.5 * (s / 4.0));
    ctx.shadowColor = 'rgba(0, 229, 255, 0.4)';
    ctx.shadowBlur = 8;

    // Rounded polygon chassis
    ctx.beginPath();
    ctx.moveTo(-chassisRearPx, -chassisWidthPx * 0.45);
    ctx.lineTo(chassisFrontPx * 0.7, -chassisWidthPx * 0.45);
    ctx.lineTo(chassisFrontPx, -chassisWidthPx * 0.28);
    ctx.lineTo(chassisFrontPx, chassisWidthPx * 0.28);
    ctx.lineTo(chassisFrontPx * 0.7, chassisWidthPx * 0.45);
    ctx.lineTo(-chassisRearPx, chassisWidthPx * 0.45);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // --- B. STM32 PCB Core Board Outline ---
    ctx.save();
    const pcbWPx = 9.0 * s;
    const pcbHPx = 10.0 * s;
    const pcbX = 2.0 * s;
    ctx.fillStyle = '#083820'; // Classic STM32 green soldermask
    ctx.strokeStyle = '#00ff88';
    ctx.lineWidth = 1;
    ctx.fillRect(pcbX, -pcbWPx / 2, pcbHPx, pcbWPx);
    ctx.strokeRect(pcbX, -pcbWPx / 2, pcbHPx, pcbWPx);

    // MCU Chip in center of PCB
    const mcuSize = 3.5 * s;
    ctx.fillStyle = '#111111';
    ctx.fillRect(pcbX + pcbHPx / 2 - mcuSize / 2, -mcuSize / 2, mcuSize, mcuSize);
    ctx.fillStyle = '#ffffff';
    ctx.font = `${Math.max(6, 7 * (s / 4.0))}px sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('STM32', pcbX + pcbHPx / 2, 0);

    // TB6612 Motor Driver Module
    ctx.fillStyle = '#004488';
    ctx.fillRect(-4.5 * s, -4.0 * s, 4.0 * s, 8.0 * s);
    ctx.fillStyle = '#aaccff';
    ctx.fillText('TB6612', -2.5 * s, 0);
    ctx.restore();

    // --- C. Drive Wheels (Left & Right) ---
    ctx.save();
    const halfTrackPx = trackWidthPx / 2;

    const drawWheel = (wheelY: number, speed: number) => {
      ctx.save();
      ctx.translate(0, wheelY);

      // Rubber Tire
      ctx.fillStyle = '#1a1a1a';
      ctx.strokeStyle = '#555555';
      ctx.lineWidth = 1;
      ctx.fillRect(-wheelDiaPx / 2, -wheelWidthPx / 2, wheelDiaPx, wheelWidthPx);
      ctx.strokeRect(-wheelDiaPx / 2, -wheelWidthPx / 2, wheelDiaPx, wheelWidthPx);

      // Wheel Rim & Hub
      ctx.fillStyle = '#e0e0e0';
      ctx.beginPath();
      ctx.ellipse(0, 0, wheelDiaPx * 0.35, wheelWidthPx * 0.4, 0, 0, Math.PI * 2);
      ctx.fill();

      // Rotation tread indicator
      ctx.strokeStyle = speed >= 0 ? '#00e5ff' : '#ff3366';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(-wheelDiaPx * 0.25, 0);
      ctx.lineTo(wheelDiaPx * 0.25, 0);
      ctx.stroke();

      ctx.restore();
    };

    // Left Wheel (top in local coordinate: -y)
    drawWheel(-halfTrackPx, sim.actual_left_speed);
    // Right Wheel (bottom in local coordinate: +y)
    drawWheel(halfTrackPx, sim.actual_right_speed);
    ctx.restore();

    // --- D. Front Caster / Ball Wheel ---
    ctx.save();
    ctx.fillStyle = '#bbbbbb';
    ctx.beginPath();
    ctx.arc(chassisFrontPx * 0.85, 0, 1.5 * s, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // --- E. Front Gray Sensor Array Bar (at GRAY_FORWARD_OFFSET_CM = 24 cm) ---
    ctx.save();
    const barWPx = 10.0 * s;
    const barHPx = 1.2 * s;

    // Sensor mounting PCB bar
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1;
    ctx.fillRect(sensorOffsetPx - barHPx / 2, -barWPx / 2, barHPx, barWPx);
    ctx.strokeRect(sensorOffsetPx - barHPx / 2, -barWPx / 2, barHPx, barWPx);

    // Render individual 8 Gray Sensor Photodiodes / LEDs (S0..S7)
    for (let i = 0; i < 8; ++i) {
      const offsetCm = (i - 3.5) * sim.sensor_pitch_cm;
      const sensorY = offsetCm * s;
      const active = (sim.raw_sensor_mask & (1 << i)) !== 0;

      ctx.save();
      ctx.translate(sensorOffsetPx, sensorY);

      if (active) {
        // Active: bright glowing neon amber/green lens
        ctx.fillStyle = '#22c55e';
        ctx.shadowColor = '#22c55e';
        ctx.shadowBlur = 10;
        ctx.beginPath();
        ctx.arc(0, 0, 1.4 * (s / 4.0), 0, Math.PI * 2);
        ctx.fill();

        // Pinpoint white center
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(0, 0, 0.7 * (s / 4.0), 0, Math.PI * 2);
        ctx.fill();
      } else {
        // Inactive: dark translucent lens
        ctx.fillStyle = 'rgba(100, 116, 139, 0.4)';
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.5)';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.arc(0, 0, 1.2 * (s / 4.0), 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    }
    ctx.restore();

    // --- F. Forward Heading Pointer Vector ---
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 235, 59, 0.85)';
    ctx.lineWidth = Math.max(1.5, 2 * (s / 4.0));
    ctx.beginPath();
    ctx.moveTo(sensorOffsetPx + 4 * s, 0);
    ctx.lineTo(sensorOffsetPx + 10 * s, 0);
    ctx.lineTo(sensorOffsetPx + 8 * s, -2 * s);
    ctx.moveTo(sensorOffsetPx + 10 * s, 0);
    ctx.lineTo(sensorOffsetPx + 8 * s, 2 * s);
    ctx.stroke();
    ctx.restore();

    ctx.restore();
  }

  private renderTrail(sim: VehicleSimulator, ctx: CanvasRenderingContext2D): void {
    if (sim.trajectory.length < 2) return;

    ctx.save();
    ctx.beginPath();
    const p0 = this.trackRenderer.toScreen(sim.trajectory[0].x, sim.trajectory[0].y);
    ctx.moveTo(p0.x, p0.y);

    for (let i = 1; i < sim.trajectory.length; ++i) {
      const pt = this.trackRenderer.toScreen(sim.trajectory[i].x, sim.trajectory[i].y);
      ctx.lineTo(pt.x, pt.y);
    }

    ctx.strokeStyle = 'rgba(0, 229, 255, 0.5)';
    ctx.lineWidth = Math.max(2, 3 * (this.trackRenderer.transform.scale / 4.0));
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.restore();
  }
}
