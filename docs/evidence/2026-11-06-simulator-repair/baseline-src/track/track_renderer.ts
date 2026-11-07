import {
  TRACK_WIDTH_CM,
  TRACK_HEIGHT_CM,
  TRACK_LINE_WIDTH_CM,
  TRACK_SEGMENTS,
  CIRCUIT_COMPONENTS,
  type CircuitComponent,
} from './map_data';

export interface ViewportTransform {
  scale: number;
  offsetX: number;
  offsetY: number;
  width: number;
  height: number;
}

export interface RenderOptions {
  showDimensions: boolean;
  showComponentLabels: boolean;
  showGrid: boolean;
  highContrast: boolean;
}

export class TrackRenderer {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;

  public options: RenderOptions = {
    showDimensions: true,
    showComponentLabels: true,
    showGrid: true,
    highContrast: false,
  };

  public transform: ViewportTransform = {
    scale: 1,
    offsetX: 0,
    offsetY: 0,
    width: 800,
    height: 600,
  };

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const context = canvas.getContext('2d');
    if (!context) throw new Error('Cannot get 2D canvas context');
    this.ctx = context;
  }

  /**
   * Automatically compute scale and offset to fill the viewport
   */
  public fitViewport(paddingPx = 40): void {
    const w = this.canvas.width;
    const h = this.canvas.height;
    this.transform.width = w;
    this.transform.height = h;

    const availW = w - paddingPx * 2;
    const availH = h - paddingPx * 2;

    const scaleX = availW / TRACK_WIDTH_CM;
    const scaleY = availH / TRACK_HEIGHT_CM;
    const scale = Math.min(scaleX, scaleY);

    this.transform.scale = scale;
    this.transform.offsetX = (w - TRACK_WIDTH_CM * scale) / 2;
    this.transform.offsetY = (h - TRACK_HEIGHT_CM * scale) / 2;
  }

  public toScreen(xCm: number, yCm: number): { x: number; y: number } {
    return {
      x: this.transform.offsetX + xCm * this.transform.scale,
      y: this.transform.offsetY + yCm * this.transform.scale,
    };
  }

  public toWorld(screenX: number, screenY: number): { x: number; y: number } {
    return {
      x: (screenX - this.transform.offsetX) / this.transform.scale,
      y: (screenY - this.transform.offsetY) / this.transform.scale,
    };
  }

  public render(): void {
    const ctx = this.ctx;
    const { width, height } = this.canvas;

    ctx.save();
    ctx.clearRect(0, 0, width, height);

    // 1. Blueprint Deep Blue Background (Exact shade of Image 1: deep royal circuit blue)
    const bgGrad = ctx.createLinearGradient(0, 0, width, height);
    bgGrad.addColorStop(0, '#020b66');
    bgGrad.addColorStop(1, '#01084d');
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, width, height);

    // Track frame background rectangle
    const p0 = this.toScreen(0, 0);
    const pEnd = this.toScreen(TRACK_WIDTH_CM, TRACK_HEIGHT_CM);
    const trackW = pEnd.x - p0.x;
    const trackH = pEnd.y - p0.y;

    ctx.fillStyle = '#020d75';
    ctx.fillRect(p0.x, p0.y, trackW, trackH);

    // 2. Blueprint Grid Lines (every 20 cm)
    if (this.options.showGrid) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.lineWidth = 1;
      for (let x = 0; x <= TRACK_WIDTH_CM; x += 20) {
        const top = this.toScreen(x, 0);
        const bot = this.toScreen(x, TRACK_HEIGHT_CM);
        ctx.beginPath();
        ctx.moveTo(top.x, top.y);
        ctx.lineTo(bot.x, bot.y);
        ctx.stroke();
      }
      for (let y = 0; y <= TRACK_HEIGHT_CM; y += 20) {
        const left = this.toScreen(0, y);
        const right = this.toScreen(TRACK_WIDTH_CM, y);
        ctx.beginPath();
        ctx.moveTo(left.x, left.y);
        ctx.lineTo(right.x, right.y);
        ctx.stroke();
      }
      ctx.restore();
    }

    // Outer border frame (border border double line)
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.lineWidth = 2;
    ctx.strokeRect(p0.x - 8, p0.y - 8, trackW + 16, trackH + 16);
    ctx.restore();

    // 3. Render White Track Line Segments
    ctx.save();
    const lineWidthPx = Math.max(2, TRACK_LINE_WIDTH_CM * this.transform.scale);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (let i = 0; i < TRACK_SEGMENTS.length; ++i) {
      const seg = TRACK_SEGMENTS[i];
      const pt1 = this.toScreen(seg.p1.x, seg.p1.y);
      const pt2 = this.toScreen(seg.p2.x, seg.p2.y);

      ctx.beginPath();
      ctx.moveTo(pt1.x, pt1.y);
      ctx.lineTo(pt2.x, pt2.y);

      if (seg.dashed) {
        const dashPx = (seg.dashLength || 5.0) * this.transform.scale;
        const gapPx = (seg.gapLength || 5.0) * this.transform.scale;
        ctx.setLineDash([dashPx, gapPx]);
      } else {
        ctx.setLineDash([]);
      }

      ctx.lineWidth = lineWidthPx;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    }
    ctx.restore();

    // 4. Render Circuit Component Symbols (Resistors, Transistors, Battery, Diode)
    for (let i = 0; i < CIRCUIT_COMPONENTS.length; ++i) {
      this.renderComponent(CIRCUIT_COMPONENTS[i]);
    }

    // 5. Render CAD Dimension Annotations (matching Image 2)
    if (this.options.showDimensions) {
      this.renderDimensions();
    }

    ctx.restore();
  }

  private renderComponent(comp: CircuitComponent): void {
    const ctx = this.ctx;
    const { x, y } = this.toScreen(comp.x, comp.y);
    const scale = this.transform.scale;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(comp.rotation);

    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#020d75'; // Matches track background so symbols pop cleanly
    ctx.lineWidth = Math.max(1.5, 2 * (scale / 4.0));

    if (comp.type === 'battery') {
      // Battery symbol at (0, 120): 4 parallel plates (Long, Short, Long, Short)
      const plateH = 24 * (scale / 4.0);
      const spacing = 4 * (scale / 4.0);

      // Plate 1 (Long, positive)
      ctx.beginPath();
      ctx.moveTo(-spacing * 1.5, -plateH * 0.7);
      ctx.lineTo(-spacing * 1.5, plateH * 0.7);
      ctx.stroke();

      // Plate 2 (Short, negative)
      ctx.beginPath();
      ctx.lineWidth = Math.max(2, 3 * (scale / 4.0));
      ctx.moveTo(-spacing * 0.5, -plateH * 0.35);
      ctx.lineTo(-spacing * 0.5, plateH * 0.35);
      ctx.stroke();

      // Plate 3 (Long)
      ctx.lineWidth = Math.max(1.5, 2 * (scale / 4.0));
      ctx.beginPath();
      ctx.moveTo(spacing * 0.5, -plateH * 0.7);
      ctx.lineTo(spacing * 0.5, plateH * 0.7);
      ctx.stroke();

      // Plate 4 (Short)
      ctx.beginPath();
      ctx.lineWidth = Math.max(2, 3 * (scale / 4.0));
      ctx.moveTo(spacing * 1.5, -plateH * 0.35);
      ctx.lineTo(spacing * 1.5, plateH * 0.35);
      ctx.stroke();

      // Start text indicator
      ctx.save();
      ctx.rotate(-comp.rotation);
      ctx.fillStyle = '#00ffcc';
      ctx.font = `bold ${Math.max(10, 12 * (scale / 4.0))}px sans-serif`;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText('P (START)', -16 * (scale / 4.0), 0);
      ctx.restore();
    } else if (comp.type === 'resistor') {
      // Rectangular box with line passing through
      const boxW = 22 * (scale / 4.0);
      const boxH = 10 * (scale / 4.0);

      ctx.beginPath();
      ctx.rect(-boxW / 2, -boxH / 2, boxW, boxH);
      ctx.fill();
      ctx.stroke();
    } else if (comp.type === 'transistor') {
      // BJT Transistor symbol (base bar, collector lead, emitter lead with arrow)
      const r = 10 * (scale / 4.0);

      // Base bar
      ctx.beginPath();
      ctx.moveTo(-r * 0.5, -r);
      ctx.lineTo(-r * 0.5, r);
      ctx.stroke();

      // Collector lead
      ctx.beginPath();
      ctx.moveTo(-r * 0.5, -r * 0.5);
      ctx.lineTo(r, -r);
      ctx.stroke();

      // Emitter lead with arrow
      ctx.beginPath();
      ctx.moveTo(-r * 0.5, r * 0.5);
      ctx.lineTo(r, r);
      ctx.stroke();

      // Emitter arrow
      ctx.beginPath();
      ctx.moveTo(r, r);
      ctx.lineTo(r * 0.4, r * 0.85);
      ctx.lineTo(r * 0.85, r * 0.4);
      ctx.closePath();
      ctx.fillStyle = '#ffffff';
      ctx.fill();
    } else if (comp.type === 'diode') {
      // Diode triangle + bar
      const size = 10 * (scale / 4.0);
      ctx.beginPath();
      ctx.moveTo(-size, -size);
      ctx.lineTo(size, 0);
      ctx.lineTo(-size, size);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(size, -size);
      ctx.lineTo(size, size);
      ctx.stroke();
    }

    // Component Label (optional)
    if (this.options.showComponentLabels && comp.label) {
      ctx.restore();
      ctx.save();
      ctx.translate(x, y);
      ctx.fillStyle = 'rgba(255, 255, 255, 0.65)';
      ctx.font = `${Math.max(9, 10 * (scale / 4.0))}px monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(comp.label, 0, 14 * (scale / 4.0));
      ctx.restore();
      return;
    }

    ctx.restore();
  }

  private renderDimensions(): void {
    const ctx = this.ctx;
    const scale = this.transform.scale;
    const fontSize = Math.max(9, 11 * (scale / 4.0));

    ctx.save();
    ctx.font = `${fontSize}px sans-serif`;
    ctx.fillStyle = '#8bc34a'; // Soft blueprint green/amber for dimension text
    ctx.strokeStyle = 'rgba(139, 195, 74, 0.4)';
    ctx.lineWidth = 1;

    // Helper to draw horizontal dimension line with arrows and text
    const drawHorizDim = (x1: number, x2: number, y: number, text: string, textAbove = true) => {
      const p1 = this.toScreen(x1, y);
      const p2 = this.toScreen(x2, y);

      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();

      // Arrow heads
      const arrowS = 3;
      ctx.beginPath();
      ctx.moveTo(p1.x + arrowS, p1.y - arrowS);
      ctx.lineTo(p1.x, p1.y);
      ctx.lineTo(p1.x + arrowS, p1.y + arrowS);
      ctx.moveTo(p2.x - arrowS, p2.y - arrowS);
      ctx.lineTo(p2.x, p2.y);
      ctx.lineTo(p2.x - arrowS, p2.y + arrowS);
      ctx.stroke();

      ctx.textAlign = 'center';
      ctx.textBaseline = textAbove ? 'bottom' : 'top';
      ctx.fillText(text, (p1.x + p2.x) / 2, p1.y + (textAbove ? -2 : 2));
    };

    // Helper to draw vertical dimension line with arrows and text
    const drawVertDim = (x: number, y1: number, y2: number, text: string, textLeft = true) => {
      const p1 = this.toScreen(x, y1);
      const p2 = this.toScreen(x, y2);

      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();

      const arrowS = 3;
      ctx.beginPath();
      ctx.moveTo(p1.x - arrowS, p1.y + arrowS);
      ctx.lineTo(p1.x, p1.y);
      ctx.lineTo(p1.x + arrowS, p1.y + arrowS);
      ctx.moveTo(p2.x - arrowS, p2.y - arrowS);
      ctx.lineTo(p2.x, p2.y);
      ctx.lineTo(p2.x + arrowS, p2.y - arrowS);
      ctx.stroke();

      ctx.textAlign = textLeft ? 'right' : 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, p1.x + (textLeft ? -4 : 4), (p1.y + p2.y) / 2);
    };

    // Top Dimension Chain (y = -10 cm)
    const topY = -8;
    drawHorizDim(0, 55, topY, '55.00');
    drawHorizDim(55, 95, topY, '40.00');
    drawHorizDim(95, 245, topY, '150.00');
    drawHorizDim(245, 275, topY, '30.00');
    drawHorizDim(275, 345, topY, '70.00');
    drawHorizDim(345, 375, topY, '30.00');
    // Total Width top
    drawHorizDim(0, 375, topY - 10, '375.00');

    // Left Dimension Chain (x = -8 cm)
    const leftX = -8;
    drawVertDim(leftX, 0, 120, '120.00');
    drawVertDim(leftX, 120, 220, '100.00');
    drawVertDim(leftX - 10, 0, 220, '220.00');

    // Middle Box Dimensions (y = 40..80, x = 165..245)
    drawHorizDim(165, 245, 34, '80.00');
    drawVertDim(160, 40, 80, '40.00');
    drawVertDim(160, 80, 120, '40.00');

    // Hexagon Dimensions
    drawHorizDim(292.5, 327.5, 134, '35.00');
    drawVertDim(350, 120, 170, '50.00', false);
    drawVertDim(382, 120, 220, '100.00', false);

    // Bottom Dimension Chain (y = 228 cm)
    const botY = 228;
    drawHorizDim(0, 55, botY, '55.00', false);
    drawHorizDim(55, 95, botY, '40.00', false);
    drawHorizDim(95, 150, botY, '55.00', false);
    drawHorizDim(150, 245, botY, '95.00', false);
    drawHorizDim(245, 345, botY, '100.00', false);
    drawHorizDim(345, 375, botY, '30.00', false);

    ctx.restore();
  }
}
