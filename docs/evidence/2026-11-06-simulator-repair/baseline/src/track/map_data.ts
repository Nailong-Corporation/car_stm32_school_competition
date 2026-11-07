/**
 * Track Map Geometry & Electrical Components
 * Dimensions: 375.00 cm x 220.00 cm
 * Exact reproduction of the CAD blueprint and schematic.
 */

export interface Point2D {
  x: number;
  y: number;
}

export interface TrackSegment {
  id: string;
  p1: Point2D;
  p2: Point2D;
  dashed?: boolean;
  dashLength?: number;
  gapLength?: number;
  width?: number; // Line width in cm (default 2.5 cm)
}

export interface CircuitComponent {
  id: string;
  type: 'battery' | 'resistor' | 'transistor' | 'diode';
  x: number;
  y: number;
  rotation: number; // in radians
  label: string;
  elementNumber?: number;
  size?: number;
}

export const TRACK_WIDTH_CM = 375.0;
export const TRACK_HEIGHT_CM = 220.0;
export const TRACK_LINE_WIDTH_CM = 2.8;

// Start / Finish Point P
export const START_POINT: Point2D = { x: 0.0, y: 120.0 };
export const START_HEADING_RAD = -Math.PI / 2; // Facing North (-y direction)

export const HEXAGON_CENTER: Point2D = { x: 310.0, y: 170.0 };
export const HEXAGON_SIDE_CM = 35.0;
const HEX_DY = (HEXAGON_SIDE_CM * Math.sqrt(3)) / 2; // ~30.3109

export const HEXAGON_VERTICES: Point2D[] = [
  { x: 345.0, y: 170.0 },                      // Right vertex (entry/exit point)
  { x: 327.5, y: 170.0 - HEX_DY },             // Top-right
  { x: 292.5, y: 170.0 - HEX_DY },             // Top-left
  { x: 275.0, y: 170.0 },                      // Left vertex
  { x: 292.5, y: 170.0 + HEX_DY },             // Bottom-left
  { x: 327.5, y: 170.0 + HEX_DY },             // Bottom-right
];

export const TRACK_SEGMENTS: TrackSegment[] = [
  // Outer Border (solid lines)
  { id: 'top_border', p1: { x: 0, y: 0 }, p2: { x: 375, y: 0 } },
  { id: 'right_border', p1: { x: 375, y: 0 }, p2: { x: 375, y: 220 } },
  { id: 'bottom_border', p1: { x: 375, y: 220 }, p2: { x: 0, y: 220 } },
  { id: 'left_border_top', p1: { x: 0, y: 0 }, p2: { x: 0, y: 120 } },
  { id: 'left_border_bottom', p1: { x: 0, y: 120 }, p2: { x: 0, y: 220 } },

  // Internal Vertical Lines
  { id: 'v_55_top', p1: { x: 55, y: 0 }, p2: { x: 55, y: 120 } },
  {
    id: 'v_55_dashed',
    p1: { x: 55, y: 120 },
    p2: { x: 55, y: 220 },
    dashed: true,
    dashLength: 5.0,
    gapLength: 5.0,
  },
  { id: 'v_95_top', p1: { x: 95, y: 0 }, p2: { x: 95, y: 40 } },
  { id: 'v_95_mid', p1: { x: 95, y: 40 }, p2: { x: 95, y: 120 } },
  { id: 'v_95_bot', p1: { x: 95, y: 170 }, p2: { x: 95, y: 220 } },
  { id: 'v_150_bot', p1: { x: 150, y: 170 }, p2: { x: 150, y: 220 } },
  { id: 'v_165_mid', p1: { x: 165, y: 40 }, p2: { x: 165, y: 80 } },
  { id: 'v_245', p1: { x: 245, y: 0 }, p2: { x: 245, y: 220 } },
  { id: 'v_275', p1: { x: 275, y: 0 }, p2: { x: 275, y: 120 } },
  { id: 'v_345_top', p1: { x: 345, y: 0 }, p2: { x: 345, y: 50 } },
  { id: 'v_345_to_hex', p1: { x: 345, y: 120 }, p2: { x: 345, y: 170 } },
  { id: 'v_345_from_hex', p1: { x: 345, y: 170 }, p2: { x: 345, y: 220 } },

  // Internal Horizontal Lines
  { id: 'h_40_mid', p1: { x: 95, y: 40 }, p2: { x: 245, y: 40 } },
  { id: 'h_50_right', p1: { x: 275, y: 50 }, p2: { x: 345, y: 50 } },
  { id: 'h_80_mid', p1: { x: 165, y: 80 }, p2: { x: 245, y: 80 } },
  { id: 'h_120_left', p1: { x: 55, y: 120 }, p2: { x: 245, y: 120 } },
  { id: 'h_120_right', p1: { x: 245, y: 120 }, p2: { x: 375, y: 120 } },
  { id: 'h_170_mid', p1: { x: 95, y: 170 }, p2: { x: 245, y: 170 } },

  // Hexagon Ring Edges (Counter-clockwise: 0->1->2->3->4->5->0)
  { id: 'hex_0_1', p1: HEXAGON_VERTICES[0], p2: HEXAGON_VERTICES[1] },
  { id: 'hex_1_2', p1: HEXAGON_VERTICES[1], p2: HEXAGON_VERTICES[2] },
  { id: 'hex_2_3', p1: HEXAGON_VERTICES[2], p2: HEXAGON_VERTICES[3] },
  { id: 'hex_3_4', p1: HEXAGON_VERTICES[3], p2: HEXAGON_VERTICES[4] },
  { id: 'hex_4_5', p1: HEXAGON_VERTICES[4], p2: HEXAGON_VERTICES[5] },
  { id: 'hex_5_0', p1: HEXAGON_VERTICES[5], p2: HEXAGON_VERTICES[0] },
];

export const CIRCUIT_COMPONENTS: CircuitComponent[] = [
  // Battery / Power Supply at Start point P(0, 120)
  { id: 'bat1', type: 'battery', x: 0, y: 120, rotation: Math.PI / 2, label: 'BAT (P起点)', elementNumber: 0 },

  // 27 Components numbered per route_table.c
  { id: 'r1', type: 'resistor', x: 0, y: 45, rotation: Math.PI / 2, label: 'R1 (元件1)', elementNumber: 1 },
  { id: 'r2', type: 'resistor', x: 27.5, y: 0, rotation: 0, label: 'R2 (元件2)', elementNumber: 2 },
  { id: 'r3', type: 'resistor', x: 55, y: 65, rotation: Math.PI / 2, label: 'R3 (元件3)', elementNumber: 3 },
  { id: 'q4', type: 'transistor', x: 95, y: 120, rotation: 0, label: 'Q4 (节点4)', elementNumber: 4 },
  { id: 'r5', type: 'resistor', x: 170, y: 120, rotation: 0, label: 'R5 (元件5)', elementNumber: 5 },
  { id: 'q6', type: 'transistor', x: 245, y: 120, rotation: 0, label: 'Q6 (节点6)', elementNumber: 6 },
  { id: 'd7', type: 'diode', x: 150, y: 170, rotation: Math.PI, label: 'D7 (二极管7)', elementNumber: 7 },
  { id: 'r8', type: 'resistor', x: 122.5, y: 220, rotation: 0, label: 'R8 (元件8)', elementNumber: 8 },
  { id: 'r9', type: 'resistor', x: 197.5, y: 220, rotation: 0, label: 'R9 (元件9)', elementNumber: 9 },
  { id: 'r10', type: 'resistor', x: 295, y: 220, rotation: 0, label: 'R10 (元件10)', elementNumber: 10 },
  { id: 'q11', type: 'transistor', x: 345, y: 220, rotation: -Math.PI / 2, label: 'Q11 (三极管11)', elementNumber: 11 },
  { id: 'r12', type: 'resistor', x: 375, y: 170, rotation: Math.PI / 2, label: 'R12 (元件12)', elementNumber: 12 },
  { id: 'q13', type: 'transistor', x: 345, y: 120, rotation: Math.PI, label: 'Q13 (三极管13)', elementNumber: 13 },
  { id: 'r14', type: 'resistor', x: 310, y: 170.0 + HEX_DY, rotation: 0, label: 'R14 (环岛元件14)', elementNumber: 14 },
  { id: 'r15', type: 'resistor', x: 375, y: 65, rotation: Math.PI / 2, label: 'R15 (元件15)', elementNumber: 15 },
  { id: 'q16', type: 'transistor', x: 345, y: 0, rotation: Math.PI / 2, label: 'Q16 (三极管16)', elementNumber: 16 },
  { id: 'r17', type: 'resistor', x: 310, y: 0, rotation: 0, label: 'R17 (元件17)', elementNumber: 17 },
  { id: 'q18', type: 'transistor', x: 275, y: 50, rotation: 0, label: 'Q18 (三极管18)', elementNumber: 18 },
  { id: 'r19', type: 'resistor', x: 310, y: 50, rotation: 0, label: 'R19 (元件19)', elementNumber: 19 },
  { id: 'r20', type: 'resistor', x: 215, y: 0, rotation: 0, label: 'R20 (元件20)', elementNumber: 20 },
  { id: 'r21', type: 'resistor', x: 145, y: 0, rotation: 0, label: 'R21 (元件21)', elementNumber: 21 },
  { id: 'q22', type: 'transistor', x: 95, y: 0, rotation: Math.PI / 2, label: 'Q22 (三极管22)', elementNumber: 22 },
  { id: 'r23', type: 'resistor', x: 130, y: 40, rotation: 0, label: 'R23 (元件23)', elementNumber: 23 },
  { id: 'r24', type: 'resistor', x: 205, y: 80, rotation: 0, label: 'R24 (元件24)', elementNumber: 24 },
  { id: 'r25', type: 'resistor', x: 205, y: 40, rotation: 0, label: 'R25 (元件25)', elementNumber: 25 },
  { id: 'r26', type: 'resistor', x: 55, y: 170, rotation: Math.PI / 2, label: 'R26 (虚线元件26)', elementNumber: 26 },
  { id: 'r27', type: 'resistor', x: 0, y: 170, rotation: Math.PI / 2, label: 'R27 (元件27)', elementNumber: 27 },
];

/**
 * Distance from point (px, py) to line segment (p1 -> p2)
 */
export function pointToSegmentDistance(
  px: number,
  py: number,
  x1: number,
  y1: number,
  x2: number,
  y2: number
): { distance: number; t: number; projX: number; projY: number } {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) {
    const d = Math.hypot(px - x1, py - y1);
    return { distance: d, t: 0, projX: x1, projY: y1 };
  }
  let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const projX = x1 + t * dx;
  const projY = y1 + t * dy;
  const distance = Math.hypot(px - projX, py - projY);
  return { distance, t, projX, projY };
}

/**
 * Checks whether a given sensor point (sx, sy) in cm detects the track line
 */
export function isSensorOnTrackLine(
  sx: number,
  sy: number,
  halfWidth = TRACK_LINE_WIDTH_CM / 2
): boolean {
  for (let i = 0; i < TRACK_SEGMENTS.length; ++i) {
    const seg = TRACK_SEGMENTS[i];
    const { distance, t } = pointToSegmentDistance(
      sx,
      sy,
      seg.p1.x,
      seg.p1.y,
      seg.p2.x,
      seg.p2.y
    );

    if (distance <= halfWidth) {
      if (seg.dashed) {
        const segLen = Math.hypot(seg.p2.x - seg.p1.x, seg.p2.y - seg.p1.y);
        const distAlong = t * segLen;
        const period = (seg.dashLength || 5.0) + (seg.gapLength || 5.0);
        const mod = distAlong % period;
        if (mod > (seg.dashLength || 5.0)) {
          // In the gap of dashed line
          continue;
        }
      }
      return true;
    }
  }
  return false;
}
