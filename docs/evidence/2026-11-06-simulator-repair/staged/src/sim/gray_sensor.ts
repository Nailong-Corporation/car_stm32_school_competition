import {
  GrayKind,
  type GrayObservation,
} from './types';
import {
  SENSOR_FILTER_SAMPLES,
  SENSOR_MAX_REGION_WIDTH,
  SENSOR_CONTINUITY_DISTANCE,
  SENSOR_CONTINUITY_MARGIN,
  GRAY_INVERT_ENABLED,
  FLOOR_SEAM_FILTER_ENABLED,
} from './config';

const WEIGHTS = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];

export class GraySensorModel {
  public history: number[] = new Array(SENSOR_FILTER_SAMPLES).fill(0);
  public next = 0;
  public count = 0;

  public reset(): void {
    this.history.fill(0);
    this.next = 0;
    this.count = 0;
  }
}

export function grayClassify(
  line_mask: number,
  have_previous: boolean,
  previous: number
): GrayObservation {
  const result: GrayObservation = {
    kind: GrayKind.GRAY_NONE,
    raw_mask: line_mask,
    line_mask: line_mask,
    selected_mask: 0,
    white_count: 0,
    region_count: 0,
    position: 0,
    prefilter_mask: line_mask,
    seam_mask: 0,
  };

  const masks: number[] = [0, 0, 0, 0];
  const widths: number[] = [0, 0, 0, 0];
  const positions: number[] = [0, 0, 0, 0];
  let in_region = false;
  let wide = false;

  for (let i = 0; i < 8; ++i) {
    if ((line_mask & (1 << i)) === 0) {
      in_region = false;
      continue;
    }
    if (!in_region) {
      result.region_count++;
      in_region = true;
    }
    const region = result.region_count - 1;
    masks[region] |= 1 << i;
    widths[region]++;
    result.white_count++;
    positions[region] += WEIGHTS[i];
  }

  if (result.white_count === 0) return result;

  for (let i = 0; i < result.region_count; ++i) {
    positions[i] /= widths[i];
    if (widths[i] > SENSOR_MAX_REGION_WIDTH) wide = true;
  }

  result.kind = GrayKind.GRAY_AMBIGUOUS;
  if (wide) return result;

  let selected = 0;
  if (result.region_count > 1) {
    if (!have_previous || !Number.isFinite(previous)) return result;
    let best = 100.0;
    let second = 100.0;
    for (let i = 0; i < result.region_count; ++i) {
      const distance = Math.abs(positions[i] - previous);
      if (distance < best) {
        second = best;
        best = distance;
        selected = i;
      } else if (distance < second) {
        second = distance;
      }
    }
    if (
      best > SENSOR_CONTINUITY_DISTANCE ||
      second - best < SENSOR_CONTINUITY_MARGIN
    ) {
      return result;
    }
  }

  result.kind = GrayKind.GRAY_VALID;
  result.selected_mask = masks[selected];
  result.position = positions[selected];
  return result;
}

export function graySensorUpdate(
  sensor: GraySensorModel,
  raw_mask: number,
  have_previous: boolean,
  previous: number
): GrayObservation {
  const inverted = GRAY_INVERT_ENABLED ? (~raw_mask & 0xff) : (raw_mask & 0xff);
  sensor.history[sensor.next] = inverted;
  sensor.next = (sensor.next + 1) % SENSOR_FILTER_SAMPLES;
  if (sensor.count < SENSOR_FILTER_SAMPLES) sensor.count++;

  let filtered = 0;
  for (let bit = 0; bit < 8; ++bit) {
    let votes = 0;
    for (let j = 0; j < sensor.count; ++j) {
      if ((sensor.history[j] & (1 << bit)) !== 0) votes++;
    }
    if (votes > Math.floor(sensor.count / 2)) {
      filtered |= 1 << bit;
    }
  }

  const prefilter = filtered;
  if (FLOOR_SEAM_FILTER_ENABLED) {
    const neighbors = ((filtered << 1) | (filtered >> 1)) & 0xff;
    filtered &= neighbors;
  }

  const result = grayClassify(filtered, have_previous, previous);
  result.raw_mask = raw_mask;
  result.prefilter_mask = prefilter;
  result.seam_mask = (prefilter & (~filtered & 0xff)) & 0xff;
  return result;
}
