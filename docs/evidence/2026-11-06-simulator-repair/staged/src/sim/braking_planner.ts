export interface BrakingPlan {
  allowed_speed: number;
  remaining_cm: number;
  required_decel: number;
  valid: boolean;
}

export interface BrakingDecision {
  required_decel: number;
  reverse_needed: boolean;
  distance_exhausted: boolean;
}

export class BrakingSlewModel {
  public speed = 0;
  public acceleration = 0;

  public reset(): void {
    this.speed = 0;
    this.acceleration = 0;
  }
}

export function brakingPlan(
  cruise: number,
  entry: number,
  length: number,
  progress: number,
  speed: number,
  decel: number,
  margin: number,
  calibrated: boolean
): BrakingPlan {
  const p: BrakingPlan = {
    allowed_speed: 0,
    remaining_cm: 0,
    required_decel: 0,
    valid: false,
  };

  if (
    !Number.isFinite(cruise) ||
    !Number.isFinite(entry) ||
    !Number.isFinite(length) ||
    !Number.isFinite(progress) ||
    !Number.isFinite(speed) ||
    !Number.isFinite(decel) ||
    !Number.isFinite(margin) ||
    cruise <= 0 ||
    entry < 0 ||
    entry > cruise ||
    length < 0 ||
    margin < 0
  ) {
    return p;
  }

  p.allowed_speed = cruise;
  if (length === 0) {
    p.valid = true;
    return p;
  }

  if (!calibrated || decel <= 0) return p;

  p.remaining_cm = Math.max(0, length - Math.max(0, progress) - margin);
  p.allowed_speed = Math.min(cruise, Math.sqrt(entry * entry + 2 * decel * p.remaining_cm));

  const excess = Math.max(0, speed * speed - entry * entry);
  p.required_decel = p.remaining_cm > 0 ? excess / (2 * p.remaining_cm) : excess > 0 ? Infinity : 0;
  p.valid = Number.isFinite(p.allowed_speed);
  return p;
}

export function brakingDecide(
  speed: number,
  entry: number,
  remaining: number,
  observed_decel: number,
  safe_decel: number,
  guard: number
): BrakingDecision {
  const d: BrakingDecision = {
    required_decel: 0,
    reverse_needed: false,
    distance_exhausted: false,
  };

  if (
    !Number.isFinite(speed) ||
    !Number.isFinite(entry) ||
    !Number.isFinite(remaining) ||
    !Number.isFinite(observed_decel) ||
    !Number.isFinite(safe_decel) ||
    !Number.isFinite(guard) ||
    speed < 0 ||
    entry < 0 ||
    remaining < 0 ||
    safe_decel <= 0 ||
    guard < 0
  ) {
    d.distance_exhausted = true;
    return d;
  }

  if (speed <= entry) return d;

  const usable = Math.max(0, remaining - guard);
  d.required_decel = usable > 0 ? (speed * speed - entry * entry) / (2 * usable) : Infinity;
  d.distance_exhausted = usable <= 0;

  const available = observed_decel > 0 ? Math.min(safe_decel, observed_decel) : safe_decel;
  d.reverse_needed = d.required_decel > available;
  return d;
}

export function brakingSlew(
  s: BrakingSlewModel,
  cap: number,
  dt: number,
  acceleration: number,
  jerk: number
): number {
  if (
    !s ||
    !Number.isFinite(cap) ||
    cap < 0 ||
    !Number.isFinite(dt) ||
    dt <= 0 ||
    !Number.isFinite(acceleration) ||
    acceleration <= 0 ||
    !Number.isFinite(jerk) ||
    jerk <= 0
  ) {
    return 0;
  }

  if (cap <= s.speed) {
    s.speed = cap;
    s.acceleration = 0;
    return cap;
  }

  s.acceleration = Math.min(acceleration, s.acceleration + jerk * dt);
  s.speed = Math.min(cap, s.speed + s.acceleration * dt);
  return s.speed;
}
