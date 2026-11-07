import './style.css';
import { LineFollowModel } from './sim/line_follow';
import { VehicleSimulator } from './sim/physics';
import { TrackRenderer } from './track/track_renderer';
import { CarRenderer } from './car/car_renderer';
import {
  parseRouteTableC,
  DEFAULT_ROUTE_TABLE_C,
  type ParseResult,
} from './sim/route_parser';
import { Action, RouteState, HexagonState } from './sim/types';
import { CONTROL_PERIOD_S } from './sim/config';

// Global Simulator State
class App {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;

  private trackRenderer: TrackRenderer;
  private carRenderer: CarRenderer;
  private lineFollow: LineFollowModel;
  private vehicleSim: VehicleSimulator;

  private parseResult: ParseResult;
  private rawSourceCode: string = DEFAULT_ROUTE_TABLE_C;

  private isRunning = false;
  private simSpeed = 1.0;
  private followCar = false;
  private timeAccumulator = 0;
  private lastRafTime = 0;

  // Pan and Zoom Interaction
  private isDragging = false;
  private dragStartX = 0;
  private dragStartY = 0;

  constructor() {
    const canvas = document.getElementById('simCanvas') as HTMLCanvasElement;
    if (!canvas) throw new Error('Canvas not found');
    this.canvas = canvas;
    const context = canvas.getContext('2d');
    if (!context) throw new Error('Cannot get 2D context');
    this.ctx = context;

    this.parseResult = parseRouteTableC(this.rawSourceCode);

    this.trackRenderer = new TrackRenderer(this.canvas);
    this.carRenderer = new CarRenderer(this.trackRenderer);
    this.lineFollow = new LineFollowModel(this.parseResult.activeSteps);
    this.vehicleSim = new VehicleSimulator(this.lineFollow);

    this.initCanvasSize();
    this.initUI();
    this.initEventListeners();

    // Start simulation loop
    this.lastRafTime = performance.now();
    requestAnimationFrame(this.loop.bind(this));
  }

  private initCanvasSize(): void {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = window.innerWidth * dpr;
    this.canvas.height = window.innerHeight * dpr;
    this.trackRenderer.fitViewport(50);
  }

  private initUI(): void {
    this.buildSensorArrayUI();
    this.updateSchemeSelectUI();
    this.renderRouteTableModal();
    this.updateHUD();
  }

  private buildSensorArrayUI(): void {
    const container = document.getElementById('sensorArray');
    if (!container) return;
    container.innerHTML = '';

    for (let i = 0; i < 8; ++i) {
      const node = document.createElement('div');
      node.className = 'sensor-node';
      node.id = `sensorNode_${i}`;
      node.innerHTML = `
        <div class="sensor-dot" id="sensorDot_${i}"></div>
        <span>S${i}</span>
      `;
      container.appendChild(node);
    }
  }

  private updateSchemeSelectUI(): void {
    const select = document.getElementById('schemeSelect') as HTMLSelectElement;
    if (!select) return;

    select.innerHTML = '';
    this.parseResult.schemes.forEach((sc) => {
      const opt = document.createElement('option');
      opt.value = sc.id.toString();
      opt.textContent = `${sc.name} (${sc.steps.length}步)`;
      if (sc.id === this.parseResult.activeSchemeId) {
        opt.selected = true;
      }
      select.appendChild(opt);
    });
  }

  private actionToString(action: Action): string {
    switch (action) {
      case Action.LEFT: return 'LEFT (左转90°)';
      case Action.RIGHT: return 'RIGHT (右转90°)';
      case Action.STRAIGHT: return 'STRAIGHT (直行)';
      case Action.STOP: return 'STOP (停止)';
      case Action.CUSTOM_TURN: return 'CUSTOM_TURN (自定义转角)';
      case Action.HEXAGON: return 'HEXAGON (环岛绕行)';
      default: return 'UNKNOWN';
    }
  }

  private routeStateToString(state: RouteState): string {
    switch (state) {
      case RouteState.ROUTE_FOLLOW: return 'ROUTE_FOLLOW (巡线)';
      case RouteState.ROUTE_CANDIDATE: return 'ROUTE_CANDIDATE (候选识别)';
      case RouteState.ROUTE_EXECUTING: return 'ROUTE_EXECUTING (动作执行)';
      case RouteState.ROUTE_CLEARING: return 'ROUTE_CLEARING (离口去重)';
      case RouteState.ROUTE_FINISHED: return 'ROUTE_FINISHED (路径完成)';
      case RouteState.ROUTE_FAILED: return 'ROUTE_FAILED (导航故障)';
      case RouteState.ROUTE_BRAKING: return 'ROUTE_BRAKING (入口减速)';
      case RouteState.ROUTE_RECOVERING: return 'ROUTE_RECOVERING (找线恢复)';
      default: return 'DISABLED';
    }
  }

  private hexStateToString(state: HexagonState): string {
    switch (state) {
      case HexagonState.HEXAGON_OFF: return 'HEXAGON_OFF (未激活)';
      case HexagonState.HEXAGON_ENTERING: return 'HEXAGON_ENTERING (入环)';
      case HexagonState.HEXAGON_FOLLOWING: return 'HEXAGON_FOLLOWING (环内巡线)';
      case HexagonState.HEXAGON_EXIT_CANDIDATE: return 'EXIT_CANDIDATE (出口候选)';
      case HexagonState.HEXAGON_EXITING: return 'HEXAGON_EXITING (出环)';
      case HexagonState.HEXAGON_DONE: return 'HEXAGON_DONE (完成)';
      default: return 'OFF';
    }
  }

  private renderRouteTableModal(): void {
    const tbody = document.getElementById('routeTableBody');
    const schemeNameElem = document.getElementById('modalSchemeName');
    const stepCountElem = document.getElementById('modalStepCount');
    const codeViewer = document.getElementById('sourceCodeViewer');

    const activeScheme = this.parseResult.schemes.find(
      (s) => s.id === this.parseResult.activeSchemeId
    );

    if (schemeNameElem && activeScheme) {
      schemeNameElem.textContent = activeScheme.name;
    }
    if (stepCountElem && activeScheme) {
      stepCountElem.textContent = `共 ${activeScheme.steps.length} 步`;
    }
    if (codeViewer) {
      codeViewer.textContent = this.rawSourceCode;
    }

    if (!tbody || !activeScheme) return;
    tbody.innerHTML = '';

    activeScheme.steps.forEach((step, idx) => {
      const tr = document.createElement('tr');
      tr.id = `routeRow_${idx}`;
      if (idx === this.lineFollow.route.index) {
        tr.className = 'active-step';
      }

      tr.innerHTML = `
        <td><strong>#${idx}</strong></td>
        <td>${step.junction_count}</td>
        <td>${step.turn_speed} cm/s</td>
        <td><code>${Action[step.action] || step.action}</code></td>
        <td>${step.cruise_speed} cm/s</td>
        <td>${step.angle_deg > 0 ? '+' : ''}${step.angle_deg}°</td>
        <td>${step.corner_arc_enabled ? '✓' : '—'}</td>
        <td>${step.segment_length_cm} cm</td>
        <td><small>${step.comment || '—'}</small></td>
      `;
      tbody.appendChild(tr);
    });
  }

  private loadScheme(schemeId: number): void {
    const sc = this.parseResult.schemes.find((s) => s.id === schemeId);
    if (!sc) return;

    this.parseResult.activeSchemeId = schemeId;
    this.parseResult.activeSteps = sc.steps;

    this.lineFollow.init(sc.steps, (completedStep) => {
      console.log(`Route finished step: ${completedStep}`);
    });
    this.vehicleSim.reset();
    this.timeAccumulator = 0;
    this.isRunning = false;
    this.updateStatusPill();
    this.renderRouteTableModal();
    this.updateHUD();
  }

  private handleFileUpload(file: File): void {
    const reader = new FileReader();
    reader.onload = (e) => {
      const content = e.target?.result as string;
      if (!content) return;

      this.rawSourceCode = content;
      this.parseResult = parseRouteTableC(content);

      this.updateSchemeSelectUI();
      this.loadScheme(this.parseResult.activeSchemeId);

      // Alert / toast
      const statusPill = document.getElementById('simStatusPill');
      if (statusPill) {
        statusPill.innerHTML = `
          <span class="status-dot"></span>
          <span class="status-text">已加载: ${file.name}</span>
        `;
      }
    };
    reader.readAsText(file);
  }

  private updateStatusPill(): void {
    const pill = document.getElementById('simStatusPill');
    const playIcon = document.getElementById('playIcon');
    if (!pill || !playIcon) return;

    if (this.isRunning) {
      pill.className = 'status-pill running';
      pill.innerHTML = `
        <span class="status-dot"></span>
        <span class="status-text">仿真运行中 (RUNNING)</span>
      `;
      playIcon.textContent = '⏸';
    } else if (this.lineFollow.route.state === RouteState.ROUTE_FINISHED) {
      pill.className = 'status-pill';
      pill.innerHTML = `
        <span class="status-dot"></span>
        <span class="status-text">已完成 (FINISHED)</span>
      `;
      playIcon.textContent = '▶';
    } else if (this.lineFollow.route.state === RouteState.ROUTE_FAILED) {
      pill.className = 'status-pill stopped';
      pill.innerHTML = `
        <span class="status-dot"></span>
        <span class="status-text">故障停止 (STOPPED)</span>
      `;
      playIcon.textContent = '▶';
    } else {
      pill.className = 'status-pill';
      pill.innerHTML = `
        <span class="status-dot"></span>
        <span class="status-text">就绪 / 暂停 (PAUSED)</span>
      `;
      playIcon.textContent = '▶';
    }
  }

  private initEventListeners(): void {
    // Window Resize -> Auto fit
    window.addEventListener('resize', () => {
      this.initCanvasSize();
    });

    // Fit Viewport Button ("画面铺满")
    const btnFit = document.getElementById('btnFitViewport');
    if (btnFit) {
      btnFit.addEventListener('click', () => {
        this.trackRenderer.fitViewport(50);
      });
    }

    // Play / Pause Button
    const btnPlay = document.getElementById('btnPlayPause');
    if (btnPlay) {
      btnPlay.addEventListener('click', () => {
        this.isRunning = !this.isRunning;
        if (this.isRunning) {
          this.vehicleSim.requestStart();
        }
        this.updateStatusPill();
      });
    }

    // Step Button
    const btnStep = document.getElementById('btnStep');
    if (btnStep) {
      btnStep.addEventListener('click', () => {
        this.isRunning = false;
        this.vehicleSim.requestStart();
        this.vehicleSim.step(CONTROL_PERIOD_S);
        this.updateStatusPill();
        this.updateHUD();
      });
    }

    // Reset Button
    const btnReset = document.getElementById('btnReset');
    if (btnReset) {
      btnReset.addEventListener('click', () => {
        this.isRunning = false;
        this.loadScheme(this.parseResult.activeSchemeId);
      });
    }

    // Speed multiplier buttons
    const speedButtons = document.querySelectorAll('.speed-btn');
    speedButtons.forEach((btn) => {
      btn.addEventListener('click', () => {
        speedButtons.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.simSpeed = parseFloat(btn.getAttribute('data-speed') || '1.0');
      });
    });

    // Scheme select change
    const schemeSelect = document.getElementById('schemeSelect') as HTMLSelectElement;
    if (schemeSelect) {
      schemeSelect.addEventListener('change', () => {
        this.loadScheme(parseInt(schemeSelect.value, 10));
      });
    }

    // File Upload Button & File Input
    const btnUpload = document.getElementById('btnUploadRoute');
    const fileInput = document.getElementById('fileInput') as HTMLInputElement;
    if (btnUpload && fileInput) {
      btnUpload.addEventListener('click', () => fileInput.click());
      fileInput.addEventListener('change', () => {
        if (fileInput.files && fileInput.files[0]) {
          this.handleFileUpload(fileInput.files[0]);
        }
      });
    }

    // Drag and Drop Route Table C file
    const overlay = document.getElementById('dragOverlay');
    window.addEventListener('dragover', (e) => {
      e.preventDefault();
      if (overlay) overlay.style.display = 'flex';
    });
    window.addEventListener('dragleave', (e) => {
      if (e.relatedTarget === null && overlay) {
        overlay.style.display = 'none';
      }
    });
    window.addEventListener('drop', (e) => {
      e.preventDefault();
      if (overlay) overlay.style.display = 'none';
      if (e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0]) {
        this.handleFileUpload(e.dataTransfer.files[0]);
      }
    });

    // Route Table Modal
    const btnViewRoute = document.getElementById('btnViewRouteTable');
    const modalBackdrop = document.getElementById('modalBackdrop');
    const btnCloseModal = document.getElementById('btnCloseModal');
    const btnReplaceFile = document.getElementById('btnReplaceFile');

    if (btnViewRoute && modalBackdrop) {
      btnViewRoute.addEventListener('click', () => {
        this.renderRouteTableModal();
        modalBackdrop.style.display = 'flex';
      });
    }
    if (btnCloseModal && modalBackdrop) {
      btnCloseModal.addEventListener('click', () => {
        modalBackdrop.style.display = 'none';
      });
    }
    if (btnReplaceFile && fileInput) {
      btnReplaceFile.addEventListener('click', () => fileInput.click());
    }

    // Checkbox toggles
    const chkFollow = document.getElementById('chkFollowCar') as HTMLInputElement;
    if (chkFollow) {
      chkFollow.addEventListener('change', () => {
        this.followCar = chkFollow.checked;
      });
    }

    const chkDims = document.getElementById('chkDimensions') as HTMLInputElement;
    if (chkDims) {
      chkDims.addEventListener('change', () => {
        this.trackRenderer.options.showDimensions = chkDims.checked;
      });
    }

    const chkComps = document.getElementById('chkComponents') as HTMLInputElement;
    if (chkComps) {
      chkComps.addEventListener('change', () => {
        this.trackRenderer.options.showComponentLabels = chkComps.checked;
      });
    }

    // HUD Collapse toggle
    const btnToggleHud = document.getElementById('btnToggleHud');
    const leftHud = document.getElementById('leftHud');
    if (btnToggleHud && leftHud) {
      btnToggleHud.addEventListener('click', () => {
        leftHud.classList.toggle('collapsed');
        btnToggleHud.textContent = leftHud.classList.contains('collapsed') ? '+' : '−';
      });
    }

    // Pan and Zoom with Mouse
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
      const mouseWorldBefore = this.trackRenderer.toWorld(e.clientX, e.clientY);

      this.trackRenderer.transform.scale *= zoomFactor;
      this.trackRenderer.transform.scale = Math.max(0.5, Math.min(25.0, this.trackRenderer.transform.scale));

      const mouseWorldAfter = this.trackRenderer.toWorld(e.clientX, e.clientY);
      this.trackRenderer.transform.offsetX += (mouseWorldAfter.x - mouseWorldBefore.x) * this.trackRenderer.transform.scale;
      this.trackRenderer.transform.offsetY += (mouseWorldAfter.y - mouseWorldBefore.y) * this.trackRenderer.transform.scale;
    });

    this.canvas.addEventListener('mousedown', (e) => {
      this.isDragging = true;
      this.dragStartX = e.clientX;
      this.dragStartY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.dragStartX;
      const dy = e.clientY - this.dragStartY;
      this.dragStartX = e.clientX;
      this.dragStartY = e.clientY;
      this.trackRenderer.transform.offsetX += dx;
      this.trackRenderer.transform.offsetY += dy;
    });

    window.addEventListener('mouseup', () => {
      this.isDragging = false;
    });

    // Spacebar to Play/Pause
    window.addEventListener('keydown', (e) => {
      if (e.code === 'Space' && e.target === document.body) {
        e.preventDefault();
        btnPlay?.click();
      }
    });
  }

  private updateHUD(): void {
    const rawMask = this.vehicleSim.raw_sensor_mask;
    const obs = this.vehicleSim.follow.observation;

    // 8-Sensor array dots
    for (let i = 0; i < 8; ++i) {
      const dot = document.getElementById(`sensorDot_${i}`);
      if (dot) {
        if ((rawMask & (1 << i)) !== 0) {
          dot.className = 'sensor-dot active';
        } else {
          dot.className = 'sensor-dot';
        }
      }
    }

    // Metric texts
    const valSensorPos = document.getElementById('valSensorPos');
    if (valSensorPos) valSensorPos.textContent = obs.position.toFixed(2);

    const valSensorKind = document.getElementById('valSensorKind');
    if (valSensorKind) {
      valSensorKind.textContent =
        obs.kind === 0 ? 'VALID (有效)' : obs.kind === 1 ? 'NONE (丢线)' : 'AMBIGUOUS (歧义)';
    }

    const valLineMask = document.getElementById('valLineMask');
    if (valLineMask) {
      valLineMask.textContent = `0x${rawMask.toString(16).padStart(2, '0')} (${rawMask.toString(2).padStart(8, '0')})`;
    }

    const valSpeedLinear = document.getElementById('valSpeedLinear');
    if (valSpeedLinear) {
      const vLinear = (this.vehicleSim.actual_left_speed + this.vehicleSim.actual_right_speed) * 0.5;
      valSpeedLinear.innerHTML = `${vLinear.toFixed(1)} <small>cm/s</small>`;
    }

    const valSpeedTarget = document.getElementById('valSpeedTarget');
    if (valSpeedTarget) {
      valSpeedTarget.innerHTML = `${this.lineFollow.base.toFixed(1)} <small>cm/s</small>`;
    }

    const valSpeedLeft = document.getElementById('valSpeedLeft');
    if (valSpeedLeft) {
      valSpeedLeft.innerHTML = `${this.vehicleSim.actual_left_speed.toFixed(1)} <small>cm/s</small>`;
    }

    const valSpeedRight = document.getElementById('valSpeedRight');
    if (valSpeedRight) {
      valSpeedRight.innerHTML = `${this.vehicleSim.actual_right_speed.toFixed(1)} <small>cm/s</small>`;
    }

    const valPwmLeft = document.getElementById('valPwmLeft');
    if (valPwmLeft) {
      valPwmLeft.innerHTML = `${Math.round(this.lineFollow.command.left_pwm)} <small>/ 255</small>`;
    }

    const valPwmRight = document.getElementById('valPwmRight');
    if (valPwmRight) {
      valPwmRight.innerHTML = `${Math.round(this.lineFollow.command.right_pwm)} <small>/ 255</small>`;
    }

    const valCarCoords = document.getElementById('valCarCoords');
    if (valCarCoords) {
      valCarCoords.textContent = `(${this.vehicleSim.pose.x.toFixed(1)}, ${this.vehicleSim.pose.y.toFixed(1)}) cm`;
    }

    const valCarHeading = document.getElementById('valCarHeading');
    if (valCarHeading) {
      const deg = (this.vehicleSim.pose.heading * 180) / Math.PI;
      valCarHeading.textContent = `${deg.toFixed(1)}° (${this.lineFollow.odometry.heading_rad.toFixed(2)} rad)`;
    }

    const valCarDistance = document.getElementById('valCarDistance');
    if (valCarDistance) {
      valCarDistance.textContent = `${this.lineFollow.odometry.distance_cm.toFixed(1)} cm`;
    }

    const valSimTime = document.getElementById('valSimTime');
    if (valSimTime) {
      valSimTime.textContent = `${this.vehicleSim.sim_time_s.toFixed(3)} s`;
    }

    // Route Status
    const r = this.lineFollow.route;
    const curStep = r.steps[r.index];

    const valCurStepIdx = document.getElementById('valCurrentStepIndex');
    if (valCurStepIdx) {
      valCurStepIdx.textContent = `第 ${r.index} / ${r.count} 步`;
    }

    const valCurAction = document.getElementById('valCurrentAction');
    if (valCurAction && curStep) {
      valCurAction.textContent = this.actionToString(curStep.action);
    }

    const valRouteState = document.getElementById('valRouteState');
    if (valRouteState) {
      valRouteState.textContent = this.routeStateToString(r.state);
    }

    const valHexState = document.getElementById('valHexState');
    if (valHexState) {
      valHexState.textContent = this.hexStateToString(r.hexagon.state);
    }

    const valJunctionsSeen = document.getElementById('valJunctionsSeen');
    if (valJunctionsSeen && curStep) {
      valJunctionsSeen.textContent = `${r.junctions_seen} / ${curStep.junction_count || '∞'}`;
    }

    const valStepComment = document.getElementById('valStepComment');
    if (valStepComment && curStep) {
      valStepComment.textContent = curStep.comment || '无动作备注';
    }
  }

  private loop(now: number): void {
    const elapsedS = Math.min(0.1, (now - this.lastRafTime) / 1000.0);
    this.lastRafTime = now;

    if (this.isRunning) {
      this.timeAccumulator += elapsedS * this.simSpeed;
      const dt = CONTROL_PERIOD_S;

      while (this.timeAccumulator >= dt) {
        this.vehicleSim.step(dt, false);
        this.timeAccumulator -= dt;

        // Auto pause on route finish or fail
        if (
          this.lineFollow.route.state === RouteState.ROUTE_FINISHED ||
          this.lineFollow.route.state === RouteState.ROUTE_FAILED
        ) {
          this.isRunning = false;
          this.updateStatusPill();
          break;
        }
      }
    }

    // Follow Car Camera Mode
    if (this.followCar) {
      const screenCar = this.trackRenderer.toScreen(this.vehicleSim.pose.x, this.vehicleSim.pose.y);
      const targetScreenCenterX = this.canvas.width / 2;
      const targetScreenCenterY = this.canvas.height / 2;
      this.trackRenderer.transform.offsetX += (targetScreenCenterX - screenCar.x) * 0.1;
      this.trackRenderer.transform.offsetY += (targetScreenCenterY - screenCar.y) * 0.1;
    }

    // Render Canvas
    this.trackRenderer.render();
    this.carRenderer.render(this.vehicleSim, this.ctx);

    // Update HUD
    this.updateHUD();

    requestAnimationFrame(this.loop.bind(this));
  }
}

// Start App when DOM is ready
window.addEventListener('DOMContentLoaded', () => {
  new App();
});
