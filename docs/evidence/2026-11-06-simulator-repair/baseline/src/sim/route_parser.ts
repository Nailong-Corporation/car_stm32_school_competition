import { Action, type RouteStep } from './types';

export interface ParsedScheme {
  id: number;
  name: string;
  steps: RouteStep[];
}

export interface ParseResult {
  activeSchemeId: number;
  schemes: ParsedScheme[];
  activeSteps: RouteStep[];
  error?: string;
}

function parseAction(actStr: string): Action {
  const clean = actStr.trim();
  switch (clean) {
    case 'LEFT':
      return Action.LEFT;
    case 'RIGHT':
      return Action.RIGHT;
    case 'STRAIGHT':
      return Action.STRAIGHT;
    case 'STOP':
      return Action.STOP;
    case 'CUSTOM_TURN':
      return Action.CUSTOM_TURN;
    case 'HEXAGON':
      return Action.HEXAGON;
    default:
      if (!isNaN(Number(clean))) return Number(clean) as Action;
      return Action.STRAIGHT;
  }
}

function parseStepLine(line: string): RouteStep | null {
  const match = line.match(/\{\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^}]+)\}/);
  if (!match) return null;

  const junction_count = parseInt(match[1].trim(), 10);
  const turn_speed = parseFloat(match[2].replace(/[fF]/g, '').trim());
  const action = parseAction(match[3].trim());
  const cruise_speed = parseInt(match[4].trim(), 10);
  const angle_deg = parseFloat(match[5].replace(/[fF]/g, '').trim());
  const corner_arc_enabled = match[6].trim().toLowerCase() === 'true';
  const segment_length_cm = parseFloat(match[7].replace(/[fF]/g, '').trim());

  let comment = '';
  const commentMatch = line.match(/\/\*\s*(.*?)\s*\*\//) || line.match(/\/\/\s*(.*)/);
  if (commentMatch) {
    comment = commentMatch[1].trim();
  }

  return {
    junction_count,
    turn_speed,
    action,
    cruise_speed,
    angle_deg,
    corner_arc_enabled,
    segment_length_cm,
    comment,
    rawLine: line.trim(),
  };
}

export function parseRouteTableC(sourceCode: string): ParseResult {
  const lines = sourceCode.split(/\r?\n/);
  let activeSchemeId = 1; // default to scheme 1 for full track simulation

  const schemeMacroMatch = sourceCode.match(/#define\s+ROUTE_SCHEME_SELECT\s+(\d+)/);
  if (schemeMacroMatch) {
    activeSchemeId = parseInt(schemeMacroMatch[1], 10);
  }

  const schemes: ParsedScheme[] = [
    { id: 0, name: '方案 0 (基础调试用赛道 - 4步)', steps: [] },
    { id: 1, name: '方案 1 (正式全元件: 走虚线+环岛 - 30步)', steps: [] },
    { id: 2, name: '方案 2 (不走虚线，走环岛 - 30步)', steps: [] },
    { id: 3, name: '方案 3 (走虚线，不走环岛 - 25步)', steps: [] },
  ];

  let currentSchemeId: number | null = null;
  let inRouteArray = false;
  const customSteps: RouteStep[] = [];

  for (let i = 0; i < lines.length; ++i) {
    const line = lines[i];

    const ifMatch = line.match(/#if\s*\(\s*ROUTE_SCHEME_SELECT\s*==\s*(\d+)\s*\)/) ||
                    line.match(/#elif\s*\(\s*ROUTE_SCHEME_SELECT\s*==\s*(\d+)\s*\)/);
    if (ifMatch) {
      currentSchemeId = parseInt(ifMatch[1], 10);
      continue;
    }

    if (line.includes('#endif')) {
      currentSchemeId = null;
      continue;
    }

    if (line.includes('RouteStep route[]')) {
      inRouteArray = true;
      continue;
    }

    if (inRouteArray && line.includes('};')) {
      inRouteArray = false;
      continue;
    }

    if (line.includes('{') && line.includes('}')) {
      const step = parseStepLine(line);
      if (step) {
        if (currentSchemeId !== null) {
          const s = schemes.find((sc) => sc.id === currentSchemeId);
          if (s) {
            s.steps.push(step);
          } else {
            schemes.push({
              id: currentSchemeId,
              name: `方案 ${currentSchemeId}`,
              steps: [step],
            });
          }
        } else {
          customSteps.push(step);
        }
      }
    }
  }

  // Filter schemes that have steps
  const validSchemes = schemes.filter((s) => s.steps.length > 0);
  if (validSchemes.length === 0 && customSteps.length > 0) {
    validSchemes.push({
      id: 0,
      name: '自定义上传路径',
      steps: customSteps,
    });
    activeSchemeId = 0;
  }

  const selected = validSchemes.find((s) => s.id === activeSchemeId) || validSchemes[0];

  return {
    activeSchemeId: selected ? selected.id : 0,
    schemes: validSchemes,
    activeSteps: selected ? selected.steps : customSteps,
  };
}

export const DEFAULT_ROUTE_TABLE_C = `#include "route.h"

/* 路径方案选择宏：
 * 0: 基础调试用赛道 (4步)
 * 1: 正式赛道全元件 (方案一：走虚线 + 走环岛，覆盖全部27个元件，共30步)
 * 2: 正式赛道 不走虚线 (方案二：跳过虚线与隔离电阻，走环岛，覆盖26个元件，共30步)
 * 3: 正式赛道全元件 不走环岛 (方案三：走虚线，跳过环岛，覆盖26个元件，共25步)
 */
#ifndef ROUTE_SCHEME_SELECT
#define ROUTE_SCHEME_SELECT 1
#endif

const RouteStep route[] = {
#if (ROUTE_SCHEME_SELECT == 0)
    /* 基础调试用赛道 */
    { 1, 50.0f, CUSTOM_TURN,  50, -90.0f, false,  60.0f},
    { 4, 50.0f, CUSTOM_TURN, 200, -90.0f, false, 180.0f},
    { 2, 50.0f, CUSTOM_TURN, 200, -90.0f, false,  60.0f},
    { 3, 50.0f, STOP       , 200,   0.0f, false, 200.0f},

#elif (ROUTE_SCHEME_SELECT == 1)
    /* 正式赛道全元件 (走虚线 + 走环岛，覆盖全部27个元件，共30步) */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 120.0f}, /* 0: P(0,120)向北直行至左上角(0,0)右转90°，途经元件1 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 1: 沿顶边向东至x=55右转90°，途经元件2 */
    { 1, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 120.0f}, /* 2: 沿x=55南下至y=120左转90°，途经元件3 */
    { 2, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 190.0f}, /* 3: 沿y=120向东过节点4(直行)至节点6(245,120)右转90°，途经元件4、5 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 4: 沿x=245南下至y=170右转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 150.0f}, /* 5: 沿y=170向西过x=150(直行)至x=95左转90°，途经二极管7 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 6: 沿x=95南下至底边y=220左转90° */
    { 4, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 280.0f}, /* 7: 沿底边向东过150、245、三极管11(直行)至右下角(375,220)左转90°，途经元件8、9、10、11 */
    { 1, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 100.0f}, /* 8: 沿x=375北上至y=120左转90°，途经元件12 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  30.0f}, /* 9: 沿y=120向西过三极管13至x=345左转90°，途经三极管13(第1次) */
    { 1, 12.0f, HEXAGON    ,  60, -300.0f, false,  50.0f}, /* 10: 沿x=345南下入六边形环岛逆时针绕行出环，途经元件14 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 11: 出环沿x=345北上至y=120三极管13右转90°(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  30.0f}, /* 12: 沿y=120短横线向东至x=375左转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 120.0f}, /* 13: 沿x=375北上过y=50(直行)至右上角(375,0)左转90°，途经元件15 */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 100.0f}, /* 14: 沿顶边向西过三极管16(直行)至x=275左转90°，途经三极管16(第1次)、元件17(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 15: 沿x=275南下至y=50三极管18左转90°，途经三极管18 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  70.0f}, /* 16: 沿y=50向东至x=345左转90°，途经元件19 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 17: 沿x=345北上至顶边三极管16左转90°(第2次) */
    { 3, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 250.0f}, /* 18: 沿顶边向西过275、245(直行)至x=95三极管22左转90°，途经元件17(第2次)、20、21、22 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 19: 沿x=95南下至y=40左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  70.0f}, /* 20: 沿y=40向东至方框西北角x=165右转90°，途经元件23(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 21: 沿方框西边南下至y=80左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  80.0f}, /* 22: 沿方框底边向东至x=245左转90°，途经元件24 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 23: 沿方框东边北上至y=40左转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 150.0f}, /* 24: 沿y=40向西过x=165(直行)至x=95左转90°，途经元件25、23(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  80.0f}, /* 25: 沿x=95南下至y=120三极管4右转90°(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 26: 沿y=120向西至x=55左转90°进入虚线 */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 100.0f}, /* 27: 穿过虚线南下至底边y=220右转90°，途经元件26 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 28: 沿底边向西至最左下角(0,220)右转90° */
    { 1, 50.0f, STOP       , 200,    0.0f, false, 100.0f}, /* 29: 沿最左边框x=0北上至P(0,120)终点停车，途经元件27 */

#elif (ROUTE_SCHEME_SELECT == 2)
    /* 正式赛道 不走虚线 (跳过虚线与隔离电阻，走环岛，覆盖26个元件，共30步) */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 120.0f}, /* 0: P(0,120)北上至左上角(0,0)右转90°，途经元件1 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 1: 沿顶边向东至x=55右转90°，途经元件2 */
    { 1, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 120.0f}, /* 2: 沿x=55南下至y=120左转90°，途经元件3 */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 190.0f}, /* 3: 沿y=120向东过节点4(直行)至节点6(245,120)左转90°北上，途经元件5 */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false,  80.0f}, /* 4: 沿x=245北上过y=80(直行)至方框东北角(245,40)左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  80.0f}, /* 5: 沿方框顶边向西至西北角(165,40)左转90°，途经元件7(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 6: 沿方框西边南下至西南角(165,80)左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  80.0f}, /* 7: 沿方框底边向东至东南角(245,80)左转90°，途经元件8 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 8: 沿方框东边北上至东北角(245,40)左转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 150.0f}, /* 9: 沿方框顶边向西过165(直行)至节点10(95,40)右转90°，途经元件7(第2次)、9 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  40.0f}, /* 10: 沿x=95北上至顶边y=0右转90° */
    { 3, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 250.0f}, /* 11: 沿顶边向东过245、275(直行)至节点(345,0)右转90°，途经元件11、12、13(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 12: 沿x=345南下至y=50右转90°，途经元件14(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  70.0f}, /* 13: 沿y=50向西至节点16(275,50)右转90°，途经元件15 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 14: 沿x=275北上至顶边(275,0)右转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  70.0f}, /* 15: 沿顶边向东至节点(345,0)右转90°，途经元件13(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 16: 沿x=345南下至y=50左转90°向东，途经元件14(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  30.0f}, /* 17: 沿y=50向东至最右垂直线x=375右转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 170.0f}, /* 18: 沿x=375南下过y=120(直行)至右下角(375,220)右转90°，途经元件17、18(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  30.0f}, /* 19: 沿底边向西至节点19(345,220)右转90°北上 */
    { 1, 12.0f, HEXAGON    ,  60,  300.0f, false,  50.0f}, /* 20: 沿x=345北上入环岛顺时针绕行出环，途经元件20 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 21: 出环沿x=345北上至节点21(345,120)右转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  30.0f}, /* 22: 沿短横线向东至x=375右转90°南下 */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 100.0f}, /* 23: 沿x=375再次南下至右下角(375,220)右转90°，途经元件18(第2次) */
    { 4, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 280.0f}, /* 24: 沿底边向西过19、245、150(直行)至(95,220)右转90°，途经元件22、23、24(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 25: 沿x=95北上至y=170右转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 26: 沿y=170向东至节点25(150,170)右转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 27: 沿x=150南下至底边(150,220)右转90° */
    { 3, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 150.0f}, /* 28: 沿底边向西过95、55(直行)至左下角(0,220)右转90°，途经元件24(第2次) */
    { 1, 50.0f, STOP       , 200,    0.0f, false, 100.0f}, /* 29: 沿x=0北上至P(0,120)终点停车，途经元件26 */

#elif (ROUTE_SCHEME_SELECT == 3)
    /* 正式赛道全元件 不走环岛 (走虚线，跳过环岛，覆盖26个元件，共25步) */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 120.0f}, /* 0: P(0,120)向北直行至左上角(0,0)右转90°，途经元件1 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 1: 沿顶边向东至x=55右转90°，途经元件2 */
    { 1, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 120.0f}, /* 2: 沿x=55南下至y=120左转90°，途经元件3 */
    { 2, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 190.0f}, /* 3: 沿y=120向东过节点4(直行)至节点6(245,120)右转90°，途经元件4、5 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  50.0f}, /* 4: 沿x=245南下至y=170右转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 150.0f}, /* 5: 沿y=170向西过x=150(直行)至x=95左转90°，途经二极管7 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 6: 沿x=95南下至底边y=220左转90° */
    { 4, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 280.0f}, /* 7: 沿底边向东过150、245、三极管11(直行)至右下角(375,220)左转90°，途经元件8、9、10、11 */
    { 3, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 220.0f}, /* 8: 不走环岛：沿x=375一路北上过y=120、y=50(直行)至右上角(375,0)左转90°，途经元件12、15 */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 100.0f}, /* 9: 沿顶边向西过三极管16(直行)至x=275左转90°，途经三极管16(第1次)、元件17(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 10: 沿x=275南下至y=50三极管18左转90°，途经三极管18 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  70.0f}, /* 11: 沿y=50向东至x=345左转90°，途经元件19 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  50.0f}, /* 12: 沿x=345北上至顶边三极管16左转90°(第2次) */
    { 3, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 250.0f}, /* 13: 沿顶边向西过275、245(直行)至x=95三极管22左转90°，途经元件17(第2次)、20、21、22 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 14: 沿x=95南下至y=40左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  70.0f}, /* 15: 沿y=40向东至方框西北角x=165右转90°，途经元件23(第1次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 16: 沿方框西边南下至y=80左转90° */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  80.0f}, /* 17: 沿方框底边向东至x=245左转90°，途经元件24 */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 18: 沿方框东边北上至y=40左转90° */
    { 2, 50.0f, CUSTOM_TURN, 200,  -90.0f, false, 150.0f}, /* 19: 沿y=40向西过x=165(直行)至x=95左转90°，途经元件25、23(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  80.0f}, /* 20: 沿x=95南下至y=120三极管4右转90°(第2次) */
    { 1, 50.0f, CUSTOM_TURN,  60,  -90.0f, false,  40.0f}, /* 21: 沿y=120向西至x=55左转90°进入虚线 */
    { 1, 50.0f, CUSTOM_TURN, 200,   90.0f, false, 100.0f}, /* 22: 穿过虚线南下至底边y=220右转90°，途经元件26 */
    { 1, 50.0f, CUSTOM_TURN,  60,   90.0f, false,  55.0f}, /* 23: 沿底边向西至最左下角(0,220)右转90° */
    { 1, 50.0f, STOP       , 200,    0.0f, false, 100.0f}, /* 24: 沿最左边框x=0北上至P(0,120)终点停车，途经元件27 */

#endif
};

const size_t route_count = sizeof(route) / sizeof(route[0]);
`;
