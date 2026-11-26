'use strict';
/* Single-task physical experiment console.
 *
 * The flow diagram is the panel's instrument: it draws the approved experiment
 * graph once and then only toggles classes, so polling never relayouts. The
 * human gate in this graph is physical reset only - the operator confirms that
 * the scene and device are safe, and the agent then writes that version against
 * the fresh observation. There is no human code gate. */
(() => {
  const R = window.capRender;
  const el = R.el, fold = R.fold, errorText_ = R.errorText, feedbackBody = R.feedbackBody;
  const NS = 'http://www.w3.org/2000/svg';
  let experiment = {status: 'idle', attempts: 0}, settings = null;
  let refreshing = false, draft = null, jumpTo = null;

  /* ---------------------------------------------------------------- flow graph */
  /* Prologue runs once; the cycle repeats for every version v00..v07. */
  const PROLOGUE = ['START', 'SETUP', 'CLICK', 'SNAP'];
  const CYCLE = ['RESET', 'CONFIRM', 'OBSERVE', 'PLAN', 'CODE', 'GATE',
                 'RUN', 'RESULT', 'POST', 'SUCCESS', 'BUDGET', 'NEXT'];
  const NODES = {
    START:   {cx: 46, cy: 33, w: 72, h: 30, shape: 'pill', kind: 'terminal', one: '开始 / Start'},
    SETUP:   {x: 100, y: 14, w: 146, h: 38, kind: 'human', zh: '布置初始场景与任务', en: 'Prepare scene & task'},
    CLICK:   {x: 262, y: 14, w: 146, h: 38, kind: 'human', zh: '点击「开始实验」', en: 'Start the experiment'},
    SNAP:    {x: 424, y: 14, w: 172, h: 38, kind: 'machine', zh: '冻结任务初始状态', en: 'Freeze initial state'},
    RESET:   {x: 132, y: 104, w: 196, h: 42, kind: 'human', zh: '复位准备', en: 'Reset readiness'},
    CONFIRM: {cx: 230, cy: 196, w: 168, h: 58, shape: 'diamond', kind: 'human', zh: '允许自动复位？', en: 'Return authorized?'},
    WAIT:    {x: 415, y: 175, w: 170, h: 42, kind: 'human', zh: '等待人工处理，不派发', en: 'Waiting — no dispatch'},
    OBSERVE: {x: 132, y: 248, w: 196, h: 42, kind: 'machine', zh: '读取当前场景新观测', en: 'Read fresh observation'},
    PLAN:    {x: 132, y: 316, w: 196, h: 42, kind: 'machine', zh: 'Agent 分析任务与反馈', en: 'Agent analyses feedback'},
    CODE:    {x: 132, y: 384, w: 196, h: 42, kind: 'machine', zh: 'PRE-A → 写程序 → PRE-B', en: 'Write this version'},
    GATE:    {cx: 230, cy: 478, w: 168, h: 58, shape: 'diamond', kind: 'machine', zh: '机器检查通过？', en: 'Machine check passed?'},
    RUN:     {x: 132, y: 530, w: 196, h: 42, kind: 'machine', zh: 'Runtime 授权并执行', en: 'Authorize & execute'},
    RESULT:  {x: 132, y: 598, w: 196, h: 42, kind: 'machine', zh: '保存结果·轨迹·图像', en: 'Save results & evidence'},
    POST:    {x: 132, y: 666, w: 196, h: 42, kind: 'machine', zh: 'Agent 记录 POST', en: 'Agent records POST'},
    SUCCESS: {cx: 230, cy: 760, w: 168, h: 58, shape: 'diamond', kind: 'machine', zh: '任务完成？', en: 'Task complete?'},
    BUDGET:  {cx: 230, cy: 854, w: 168, h: 58, shape: 'diamond', kind: 'machine', zh: '本轮结束？', en: 'End this run?'},
    NEXT:    {x: 132, y: 906, w: 196, h: 42, kind: 'machine', zh: '推进下一版，留存证据', en: 'Next version, keep evidence'},
    DONE:    {x: 415, y: 800, w: 170, h: 42, kind: 'machine', zh: '归档实验结果', en: 'Archive results'},
    END:     {cx: 500, cy: 900, w: 84, h: 30, shape: 'pill', kind: 'terminal', one: '结束 / End'}
  };
  /* `to` is the node an edge feeds, used to light the edge into the active node. */
  const EDGES = [
    {id: 'START>SETUP', to: 'SETUP', d: 'M82,33 H96', head: [100, 33, 'e']},
    {id: 'SETUP>CLICK', to: 'CLICK', d: 'M246,33 H258', head: [262, 33, 'e']},
    {id: 'CLICK>SNAP', to: 'SNAP', d: 'M408,33 H420', head: [424, 33, 'e']},
    {id: 'SNAP>OBSERVE', to: 'OBSERVE', d: 'M596,33 H608 V269 H336', head: [332, 269, 'w'], label: ['首次不复位 / First: no return', 465, 260]},
    {id: 'RESET>CONFIRM', to: 'CONFIRM', d: 'M230,146 V159', head: [230, 163, 's']},
    {id: 'CONFIRM>WAIT', to: 'WAIT', d: 'M314,196 H407', head: [411, 196, 'e'], label: ['否 / no', 358, 189]},
    {id: 'WAIT>RESET', to: 'RESET', d: 'M500,175 V126 a6,6 0 0 0 -6,-6 H338', head: [332, 120, 'w']},
    {id: 'CONFIRM>OBSERVE', to: 'OBSERVE', d: 'M230,225 V240', head: [230, 244, 's'], label: ['是 / yes', 244, 239]},
    {id: 'OBSERVE>PLAN', to: 'PLAN', d: 'M230,290 V308', head: [230, 312, 's']},
    {id: 'PLAN>CODE', to: 'CODE', d: 'M230,358 V376', head: [230, 380, 's']},
    {id: 'CODE>GATE', to: 'GATE', d: 'M230,426 V441', head: [230, 445, 's']},
    {id: 'GATE>CODE', to: 'CODE', d: 'M146,478 H70 a6,6 0 0 1 -6,-6 V411 a6,6 0 0 1 6,-6 H122', head: [128, 405, 'e'], label: ['否 / no', 104, 471]},
    {id: 'GATE>RUN', to: 'RUN', d: 'M230,507 V522', head: [230, 526, 's'], label: ['是 / yes', 244, 521]},
    {id: 'RUN>RESULT', to: 'RESULT', d: 'M230,572 V590', head: [230, 594, 's']},
    {id: 'RESULT>POST', to: 'POST', d: 'M230,640 V658', head: [230, 662, 's']},
    {id: 'POST>SUCCESS', to: 'SUCCESS', d: 'M230,708 V723', head: [230, 727, 's']},
    {id: 'SUCCESS>DONE', to: 'DONE', d: 'M314,760 H494 a6,6 0 0 1 6,6 V792', head: [500, 796, 's'], label: ['是 / yes', 356, 753]},
    {id: 'SUCCESS>BUDGET', to: 'BUDGET', d: 'M230,789 V817', head: [230, 821, 's'], label: ['否 / no', 244, 806]},
    {id: 'BUDGET>DONE', to: 'DONE', d: 'M314,854 H594 a6,6 0 0 0 6,-6 V827 a6,6 0 0 0 -6,-6 H591', head: [585, 821, 'w'], label: ['是 / yes', 400, 847]},
    {id: 'BUDGET>NEXT', to: 'NEXT', d: 'M230,883 V898', head: [230, 902, 's'], label: ['否 / no', 244, 896]},
    {id: 'NEXT>RESET', to: 'RESET', d: 'M132,927 H40 a6,6 0 0 1 -6,-6 V131 a6,6 0 0 1 6,-6 H122', head: [128, 125, 'e']},
    {id: 'DONE>END', to: 'END', d: 'M500,842 V881', head: [500, 885, 's']}
  ];

  // Preserve the original vertical topology and trim vertical spacing only.
  const FLOW_Y_SCALE = 0.68;
  for (const node of Object.values(NODES)) {
    if (node.y !== undefined) node.y *= FLOW_Y_SCALE;
    if (node.cy !== undefined) node.cy *= FLOW_Y_SCALE;
    node.h *= FLOW_Y_SCALE;
  }

  const make = (tag, attrs, text) => {
    const node = document.createElementNS(NS, tag);
    for (const [key, value] of Object.entries(attrs || {})) node.setAttribute(key, value);
    if (text !== undefined) node.textContent = text;
    return node;
  };
  const centre = node => [node.cx !== undefined ? node.cx : node.x + node.w / 2,
                          node.cy !== undefined ? node.cy : node.y + node.h / 2];
  const nodeElements = {}, edgeElements = {};

  function buildFlow() {
    const root = make('svg', {viewBox: '0 0 620 667', preserveAspectRatio: 'xMidYMid meet', xmlns: NS});
    for (const edge of EDGES) {
      const group = make('g', {});
      group.append(make('path', {class: 'exp-edge', d: edge.d, transform: `scale(1 ${FLOW_Y_SCALE})`}));
      const [hx, originalY, dir] = edge.head, hy = originalY * FLOW_Y_SCALE, s = 3.5;
      const points = dir === 'n' ? `${hx},${hy} ${hx - s},${hy + s * 1.6} ${hx + s},${hy + s * 1.6}` :
        dir === 's' ? `${hx},${hy} ${hx - s},${hy - s * 1.6} ${hx + s},${hy - s * 1.6}` :
        dir === 'e' ? `${hx},${hy} ${hx - s * 1.6},${hy - s} ${hx - s * 1.6},${hy + s}` :
                      `${hx},${hy} ${hx + s * 1.6},${hy - s} ${hx + s * 1.6},${hy + s}`;
      group.append(make('polygon', {class: 'exp-edge-head', points}));
      if (edge.label) group.append(make('text', {class: 'exp-edge-label', x: edge.label[1], y: edge.label[2] * FLOW_Y_SCALE}, edge.label[0]));
      edgeElements[edge.id] = group;
      root.append(group);
    }
    for (const [id, node] of Object.entries(NODES)) {
      const group = make('g', {class: `exp-node k-${node.kind}`}), [cx, cy] = centre(node);
      if (node.shape === 'diamond') {
        group.append(make('polygon', {class: 'exp-shape',
          points: `${cx},${cy - node.h / 2} ${cx + node.w / 2},${cy} ${cx},${cy + node.h / 2} ${cx - node.w / 2},${cy}`}));
        group.append(make('polygon', {class: 'exp-ring',
          points: `${cx},${cy - node.h / 2 - 5} ${cx + node.w / 2 + 6},${cy} ${cx},${cy + node.h / 2 + 5} ${cx - node.w / 2 - 6},${cy}`}));
      } else {
        const x = node.x !== undefined ? node.x : cx - node.w / 2;
        const y = node.y !== undefined ? node.y : cy - node.h / 2;
        const radius = node.shape === 'pill' ? node.h / 2 : 8;
        group.append(make('rect', {class: 'exp-shape', x, y, width: node.w, height: node.h, rx: radius}));
        group.append(make('rect', {class: 'exp-ring', x: x - 4, y: y - 4, width: node.w + 8, height: node.h + 8,
          rx: radius + 3}));
      }
      if (node.one) {
        group.append(make('text', {class: 'exp-label', x: cx, y: cy + 4, 'text-anchor': 'middle'}, node.one));
      } else {
        group.append(make('text', {class: 'exp-label', x: cx, y: cy - 2, 'text-anchor': 'middle'}, node.zh));
        group.append(make('text', {class: 'exp-label-en', x: cx, y: cy + 11, 'text-anchor': 'middle'}, node.en));
        if (node.kind === 'human') group.append(make('text', {class: 'exp-glyph', x: cx - node.w / 2 + 9, y: cy - node.h / 2 + 13}, '✋'));
      }
      nodeElements[id] = group;
      root.append(group);
    }
    $('exp-flow').replaceChildren(root);
  }

  /* Status → where we are in the graph. `through` is how far the current pass got. */
  const STAGE = {
    idle:             {node: 'SETUP', actor: 'idle'},
    capturing:        {node: 'SNAP', actor: 'machine'},
    waiting_reset:    {node: 'CONFIRM', actor: 'human'},
    returning:        {node: 'OBSERVE', actor: 'machine'},
    observing:        {node: 'OBSERVE', actor: 'machine'},
    generating:       {node: 'CODE', actor: 'machine'},
    executing:        {node: 'RUN', actor: 'machine'},
    feedback:         {node: 'RESULT', actor: 'machine'},
    reviewing:        {node: 'POST', actor: 'machine'},
    paused:           {node: 'WAIT', actor: 'human', stop: true},
    interrupted:      {node: 'WAIT', actor: 'human', stop: true},
    waiting_operator: {node: 'WAIT', actor: 'human', stop: true},
    error:            {node: 'WAIT', actor: 'error', stop: true},
    completed:        {node: 'END', actor: 'done', through: ['SUCCESS'], extra: ['DONE']},
    exhausted:        {node: 'END', actor: 'done', through: ['BUDGET'], extra: ['DONE']},
    stopped:          {node: 'END', actor: 'done'}
  };
  const TITLES = {
    idle: ['填写任务，布置好场景再开始', 'Describe the task, prepare the scene, then start.'],
    capturing: ['正在冻结初始状态', 'Freezing the initial state. No motion is sent.'],
    waiting_reset: ['等待你授权自动复位', 'Authorize return to the first experiment pose; after the return command completes, the agent writes the next version.'],
    returning: ['机械臂正在回到实验起点', 'The arm is planning back to the recorded initial joints.'],
    observing: ['正在读取当前场景新观测', 'Reading a fresh observation of the current scene.'],
    generating: ['Agent 正在写这一版程序', 'The agent is analysing and writing this version. Machine checks run before dispatch.'],
    executing: ['Runtime 正在执行这一版', 'The runtime is executing this version.'],
    feedback: ['正在保存执行结果与证据', 'Saving results, trajectory and images.'],
    reviewing: ['Agent 正在分析执行结果', 'The agent is reading the feedback and fresh images.'],
    paused: ['已暂停派发，等你确认后继续', 'Dispatch is paused. Pause is not an emergency stop. Authorize return to continue.'],
    interrupted: ['进程重启过，需重新检查并确认', 'The process restarted. Nothing was replayed — inspect the device, then authorize return.'],
    waiting_operator: ['需要你处理后才能继续', 'The agent asked for help, or the run stopped. Resolve it, then authorize return.'],
    error: ['出错了，等你处理', 'The experiment stopped on an error. Nothing is dispatched until you authorize return.'],
    completed: ['Agent 报告任务完成', 'The agent reported completion. This is an AI-reported outcome, not an independent physical verification.'],
    exhausted: ['版本额度已用完', 'The configured version budget has been used.'],
    stopped: ['实验已由你结束', 'You ended this experiment.']
  };
  const ACTORS = {idle: '尚未开始 / Not started', human: '轮到你 / Your turn',
    machine: '机器在做 / Machine working', done: '已结束 / Finished', error: '需要处理 / Needs attention'};

  /* The server records every node the run passes through, including PLAN, GATE,
     SUCCESS, BUDGET, NEXT and DONE, which have no status of their own. The panel
     replays that list one node at a time so the path lights up in order instead
     of jumping; the timestamps shown are the recorded ones, not the replay's. */
  const TRAIL_DEPTH = 4;
  const REPLAY_MS = 260;
  let shown = 0, replayTimer = null;

  function steps() { return experiment.steps || []; }

  function pumpReplay() {
    if (replayTimer) return;
    if (shown >= steps().length) return;
    replayTimer = setTimeout(() => {
      replayTimer = null;
      // Catch up fast when far behind (a reload), step by step when live.
      const behind = steps().length - shown;
      shown += behind > 12 ? behind - 6 : 1;
      render();
      pumpReplay();
    }, REPLAY_MS);
  }

  function trailNodes() {
    const list = steps().slice(0, shown).reverse(), seen = [];
    for (const item of list) {
      if (seen.length && seen[seen.length - 1].node === item.node) continue;
      seen.push(item);
      if (seen.length > TRAIL_DEPTH + 1) break;
    }
    return seen;
  }

  function renderTrailStrip(trail) {
    const host = $('exp-flow-trail');
    if (!host) return;
    host.replaceChildren();
    const catching = shown < steps().length;
    if (trail.length < 2) {
      host.append(el('span', 'exp-trail-hint', '流程推进后这里按顺序显示经过的步骤 / Steps appear here in order as the run advances'));
      return;
    }
    trail.forEach((item, index) => {
      if (index) host.append(el('span', 'exp-trail-sep', '←'));
      const node = NODES[item.node] || {};
      const chip = el('span', 'exp-trail-chip', node.one || node.zh || item.node);
      chip.dataset.depth = String(index);
      chip.dataset.kind = node.kind || 'machine';
      chip.title = (node.en || '') + ' · ' + new Date((item.time || 0) * 1000).toLocaleTimeString();
      host.append(chip);
    });
    if (catching) host.append(el('span', 'exp-trail-catching',
      `回放中 ${shown}/${steps().length} / replaying`));
  }

  function renderFlow() {
    const stage = STAGE[experiment.status] || STAGE.idle;
    const list = steps();
    if (shown > list.length) shown = list.length;
    const caught = shown >= list.length;
    const trail = trailNodes();
    // While replaying, the lit node is the one being replayed; once caught up it
    // is wherever the run actually rests.
    const current = caught ? stage.node : (trail[0] ? trail[0].node : stage.node);
    const done = new Set();
    for (const item of list.slice(0, shown)) done.add(item.node);
    if (!list.length) {
      // Nothing recorded yet: fall back to the prologue position.
      for (const id of PROLOGUE) { if (id === stage.node) break; done.add(id); }
    }
    done.delete(current);
    const rejected = current === 'CODE' && (experiment.gate || {}).rejected > 0;
    const edgeIn = rejected ? 'GATE>CODE' : EDGES.filter(e => e.to === current).map(e => e.id)[0];

    const depth = new Map();
    trail.slice(1).forEach((item, i) => {
      if (item.node !== current && !depth.has(item.node)) depth.set(item.node, i + 1);
    });

    for (const [id, group] of Object.entries(nodeElements)) {
      group.classList.toggle('is-active', id === current);
      group.classList.toggle('is-done', done.has(id));
      group.classList.toggle('k-stop', id === current && Boolean(stage.stop) && caught);
      if (depth.has(id)) group.dataset.trail = String(depth.get(id)); else delete group.dataset.trail;
      if (id === current) group.setAttribute('aria-current', 'step'); else group.removeAttribute('aria-current');
    }
    for (const edge of EDGES) {
      const group = edgeElements[edge.id];
      const wake = depth.has(edge.to) && depth.has(edge.id.split('>')[0]);
      group.classList.toggle('is-active', edge.id === edgeIn);
      group.classList.toggle('is-done', edge.id !== edgeIn && done.has(edge.to));
      if (wake) group.dataset.trail = String(depth.get(edge.to)); else delete group.dataset.trail;
      group.querySelector('.exp-edge-head').classList.toggle('is-active', edge.id === edgeIn);
    }
    renderTrailStrip(trail);
    pumpReplay();
    return Object.assign({}, stage, {node: current});
  }

  /* ------------------------------------------------------------------ readouts */
  /* A version is green only when the agent's review judged the TASK complete.
     `feedback.ok` only means the Python program did not raise: a program can run
     cleanly and move nothing, so colouring by it reads as success when it is not. */
  const VERDICTS = {
    ok:       ['任务完成 / Task complete', '这一版被判定完成任务 / Review judged the task complete'],
    failed:   ['未完成 / Not complete', '程序结束但任务未完成 / Program finished, task did not'],
    crashed:  ['程序报错 / Program error', '程序抛出异常 / The program raised'],
    blocked:  ['需人工处理 / Blocked', 'Agent 请求人工协助 / The agent asked for help'],
    pending:  ['待复盘 / Awaiting review', '已执行，等待 Agent 判定 / Executed, awaiting the verdict'],
    running:  ['执行中 / Running', '正在执行 / Executing now'],
    empty:    ['尚未执行 / Not executed', '']
  };

  function verdict(version) {
    if (!version) return 'empty';
    if (version.status !== 'finished') return 'running';
    if (version.outcome === 'task_complete') return 'ok';
    if (version.outcome === 'blocked') return 'blocked';
    if (!version.outcome) return (version.feedback || {}).ok === false ? 'crashed' : 'pending';
    return (version.feedback || {}).ok === false ? 'crashed' : 'failed';
  }

  function renderBudget() {
    const slots = $('exp-budget'), versions = experiment.versions || [];
    slots.replaceChildren();
    for (let i = 0; i < Math.max(experiment.maximum_attempts ?? 1, versions.length); i += 1) {
      const version = versions[i], id = 'v' + String(i).padStart(2, '0');
      const fill = verdict(version);
      const slot = el('button', 'exp-slot', id);
      slot.type = 'button';
      slot.dataset.fill = fill;
      slot.dataset.jump = version ? 'yes' : 'no';
      slot.disabled = !version;
      slot.title = id + '：' + VERDICTS[fill][0] + (VERDICTS[fill][1] ? ' — ' + VERDICTS[fill][1] : '');
      if (version) slot.onclick = () => { jumpTo = id; render(); };
      slots.append(slot);
    }
    const used = Math.max(experiment.attempts || 0, versions.length);
    const won = versions.filter(v => verdict(v) === 'ok').length;
    $('exp-budget-note').textContent = experiment.session_id
      ? `已启动 ${used}${experiment.maximum_attempts == null ? "" : " / " + experiment.maximum_attempts} 版 · 完成 ${won} 版 · ${experiment.synthetic ? 'Mock 软件验证 / Synthetic' : '实机 / Physical'}`
      : '版本记录 / Program versions';
  }

  const RECORDS = [['pre_a', 'PRE-A · 根因与改动 / Root cause & change'],
                   ['pre_b', 'PRE-B · 预测与风险 / Predictions & risks'],
                   ['post', 'POST · 结果分析 / Outcome analysis']];

  function renderVersions() {
    const host = $('exp-versions'), versions = experiment.versions || [];
    $('exp-versions-empty').hidden = versions.length > 0;
    for (const entry of versions) {
      const outcome = verdict(entry);
      const signature = JSON.stringify([outcome, entry.pre_a, entry.pre_b, entry.post, entry.rejected,
        entry.feedback?.call_summary?.total ?? entry.feedback?.calls?.length ?? 0]);
      let box = host.querySelector(`[data-version="${entry.version}"]`);
      if (box && box.dataset.signature === signature) continue;
      const open = box ? box.open : versions.length === 1;
      if (!box) { box = el('details', 'exp-version'); box.dataset.version = entry.version; host.append(box); }
      box.dataset.outcome = outcome;
      box.dataset.signature = signature;
      const summary = el('summary');
      summary.append(el('span', 'exp-version-id', entry.version),
        el('span', `chat-tag ${['failed', 'crashed'].includes(outcome) ? 'error' : ''}`, VERDICTS[outcome][0]),
        el('span', 'exp-version-sum', entry.reason || ''));
      const body = el('div', 'exp-version-body');
      if (entry.rejected) body.append(el('p', 'exp-rejected',
        `机器检查退回 ${entry.rejected} 次后通过，未占用尝试次数 / Bounced by the machine check ${entry.rejected} time(s) before passing; no attempt spent`));
      for (const [key, label] of RECORDS) {
        const record = el('div', 'exp-record');
        record.dataset.kind = key;
        if (!entry[key]) record.dataset.missing = 'yes';
        record.append(el('span', '', label),
          el('p', '', entry[key] || '模型未提供此项记录 / The model did not provide this record'));
        body.append(record);
      }
      const source = el('details', 'exp-source');
      source.append(el('summary', '', `这一版执行的程序 / Program executed for ${entry.version}`),
        el('pre', '', entry.source || ''));
      body.append(source);
      if (entry.feedback) {
        body.append(el('strong', '', '执行轨迹 / Execution trace'));
        body.append(feedbackBody(el('div', 'chat-body'), entry.feedback));
      }
      box.replaceChildren(summary, body);
      box.open = open;
    }
    if (jumpTo) {
      const target = host.querySelector(`[data-version="${jumpTo}"]`);
      if (target) { target.open = true; target.scrollIntoView({block: 'center', behavior: 'smooth'}); }
      jumpTo = null;
    }
  }

  const FIELDS = [
    ['关节角 / Joint angles · rad', (s,a) => s.joint_positions[a], v => v.map(n => n.toFixed(4)).join('  ')],
    ['TCP 位置 / TCP position · m', (s,a) => s.end_effector_poses[a].position, v => v.map(n => n.toFixed(4)).join('  ')],
    ['TCP 姿态 / TCP quaternion wxyz', (s,a) => s.end_effector_poses[a].quaternion_wxyz, v => v.map(n => n.toFixed(4)).join('  ')],
    ['夹爪开度 / Gripper opening', (s,a) => s.gripper_positions[a], v => Number(v).toFixed(3)],
    ['基座坐标系 / Base frame', (s,a) => s.end_effector_poses[a].frame, String]
  ];

  function renderInitial() {
    const initial = experiment.initial, table = $('exp-initial-table');
    $('exp-initial').textContent = initial ? JSON.stringify(initial.observation, null, 2) : '尚未记录 / Not captured';
    table.replaceChildren();
    if (initial) {
      const state = initial.observation.robot_state || {};
      for (const arm of Object.keys(state.joint_positions || {})) for (const [label, pick, format] of FIELDS) {
        let value;
        try { value = pick(state,arm); } catch (_) { value = undefined; }
        if (value === undefined || value === null) continue;
        const row = document.createElement('tr');
        row.append(el('th', '', arm === 'primary' ? label : arm+' · '+label), el('td', '', format(value)));
        table.append(row);
      }
      const row = document.createElement('tr');
      row.append(el('th', '', '设备绑定 / Device binding'),
        el('td', '', JSON.stringify(initial.device_binding)));
      table.append(row);
    }
    const images = $('exp-initial-images');
    if (images.dataset.session !== (experiment.session_id || '')) {
      images.replaceChildren();
      images.dataset.session = experiment.session_id || '';
    }
    if (initial && !images.childElementCount) for (const item of initial.images) {
      const figure = document.createElement('figure'), image = document.createElement('img');
      image.src = '/api/hardware/experiment/image?session=' + encodeURIComponent(experiment.session_id) +
        '&file=' + encodeURIComponent(item.file);
      image.alt = '初始场景 / Initial scene · ' + item.camera;
      image.loading = 'lazy';
      figure.append(image, el('figcaption', '', item.camera));
      images.append(figure);
    }
  }

  const TRAIL = {
    human_started_experiment: ['你开始了实验 / You started the experiment', 'human'],
    human_unlocked_controller: ['你恢复了控制器故障 / You recovered the controller fault', 'human'],
    human_manual_positioning: ['你操作了手动摆位 / You operated manual positioning', 'human'],
    human_authorized_reset: ['你授权了自动复位 / You authorized automatic return', 'human'],
    automatic_small_motion_reset: ['运动低于阈值，自动复位并继续 / Small motion: automatic return and retry', 'machine'],
    human_reset_confirmed: ['你确认了场景复位 / You confirmed the scene reset', 'human'],
    human_requested_arm_return: ['你要求机械臂回到起点 / You asked the arm to return', 'human'],
    arm_return_result: ['回位结果 / Arm return result', 'machine'],
    pause_requested: ['你暂停了派发 / You paused dispatch', 'human'],
    human_ended_experiment: ['你结束了实验 / You ended the experiment', 'end']
  };

  function renderTrail() {
    const host = $('exp-trail'), entries = experiment.interventions || [];
    host.replaceChildren();
    if (!entries.length) {
      host.append(el('li', 'exp-trail-empty', '尚无人工干预记录 / No interventions recorded yet'));
    }
    for (const entry of entries) {
      const [label, kind] = TRAIL[entry.kind] || [entry.kind, 'event'];
      const item = el('li');
      item.dataset.kind = kind;
      item.append(el('strong', '', label),
        el('small', '', new Date(entry.time * 1000).toLocaleString() +
          (entry.before_attempt !== undefined ? ` · 第 ${entry.before_attempt} 版之前 / before version ${entry.before_attempt}` : '')));
      if (entry.note) item.append(el('p', '', entry.note));
      if (entry.result) item.append(el('p', '', (entry.result.ok && entry.command_completed && entry.verified == null ? '指令完成；实测到位未验证 / Command completed; arrival unverified' : entry.result.ok && entry.verified === true ? '已验证 / Verified' : '未完成 / Not completed') +
        (entry.result.error ? ' · ' + errorText_(entry.result.error) : '')));
      host.append(item);
    }
    $('exp-events').textContent = JSON.stringify({status: experiment.status, reason: experiment.reason,
      gate: experiment.gate, assistance_validation: experiment.assistance_validation, events: experiment.events || [], history: experiment.history || []}, null, 2);
  }

  /* ------------------------------------------------------------------ controls */
  function renderControls() {
    const status = experiment.status || 'idle';
    const terminal = ['idle', 'completed', 'exhausted', 'stopped'].includes(status);
    const waiting = ['waiting_reset', 'paused', 'interrupted', 'waiting_operator', 'error'].includes(status) && !experiment.busy;
    const exhausted = experiment.maximum_attempts != null && (experiment.attempts || 0) >= experiment.maximum_attempts;
    $('exp-start').disabled = !terminal || experiment.busy;
    $('exp-goal').disabled = !terminal;
    $('exp-criteria').disabled = !terminal;
    for (const id of ['exp-preset-search', 'exp-preset-name', 'exp-speed'])
      $(id).disabled = !terminal || experiment.busy;
    if (!terminal && Number.isFinite(experiment.speed_factor))
      $('exp-speed').value = experiment.speed_factor;
    renderRecordCameras();
    renderSkillChoice();
    renderPresets();
    $('exp-review-retry-controls').hidden = !experiment.review_retry_supported;
    $('exp-retry-review').disabled = !experiment.review_retry_supported || !!experiment.busy;
    $('exp-confirm').disabled = handBusy || !waiting || !experiment.initial || !experiment.reset_authorization_supported || !!experiment.review_retry_supported;
    $('exp-confirm').textContent = !experiment.reset_authorization_supported ? '复位流程更新待重载 / Reload required for reset update' : exhausted ? '继续最终评估（不再执行） / Final assessment only'
      : '授权自动复位并继续 / Authorize reset & continue';
    $('exp-unlock').disabled = !!experiment.busy;
    $('exp-pause').disabled = terminal;
    const canAbort = state?.capabilities?.execution_abort_supported === true;
    $('exp-end').disabled = terminal || !!experiment.end_requested || (experiment.busy && !canAbort);
    $('exp-end').textContent = experiment.end_requested ? '正在结束… / Ending…' : '结束实验 / End experiment';
    $('exp-reset-card').hidden = !experiment.session_id;
    renderModelControls();
  }

  function renderModelControls() {
    const locked = !['idle', 'completed', 'exhausted', 'stopped'].includes(experiment.status || 'idle') || !!experiment.busy;
    // Editing is local. Only mutations of the saved model are locked during a run.
    for (const id of ['exp-new-profile', 'exp-mode', 'exp-cancel-edit']) $(id).disabled = !settings;
    $('exp-save-settings').disabled = locked || !settings;
    $('exp-profile-list').querySelectorAll('button.danger').forEach(button => { button.disabled = locked; });
    $('exp-settings-lock').hidden = !locked;
    $('exp-settings-lock').textContent = experiment.busy
      ? '当前实验正在运行。可以选择和编辑 API 配置；请先暂停，等待本次处理结束，再结束实验并保存。 / You can edit API settings now. Pause, wait for the current operation to finish, then end the experiment before saving.'
      : '当前实验尚未结束。可以选择和编辑 API 配置；点击「结束实验」后即可保存。 / You can edit API settings now. End the current experiment before saving.';
    $('exp-machine-settings').textContent = locked ? '锁定 · 先结束实验 / Locked until experiment ends' : '可保存 / Ready to save';
  }

  function renderSpeedRecommendation() {
    const caps = state?.capabilities;
    const arx = (caps?.embodiment || '').startsWith('arx_');
    const speed = $('exp-speed'), speedNote = document.querySelector('.exp-speed-note');
    speed.closest('label').hidden = false;
    speed.min = arx ? '0' : '0.01';
    speed.step = arx ? 'any' : '0.01';
    if (arx) speed.removeAttribute('max'); else speed.max = '1';
    speedNote.hidden = false;
    $('exp-speed-label').textContent = arx
      ? '4 · 推荐速度系数（相对默认） / Recommended speed multiplier'
      : '4 · 推荐运行速度 / Recommended speed factor';
    speedNote.textContent = arx
      ? '取值 > 0；1 为默认速度，0.1 约为默认速度的 1/10，越小越慢。 / Value > 0; 1 = default speed, 0.1 ≈ 1/10 speed; smaller is slower.'
      : '取值 0 < s ≤ 1；1 为默认速度，越小越慢。 / Range: 0 < s ≤ 1; 1 = default speed; smaller is slower.';
  }

  function renderMachineStatus() {
    const caps = state?.capabilities, connection = state?.connection;
    const arx = (caps?.embodiment || '').startsWith('arx_');
    renderSpeedRecommendation();
    $('exp-machine-connection').textContent = !caps ? '读取中 / Loading'
      : connection?.error ? '异常 / Error' : connection?.status === 'observation_read' ? '已连接 / Connected' : '等待状态 / Waiting for state';
    $('exp-machine-fault').textContent = !caps ? '未知 / Unknown'
      : caps.faulted ? '已闭锁 / Fault latched' : '未闭锁 / Not latched';
    $('exp-machine-motion').textContent = !caps ? '未知 / Unknown'
      : caps.paused ? '已暂停 / Paused' : caps.busy ? '正在处理 / Busy'
      : caps.motion_enabled ? '已授权 / Enabled' : '未授权 / Disabled';
    $('exp-machine-error').hidden = !connection?.error;
    $('exp-machine-error').textContent = connection?.error ? errorText_({message: connection.error}) : '';
    $('exp-machine-status').dataset.fault = caps?.faulted || connection?.error ? 'yes' : 'no';
  }

  function sizeWorkspace() {
    if ($('experiment-panel').hidden) return;
    const box = $('exp-workspace');
    box.style.setProperty('--exp-workspace-height', Math.max(360, window.innerHeight - Math.max(16, box.getBoundingClientRect().top) - 16) + 'px');
  }

  function renderCurrentCard() {
    // Follow live state, not the pipeline's historical replay animation.
    const cards = {
      idle: 'exp-start-card', capturing: 'exp-initial-card',
      waiting_reset: 'exp-reset-card', returning: 'exp-reset-card',
      observing: 'exp-prompt-card', generating: 'exp-prompt-card',
      executing: 'exp-versions-card', feedback: 'exp-versions-card', reviewing: 'exp-versions-card',
      paused: 'exp-reset-card', interrupted: 'exp-reset-card', waiting_operator: 'exp-reset-card', error: 'exp-reset-card',
      completed: 'exp-records-card', exhausted: 'exp-records-card', stopped: 'exp-records-card'
    };
    let current = cards[experiment.status || 'idle'] || 'exp-start-card';
    if (experiment.error_category === 'model_configuration') current = 'exp-settings';
    const manual = $('exp-manual-positioning');
    if (!manual.hidden && (manual.dataset.positioningActive === 'true' || manual.dataset.positioningRecovery === 'true'))
      current = manual.id;
    for (const card of $('exp-editor').querySelectorAll('.card')) {
      const active = card.id === current && !card.hidden;
      card.classList.toggle('is-current-step', active);
      if (active) card.setAttribute('aria-current', 'step');
      else card.removeAttribute('aria-current');
    }
  }

  function render() {
    const stage = renderFlow();
    const [title, detail] = TITLES[experiment.status] || TITLES.idle;
    $('exp-now').dataset.actor = stage.actor;
    $('exp-actor').textContent = ACTORS[stage.actor];
    const configurationError = experiment.error_category === 'model_configuration';
    const protocolError = experiment.error_category === 'model_protocol';
    const modelServiceError = experiment.error_category === 'model_service';
    $('exp-now-title').textContent = modelServiceError ? '模型请求失败 / Model request failed' : configurationError ? '模型配置错误 / Model configuration error' : protocolError ? '模型协助请求错误 / Invalid assistance request' : title;
    $('exp-now-detail').textContent = (configurationError ? '请结束当前实验后修改模型配置；不会自动重试或派发。 / End this experiment to edit model settings; no automatic retry or dispatch.' : protocolError ? '模型未能提供有效的外部问题依据，已停止；不是要求你开启运动授权。 / The model failed to identify an external problem. Stopped; no manual motion authorization is requested.' : detail) + (experiment.reason ? '\n' + errorText_({message: experiment.reason}) : '');
    if (modelServiceError) {
      const categories = {context_limit: '请求超过模型容量 / Context limit exceeded', rate_limit: '模型服务限流 / Provider rate limit', provider_unavailable: '模型服务暂不可用 / Provider unavailable', invalid_request: '模型服务拒绝请求参数 / Provider rejected request', http_error: '模型接口错误 / Model HTTP error'};
      const diagnostic = experiment.model_error;
      $('exp-now-detail').textContent = (diagnostic ? (categories[diagnostic.category] || categories.http_error) + ` · HTTP ${diagnostic.http_status} · ${diagnostic.request_bytes} bytes` : experiment.reason) +
        (experiment.review_retry_supported ? '\n可仅重试复盘，无需复位或开启运动授权。 / Retry review without resetting or enabling motion.' : '\n已停止派发；请检查模型服务。 / Dispatch stopped; check the model service.');
    }
    $('exp-device').textContent = !experiment.session_id ? '未启动 / Idle'
      : (experiment.synthetic ? 'Mock 软件验证 / Synthetic' : '实机 / Physical') + ' · ' + (experiment.attempts || 0) + (experiment.maximum_attempts == null ? '' : ' / ' + experiment.maximum_attempts);
    renderBudget();
    renderVersions();
    renderInitial();
    renderTrail();
    renderControls();
    renderMachineStatus();
    renderCurrentCard();
    sizeWorkspace();
  }

  function bind(id, fn) {
    $(id).onclick = async () => {
      const button = $(id);
      button.disabled = true;
      notice('');
      try { await fn(); await refresh(); } catch (error) { notice(error.message); } finally { renderControls(); }
    };
  }
  bind('exp-start', async () => {
    // The button itself is the authorization: no separate box to tick first.
    // A zip chosen in the browser is uploaded here, at the moment it is needed.
    if (pendingZip) await uploadPendingZip();
    const wanted = $('exp-skills-pick').value;
    if (wanted !== (skillSet.active || '')) {
      renderSkills(await request('experiment/skills-select', {name: wanted}));
    }
    await request('experiment/start', {goal: $('exp-goal').value, criteria: $('exp-criteria').value,
      confirmed: true, speed_factor: Number($('exp-speed').value),
      record_cameras: [...recordWanted]});
    if (selectedPreset) request('experiment/presets-use', {id: selectedPreset}).catch(() => {});
  });
  bind('exp-confirm', async () => {
    if (handBusy) throw Error('请等候夹爪操作反馈，整理好物品后再授权复位。 / Wait for gripper feedback and arrange the objects before authorizing reset.');
    await request('experiment/authorize-reset', {session_id: experiment.session_id, confirmed: true,
      note: $('exp-note').value, expected_attempt: experiment.attempts});
    $('exp-note').value = '';
  });
  bind('exp-unlock', () => request('unlock', {}));
  bind('exp-retry-review', () => request('experiment/retry-review', {session_id: experiment.session_id, expected_attempt: experiment.attempts}));
  bind('exp-pause', () => request('experiment/pause', {session_id: experiment.session_id}));
  bind('exp-end', () => request('experiment/end', {session_id: experiment.session_id}));

  /* ------------------------------------------------------------ model settings */
  const PROVIDERS = {openai: 'ChatGPT / OpenAI-compatible', claude: 'Claude Messages'};
  const defaultBase = provider => provider === 'claude' ? 'https://api.anthropic.com/v1' : 'https://api.openai.com/v1';

  function renderSettings() {
    if (!settings) return;
    const mode = $('exp-mode').value;
    $('exp-codex-fields').hidden = mode !== 'codex';
    $('exp-api-fields').hidden = mode !== 'api';
    $('exp-api-key-env').disabled = !settings.api_key_env_editable;
    $('exp-env-reload').hidden = !!settings.api_key_env_editable;
    const active = settings.profiles[settings.profile_id];
    $('exp-active-model').textContent = settings.mode === 'codex' ? '生效：Codex 额度 / Active: Codex plan'
      : settings.mode === 'fixture' ? '生效：Mock 软件模型 / Active: software fixture'
      : '生效 API / Active API: ' + (active ? `${active.name} · ${active.model || '未设模型 / No model'}` : settings.profile_id);
    const host = $('exp-profile-list');
    host.replaceChildren();
    for (const [id, profile] of Object.entries(settings.profiles)) {
      const row = el('div', 'exp-profile');
      row.dataset.active = settings.mode === 'api' && id === settings.profile_id ? 'yes' : 'no';
      const main = el('div', 'exp-profile-main');
      main.append(el('strong', '', profile.name || '（未命名） / (unnamed)'),
        el('small', '', `${PROVIDERS[profile.provider] || profile.provider} · ${profile.model || '未设模型 / no model'} · ` +
          (profile.key_saved ? '已存 key / key saved' : profile.environment_key_loaded ? '环境 key / env key' : '无 key / no key')));
      row.append(main);
      const edit = el('button', '', draft && draft.id === id ? '正在编辑 / Editing' : '编辑 / Edit');
      edit.type = 'button';
      edit.onclick = () => { loadDraft(id); };
      row.append(edit);
      const remove = el('button', 'danger', '删除 / Delete');
      remove.type = 'button';
      remove.onclick = () => deleteProfile(id, profile.name);
      row.append(remove);
      host.append(row);
    }
    $('exp-form').hidden = !draft;
    $('exp-form-title').textContent = draft && draft.id === null
      ? '新配置：填写名称后保存 / New profile — name it, then save'
      : '编辑配置 / Edit profile';
    renderModelControls();
  }

  function loadDraft(id) {
    const profile = settings.profiles[id];
    draft = {id};
    $('exp-profile-name').value = profile.name || '';
    $('exp-provider').value = profile.provider;
    $('exp-base-url').value = profile.base_url || '';
    $('exp-api-model').value = profile.model || '';
    $('exp-api-reasoning').value = profile.reasoning || settings.reasoning || 'medium';
    $('exp-api-key').value = '';
    $('exp-api-key-env').value = profile.api_key_env || '';
    $('exp-clear-key').checked = false;
    $('exp-key-status').textContent = profile.key_saved ? '已保存 key；留空即保留 / Key saved; blank keeps it'
      : profile.environment_key_loaded ? '可用服务端环境 key / Server environment key available'
      : '尚无 key / No key stored';
    renderSettings();
  }

  async function loadSettings() {
    settings = await request('experiment/settings');
    $('exp-mode').value = settings.mode;
    $('exp-codex-model').value = settings.codex_model;
    $('exp-reasoning').value = settings.reasoning;
    if (draft && draft.id !== null && !settings.profiles[draft.id]) draft = null;
    if (!draft && settings.mode === 'api') loadDraft(settings.profile_id);
    else renderSettings();
  }

  async function deleteProfile(id, name) {
    const others = Object.keys(settings.profiles).filter(key => key !== id);
    if (!others.length) return notice('至少要保留一套配置 / Keep at least one profile');
    if (!confirm(`删除配置「${name}」？已保存的 key 一并删除，无法撤销。\nDelete the profile "${name}"? Its saved key goes with it and this cannot be undone.`)) return;
    notice('');
    try {
      settings = await request('experiment/delete-profile',
        {delete_profile_id: id, profile_id: id === settings.profile_id ? others[0] : settings.profile_id, confirmed: true});
      if (draft && draft.id === id) draft = null;
      await loadSettings();
      notice('已删除配置 / Profile deleted');
    } catch (error) { notice(error.message); }
  }

  $('exp-mode').onchange = () => {
    // Switching to API with nothing open would leave Save with no profile to
    // save; open the active profile so the form is visible and Save just works.
    if ($('exp-mode').value === 'api' && !draft && settings) loadDraft(settings.profile_id);
    else renderSettings();
  };
  $('exp-provider').onchange = () => {
    if (draft && draft.id === null) $('exp-base-url').value = defaultBase($('exp-provider').value);
  };
  $('exp-new-profile').onclick = () => {
    draft = {id: null};
    $('exp-profile-name').value = '';
    $('exp-provider').value = 'openai';
    $('exp-base-url').value = defaultBase('openai');
    $('exp-api-model').value = '';
    $('exp-api-reasoning').value = 'medium';
    $('exp-api-key').value = '';
    $('exp-api-key-env').value = '';
    $('exp-clear-key').checked = false;
    $('exp-key-status').textContent = '新配置不会复制其他配置的 key / A new profile never copies a saved key';
    renderSettings();
    $('exp-profile-name').focus();
  };
  $('exp-cancel-edit').onclick = async () => { draft = null; notice(''); await loadSettings(); };
  bind('exp-save-settings', async () => {
    const mode = $('exp-mode').value;
    const body = {mode, codex_model: $('exp-codex-model').value, reasoning: $('exp-reasoning').value};
    if (mode === 'api') {
      if (!draft) loadDraft(settings.profile_id);
      Object.assign(body, {provider: $('exp-provider').value, profile_name: $('exp-profile-name').value,
        profile_reasoning: $('exp-api-reasoning').value,
        profile: {base_url: $('exp-base-url').value, model: $('exp-api-model').value, api_key: $('exp-api-key').value},
        clear_key: $('exp-clear-key').checked},
        draft.id === null ? {create_profile: true} : {profile_id: draft.id});
      if (settings.api_key_env_editable) body.profile.api_key_env = $('exp-api-key-env').value;
    }
    settings = await request('experiment/settings', body);
    draft = null;
    await loadSettings();
    notice('模型配置已保存 / Model settings saved');
  });

  /* --------------------------------------------------- what the agent receives */
  const PROMPT_ROWS = [
    ['task', '任务 / Task', '你填的目标和完成条件 / Your goal and completion criteria'],
    ['skills', '技能库 / Skill library', '你上传的参考资料 / Reference material you uploaded'],
    ['api', '可用接口 / Available API', '机器生成：24 个公开函数的签名与语义 / Machine-generated: signatures and semantics'],
    ['observation', '观测 / Observation', '机器生成：机器人状态与相机标定 / Machine-generated: robot state and calibration'],
    ['history', '历史 / History', '机器生成：此前每一版的程序与反馈 / Machine-generated: previous versions and feedback'],
    ['experiment', '实验进度与推荐速度 / Experiment & speed', '第几版、剩余次数、授权状态，以及你填的推荐速度 / Attempt, budget, authorization, and your recommended speed']
  ];

  async function loadPrompt() {
    const summary = $('exp-prompt-summary'), body = $('exp-prompt-body');
    body.replaceChildren();
    summary.textContent = '正在读取 / Loading…';
    try {
      // Before a run exists, preview the first message from what is in the form.
      const payload = experiment.session_id && experiment.initial ? {}
        : {goal: $('exp-goal').value, criteria: $('exp-criteria').value};
      const data = await request('experiment/preview', payload);
      const context = data.context || {};
      const images = (data.blocks || []).filter(b => b.type === 'image');
      summary.replaceChildren();
      summary.append(el('strong', '', `system ${data.system_chars} 字符 + user ${data.context_chars} 字符 + ${images.length} 张相机图像`),
        el('span', '', ' / system + user JSON + camera images'));
      const table = el('table', 'exp-table');
      for (const [key, label, note] of PROMPT_ROWS) {
        if (context[key] === undefined) continue;
        const row = document.createElement('tr');
        const size = JSON.stringify(context[key]).length;
        row.append(el('th', '', label), el('td', '', `${size} 字符 — ${note}`));
        table.append(row);
      }
      body.append(table);
      const yours = el('div', 'exp-prompt-yours');
      yours.append(el('span', '', '你填的部分，原样送出 / Your input, sent verbatim'),
        el('pre', '', (context.task || {}).language || '(empty)'));
      if (context.skills) yours.append(el('span', '', `技能库 / Skill library: ${context.skills.name} · ${context.skills.included}/${context.skills.file_count} 个文件`));
      body.append(yours);
      fold(body, 'system 消息全文 / Full system message', data.system);
      fold(body, 'user 消息 JSON 全文 / Full user JSON', context);
    } catch (error) {
      summary.textContent = errorText_({message: error.message});
    }
  }
  $('exp-prompt-refresh').onclick = loadPrompt;

  /* ------------------------------------------------------------ saved cases */
  /* Selection is by NAME: each saved case is a named row you click. The name
     field is only used when saving, so picking and naming never share a box. */
  let presets = [], skillSet = {libraries: []}, selectedPreset = null, pendingZip = null;

  const presetById = id => presets.find(p => p.id === id);
  const TEXT_SUFFIX = /\.(md|markdown|txt|py|json|ya?ml|rst|cfg|ini|toml)$/i;

  function currentMatches(preset) {
    return preset && preset.goal === $('exp-goal').value.trim()
      && preset.criteria === $('exp-criteria').value.trim()
      && (preset.skills || '') === chosenLibrary()
      && Number(preset.speed || 0.05) === Number($('exp-speed').value)
      && (preset.record || '') === [...recordWanted].join(',');
  }

  function locked() {
    return !['idle', 'completed', 'exhausted', 'stopped'].includes(experiment.status || 'idle')
      || Boolean(experiment.busy);
  }

  function renderPresets() {
    const host = $('exp-preset-list');
    const filter = $('exp-preset-search').value.trim().toLowerCase();
    const shown = presets.filter(p => !filter || p.name.toLowerCase().includes(filter)
      || (p.goal || '').toLowerCase().includes(filter));
    $('exp-preset-count').textContent = presets.length ? presets.length + ' 个 / saved' : '暂无 / none';
    $('exp-preset-box').hidden = false;
    host.replaceChildren();
    if (!presets.length) {
      host.append(el('p', 'exp-trail-hint', '还没有常用任务。填好上面三步后，在下方「存为常用任务」里命名保存。 / None yet — fill the three steps, then name and save it below.'));
    } else if (!shown.length) {
      host.append(el('p', 'exp-trail-hint', '没有匹配的常用任务 / No case matches that search'));
    }
    for (const preset of shown) {
      const row = el('div', 'exp-preset-row');
      row.dataset.active = preset.id === selectedPreset ? 'yes' : 'no';
      const main = el('button', 'exp-preset-main');
      main.type = 'button';
      main.disabled = locked();
      main.append(el('strong', '', preset.name),
        el('small', '', (preset.goal || '').slice(0, 70) + (preset.skills ? ' · 参考资料 ' + preset.skills : '')));
      main.onclick = () => applyPreset(preset);
      row.append(main);
      const drop = el('button', 'danger', '删除 / Delete');
      drop.type = 'button';
      drop.disabled = locked();
      drop.onclick = () => deletePreset(preset);
      row.append(drop);
      host.append(row);
    }
    const picked = presetById(selectedPreset);
    const named = $('exp-preset-name').value.trim();
    $('exp-preset-save').disabled = locked() || !named;
    $('exp-preset-save').textContent = named && presets.some(p => p.name === named) ? '更新 / Update' : '保存 / Save';
    $('exp-preset-state').textContent = !named ? '填名称后可保存 / Name it to save'
      : picked && !currentMatches(picked) ? `「${picked.name}」已被修改 / "${picked.name}" has unsaved edits` : '';
  }

  function applyPreset(preset) {
    selectedPreset = preset.id;
    $('exp-goal').value = preset.goal;
    $('exp-criteria').value = preset.criteria;
    $('exp-preset-name').value = preset.name;
    $('exp-speed').value = preset.speed || 0.05;
    recordWanted = new Set((preset.record || '').split(',').map(x => x.trim()).filter(Boolean));
    renderRecordCameras();
    pendingZip = null;
    if (preset.skills && (skillSet.libraries || []).some(l => l.name === preset.skills)) {
      $('exp-skills-pick').value = preset.skills;
    } else {
      $('exp-skills-pick').value = '';
    }
    renderSkillChoice();
    renderPresets();
    renderSpeedRecommendation();
  }

  async function deletePreset(preset) {
    if (!confirm(`删除常用任务「${preset.name}」？\n\nDelete the saved case "${preset.name}"?`)) return;
    try {
      presets = (await request('experiment/presets-delete', {id: preset.id, confirmed: true})).presets || [];
      if (selectedPreset === preset.id) selectedPreset = null;
      renderPresets();
    } catch (error) { notice(error.message); }
  }

  async function loadPresets() {
    try { presets = (await request('experiment/presets')).presets || []; renderPresets(); }
    catch (error) { notice(error.message); }
  }

  $('exp-preset-search').oninput = renderPresets;
  $('exp-preset-name').oninput = renderPresets;
  for (const id of ['exp-goal', 'exp-criteria', 'exp-speed']) $(id).oninput = () => {
    renderPresets();
    renderSpeedRecommendation();
  };

  bind('exp-preset-save', async () => {
    const name = $('exp-preset-name').value.trim();
    if (!name) throw Error('先给这个任务起个名字 / Name this case first');
    presets = (await request('experiment/presets', {
      name, goal: $('exp-goal').value, criteria: $('exp-criteria').value,
      skills: chosenLibrary(), speed: Number($('exp-speed').value),
      record: [...recordWanted].join(',')})).presets || [];
    const saved = presets.find(p => p.name === name);
    selectedPreset = saved ? saved.id : null;
    renderPresets();
    notice('常用任务已保存 / Case saved');
  });

  /* ------------------------------------------------- reference material (zip) */
  /* The file is chosen and previewed in the browser; it only reaches the server
     when the run starts, so picking a zip never changes anything on its own. */
  const chosenLibrary = () => pendingZip ? pendingZip.name : $('exp-skills-pick').value;

  function readZipEntries(bytes) {
    const view = new DataView(bytes.buffer), text = new TextDecoder();
    let eocd = -1;
    for (let i = bytes.length - 22; i >= 0 && i > bytes.length - 66000; i -= 1) {
      if (view.getUint32(i, true) === 0x06054b50) { eocd = i; break; }
    }
    if (eocd < 0) throw Error('这不是一个有效的 zip 文件 / Not a valid zip file');
    let offset = view.getUint32(eocd + 16, true);
    const count = view.getUint16(eocd + 10, true), entries = [];
    for (let i = 0; i < count; i += 1) {
      if (view.getUint32(offset, true) !== 0x02014b50) break;
      const nameLength = view.getUint16(offset + 28, true);
      const extra = view.getUint16(offset + 30, true), comment = view.getUint16(offset + 32, true);
      const name = text.decode(bytes.subarray(offset + 46, offset + 46 + nameLength));
      entries.push({name, bytes: view.getUint32(offset + 24, true)});
      offset += 46 + nameLength + extra + comment;
    }
    return entries;
  }

  function renderSkillChoice() {
    const preview = $('exp-skills-preview'), picker = $('exp-skills-pick');
    picker.disabled = locked() || Boolean(pendingZip);
    $('exp-skills-choose').disabled = locked();
    preview.replaceChildren();
    if (pendingZip) {
      const head = el('div', 'exp-attach-chosen');
      head.append(el('strong', '', '已选择 / Chosen: ' + pendingZip.file),
        el('small', '', `${pendingZip.keep.length} 个文本文件将发给 Agent，${pendingZip.drop.length} 个会被忽略 · ${Math.round(pendingZip.size / 1024)} KB`));
      const clear = el('button', 'quiet', '取消选择 / Clear');
      clear.type = 'button';
      clear.disabled = locked();
      clear.onclick = () => { pendingZip = null; $('exp-skills-file').value = ''; renderSkillChoice(); renderPresets(); };
      head.append(clear);
      preview.append(head);
      const list = el('div', 'exp-skills-list');
      for (const entry of pendingZip.keep.slice(0, 40)) {
        const row = el('div', 'exp-skill-row');
        row.append(el('strong', '', entry.name), el('small', '', entry.bytes + ' B'));
        list.append(row);
      }
      if (pendingZip.keep.length > 40) list.append(el('p', 'exp-trail-hint', `…另有 ${pendingZip.keep.length - 40} 个`));
      for (const entry of pendingZip.drop.slice(0, 8)) {
        list.append(el('p', 'exp-skill-skipped', '将忽略 / Ignored: ' + entry.name));
      }
      preview.append(list);
      preview.append(el('p', 'exp-trail-hint', '点「开始实验」时才会上传 / Uploaded when you start the experiment'));
      return;
    }
    const name = picker.value;
    const library = (skillSet.libraries || []).find(l => l.name === name);
    if (!library) {
      preview.append(el('p', 'exp-trail-hint', '本次不使用参考资料 / This run carries no reference material'));
      return;
    }
    preview.append(el('p', 'exp-records-path', `${library.file_count} 个文件 · ${Math.round((library.bytes || 0) / 1024)} KB · ${library.path}`));
    const list = el('div', 'exp-skills-list');
    for (const file of (library.files || []).slice(0, 40)) {
      const row = el('div', 'exp-skill-row');
      row.append(el('strong', '', file.title || file.file), el('small', '', file.file));
      list.append(row);
    }
    preview.append(list);
  }

  $('exp-skills-choose').onclick = () => $('exp-skills-file').click();
  $('exp-skills-pick').onchange = () => { pendingZip = null; renderSkillChoice(); renderPresets(); };
  $('exp-skills-file').onchange = async () => {
    const file = $('exp-skills-file').files[0];
    if (!file) return;
    notice('');
    try {
      const bytes = new Uint8Array(await file.arrayBuffer());
      const entries = readZipEntries(bytes).filter(e => !e.name.endsWith('/'));
      const usable = e => TEXT_SUFFIX.test(e.name) && !e.name.split('/').some(p => p.startsWith('.'))
        && !e.name.split('/').includes('..');
      const keep = entries.filter(usable), drop = entries.filter(e => !usable(e));
      if (!keep.length) throw Error('这个 zip 里没有可用的文本文件 / This zip has no usable text files');
      pendingZip = {file: file.name, name: file.name.replace(/\.zip$/i, ''), size: file.size,
                    keep, drop, bytes};
      renderSkillChoice();
      renderPresets();
    } catch (error) {
      pendingZip = null;
      $('exp-skills-file').value = '';
      notice(error.message);
      renderSkillChoice();
    }
  };

  async function uploadPendingZip() {
    let binary = '';
    for (let i = 0; i < pendingZip.bytes.length; i += 0x8000) {
      binary += String.fromCharCode.apply(null, pendingZip.bytes.subarray(i, i + 0x8000));
    }
    const value = await request('experiment/skills', {name: pendingZip.name, archive_base64: btoa(binary)});
    pendingZip = null;
    $('exp-skills-file').value = '';
    renderSkills(value);
    return value;
  }

  /* -------------------------------------------------------- stored libraries */
  function renderSkills(value) {
    skillSet = value;
    const picker = $('exp-skills-pick'), chosen = picker.value;
    picker.replaceChildren();
    const none = document.createElement('option');
    none.value = ''; none.textContent = '不使用参考资料 / No reference material';
    picker.append(none);
    for (const library of value.libraries || []) {
      const option = document.createElement('option');
      option.value = library.name;
      option.textContent = `${library.name} · ${library.file_count} 个文件`;
      picker.append(option);
    }
    const names = (value.libraries || []).map(l => l.name);
    picker.value = names.includes(chosen) ? chosen : (value.active || '');
    const state = $('exp-skills-state');
    if (state) state.textContent = value.installed
      ? `启用中 / Active · ${value.active} · ${value.file_count} 个文件` : '未启用 / None active';
    const host = $('exp-skills-libraries');
    if (host) {
      host.replaceChildren();
      if (!(value.libraries || []).length) {
        host.append(el('p', 'exp-trail-hint', '还没有保存过参考资料。在「开始实验」里选一个 zip 即可。 / None saved yet — pick a zip in Start experiment.'));
      }
      for (const library of value.libraries || []) {
        const row = el('div', 'exp-profile');
        row.dataset.active = library.active ? 'yes' : 'no';
        const main = el('div', 'exp-profile-main');
        main.append(el('strong', '', library.name),
          el('small', '', `${library.file_count} 个文件 · ${Math.round((library.bytes || 0) / 1024)} KB · ${new Date((library.installed_s || 0) * 1000).toLocaleString()}`));
        row.append(main);
        const drop = el('button', 'danger', '删除 / Delete');
        drop.type = 'button';
        drop.onclick = async () => {
          if (!confirm(`删除参考资料「${library.name}」？\n\nDelete the reference material "${library.name}"?`)) return;
          try { renderSkills(await request('experiment/skills-remove', {confirmed: true, name: library.name})); }
          catch (error) { notice(error.message); }
        };
        row.append(drop);
        host.append(row);
      }
    }
    renderSkillChoice();
  }
  async function loadSkills() {
    try { renderSkills(await request('experiment/skills')); } catch (error) { notice(error.message); }
  }

  /* ------------------------------------------------------- recording toggles */
  let cameraNames = [], recordWanted = new Set();

  function renderRecordCameras() {
    const host = $('exp-record-cameras');
    host.replaceChildren();
    if (!cameraNames.length) {
      host.append(el('p', 'exp-trail-hint', '未检测到相机 / No cameras reported'));
      return;
    }
    for (const name of cameraNames) {
      const label = el('label', 'exp-check');
      const box = document.createElement('input');
      box.type = 'checkbox';
      box.checked = recordWanted.has(name);
      box.disabled = locked();
      box.onchange = () => {
        if (box.checked) recordWanted.add(name); else recordWanted.delete(name);
        renderPresets();
      };
      label.append(box, document.createTextNode(name));
      host.append(label);
    }
  }

  async function loadCameras() {
    try {
      const state = await request('state');
      const caps = state.capabilities || {};
      $('exp-unlock').hidden = caps.embodiment === 'arx_x5_2023_dual';
      cameraNames = caps.cameras || [];
      renderRecordCameras();
    } catch (error) { /* the control panel reports connection problems already */ }
  }

  /* ----------------------------------------------------------- record archive */
  const RECORD_STATUS = {completed: 'AI 报告完成 / AI reported complete', exhausted: '额度用尽 / Budget exhausted',
    stopped: '人工结束 / Ended by operator', error: '出错 / Error', waiting_operator: '等待人工 / Waiting',
    waiting_reset: '等待复位 / Waiting for reset', interrupted: '重启中断 / Interrupted'};

  let recordRows = [], recordPicked = new Set();

  function renderRecordBulk() {
    const count = recordPicked.size;
    $('exp-records-selected').textContent = count ? `已选 ${count} 条 / ${count} selected` : '未选择 / none selected';
    for (const id of ['exp-records-tag', 'exp-records-delete']) $(id).disabled = !count;
    const all = $('exp-records-all'), eligible = recordRows.filter(r => !r.locked).length;
    all.checked = count > 0 && count === eligible;
    all.indeterminate = count > 0 && count < eligible;
  }

  function renderRecords() {
    const host = $('exp-records');
    host.replaceChildren();
    $('exp-records-path').textContent = recordRows.length
      ? '目录 / Directory: ' + recordRows[0].directory.replace(/\/[^/]+$/, '') : '';
    if (!recordRows.length) {
      host.append(el('p', 'exp-trail-hint', '尚无实验记录 / No records yet'));
      renderRecordBulk();
      return;
    }
    for (const row of recordRows) {
      const box = el('details', 'exp-record-row');
      box.dataset.success = row.succeeded ? 'yes' : 'no';
      const summary = el('summary');
      const pick = document.createElement('input');
      pick.type = 'checkbox';
      pick.className = 'exp-record-pick';
      pick.checked = recordPicked.has(row.session_id);
      pick.disabled = row.locked;
      if (row.locked) pick.title = '实验进行中，先结束才能删除 / Still running; end it before deleting';
      pick.onclick = event => {
        event.stopPropagation();
        if (pick.checked) recordPicked.add(row.session_id); else recordPicked.delete(row.session_id);
        renderRecordBulk();
      };
      summary.append(pick,
        el('span', 'exp-record-when', new Date((row.updated_s || 0) * 1000).toLocaleString()),
        el('span', `chat-tag ${row.succeeded ? '' : 'error'}`, row.succeeded ? '完成 / Complete' : '未完成 / Not complete'),
        el('span', 'exp-version-sum', row.goal || '(no goal)'));
      for (const tag of row.tags || []) summary.append(el('span', 'exp-record-tag', tag));
      if ((row.videos || []).length) summary.append(el('span', 'exp-record-meta', '🎥 ' + row.videos.length));
      summary.append(el('span', 'exp-record-meta',
        `${row.versions} 版 · ${RECORD_STATUS[row.status] || row.status}${row.synthetic ? ' · Mock' : ''}${row.active ? ' · 当前 / current' : ''}`));
      const body = el('div', 'exp-version-body');
      body.append(el('p', 'exp-records-path', '路径 / Path: ' + row.directory));
      const actions = el('div', 'actions');
      const open = el('button', '', '查看记录 / View record');
      open.type = 'button';
      open.onclick = async () => {
        open.disabled = true;
        try {
          const full = await request('experiment/record?id=' + encodeURIComponent(row.session_id));
          for (const version of full.versions || []) {
            for (const [camera, video] of Object.entries(version.videos || {})) {
              const clip = el('div', 'exp-video');
              clip.append(el('strong', '', `${version.version} · ${camera}`));
              if (video.written) {
                const player = document.createElement('video');
                player.controls = true;
                player.preload = 'none';
                player.src = `/api/hardware/experiment/video?session=${encodeURIComponent(row.session_id)}&file=${encodeURIComponent(video.file)}`;
                clip.append(player);
                clip.append(el('small', '', `${video.frames} 帧 · ${video.seconds}s · ${video.fps}fps`));
              } else {
                clip.append(el('small', 'chat-error', '未写入 / not written' + (video.error ? ' · ' + video.error : '')));
              }
              body.append(clip);
            }
          }
          fold(body, '完整记录 / Full record', full, true);
        } catch (error) { body.append(el('p', 'chat-error', errorText_({message: error.message}))); }
        finally { open.disabled = false; }
      };
      const download = el('a', 'exp-download', '导出 zip / Export zip');
      download.href = '/api/hardware/experiment/export?id=' + encodeURIComponent(row.session_id);
      download.download = row.session_id + '.zip';
      const drop = el('button', 'danger', '删除记录 / Delete record');
      drop.type = 'button';
      drop.onclick = async () => {
        if (!confirm(`删除这次实验的全部记录和图像？无法撤销。\n${row.goal}\n\nDelete this run's records and images? This cannot be undone.`)) return;
        drop.disabled = true;
        try { await request('experiment/records-delete', {id: row.session_id, confirmed: true}); await loadRecords(); }
        catch (error) { notice(error.message); drop.disabled = false; }
      };
      actions.append(open, download, drop);
      body.append(actions);
      box.append(summary, body);
      host.append(box);
    }
    renderRecordBulk();
  }

  async function loadRecords() {
    try {
      recordRows = (await request('experiment/records')).records || [];
      const live = new Set(recordRows.map(r => r.session_id));
      recordPicked = new Set([...recordPicked].filter(id => live.has(id)));
      renderRecords();
    } catch (error) { $('exp-records').replaceChildren(el('p', 'chat-error', errorText_({message: error.message}))); }
  }

  $('exp-records-all').onchange = () => {
    // Never select a run that is still in progress; it cannot be deleted.
    recordPicked = $('exp-records-all').checked
      ? new Set(recordRows.filter(r => !r.locked).map(r => r.session_id)) : new Set();
    renderRecords();
  };
  bind('exp-records-tag', async () => {
    const tags = $('exp-records-tags').value.split(',').map(t => t.trim()).filter(Boolean);
    recordRows = (await request('experiment/records-tag',
      {ids: [...recordPicked], tags, confirmed: true})).records || [];
    renderRecords();
    notice(tags.length ? '标签已更新 / Tags applied' : '标签已清除 / Tags cleared');
  });
  bind('exp-records-delete', async () => {
    const ids = [...recordPicked];
    if (!confirm(`删除所选 ${ids.length} 条实验记录及其图像和录像？无法撤销。\n\nDelete ${ids.length} selected runs with their images and videos? This cannot be undone.`)) return;
    recordRows = (await request('experiment/records-delete-many', {ids, confirmed: true})).records || [];
    recordPicked = new Set();
    renderRecords();
    notice(`已删除 ${ids.length} 条记录 / Deleted ${ids.length} records`);
  });
  $('exp-records-refresh').onclick = loadRecords;

  /* ---------------------------------------------------------------------- poll */
  async function refresh() {
    if (refreshing) return;
    refreshing = true;
    try {
      const next = await request('experiment/state');
      const changed = next.session_id !== experiment.session_id;
      if (changed) {
        $('exp-versions').replaceChildren();
        shown = 0;
        if (next.goal) $('exp-goal').value = next.goal;
        if (next.criteria) $('exp-criteria').value = next.criteria;
      }
      const settled = experiment.busy && !next.busy;
      experiment = next;
      render();
      // A finished run is a new archive entry; refresh the list once, not per poll.
      if (changed || settled) loadRecords();
    } finally { refreshing = false; }
  }

  buildFlow();
  window.addEventListener('resize', sizeWorkspace);
  render();
  loadSettings().catch(error => notice(error.message));
  loadPresets();
  loadSkills();
  loadCameras();
  loadRecords();
  refresh().catch(error => notice(error.message));
  setInterval(() => {
    if (!$('experiment-panel').hidden) refresh().catch(error => { $('exp-now-detail').textContent = error.message; });
  }, 1200);
})();
