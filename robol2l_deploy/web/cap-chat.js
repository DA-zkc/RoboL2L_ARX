/* Read-only conversation view. Expanding a card never executes a program. */
(() => {
  let session = null, signature = '', cards = new Map(), details = new Map();
  const el = (tag, className, text) => {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
  };
  const pretty = value => typeof value === 'string' ? value : JSON.stringify(value, null, 2);
  const fold = (parent, title, content, open = false) => {
    const box = el('details', 'chat-detail'); box.open = open;
    box.append(el('summary', '', title), el('pre', '', pretty(content))); parent.append(box);
  };
  const errorText = value => {
    const labels = {
      model_assistance_protocol_error:'模型协助请求格式错误：未提供有效的外部问题类别与证据。不是运动授权失败；请检查模型输出。 / Invalid assistance decision: missing external-problem type or evidence. This is not a motion authorization failure.',
      model_configuration_error_key_format:'模型配置错误：API key 含非 ASCII、空白或控制字符。请重新填写凭据；未进入程序检查。 / Model configuration error: the API key contains non-ASCII, whitespace or control characters. Re-enter the credential; program validation was not reached.',
      model_configuration_error_key_missing:'模型配置错误：未配置 API key。 / Model configuration error: API key is missing.',
      model_configuration_error_model_missing:'模型配置错误：未填写模型名称。 / Model configuration error: model name is missing.',
      model_configuration_error_request_header:'模型配置错误：请求头格式或编码无效。 / Model configuration error: invalid request header format or encoding.',
      model_configuration_error_http_401:'模型配置错误：服务端拒绝认证（HTTP 401）。请核对 key 与服务地址。 / Model configuration error: authentication rejected (HTTP 401). Check the key and endpoint.',
      model_configuration_error_http_403:'模型配置错误：服务端拒绝访问（HTTP 403）。请核对账户权限与服务地址。 / Model configuration error: access denied (HTTP 403). Check account permissions and endpoint.',

      runtime_fault_latched:'执行层当时停止了程序，具体失败见下方 API 调用。 / The runtime stopped this program; see the failed API call below.',
      paused_before_dispatch:'程序在下一次派发前暂停。 / Paused before the next dispatch.',
      process_restarted_no_replay:'进程重启后旧任务已结束，没有重放动作。 / The process restarted; previous actions were not replayed.',
      codex_request_timeout:'Codex 请求超时，本次没有生成程序，也没有占用尝试次数。确认复位后可重试。 / Codex timed out; no program was generated and no attempt was spent. Confirm a reset to retry.',
      codex_request_failed:'Codex 请求失败。检查网络、ChatGPT 登录和额度后重试；不会自动切到付费 API。 / Codex request failed. Check network, ChatGPT login and quota; it never falls back to a paid API.',
      codex_usage_limit_reached:'Codex 额度或速率已达上限，请在 Codex 查看后重试。 / Codex quota or rate limit reached.',
      codex_chatgpt_login_required:'Codex 的 ChatGPT 登录不可用，请在本机执行 codex login。 / Codex needs a ChatGPT login; run codex login on this machine.',
      three_invalid_model_responses:'模型连续三次没有通过机器检查，已停下等待你处理。 / Three responses failed the machine check; stopped for you.',
      runtime_fault_or_unknown_outcome:'控制器故障或执行结果未知，已停止派发。确认按钮不会清除故障。 / Controller fault or unknown outcome; dispatch stopped. Confirming does not clear a fault.',
      confirm_initial_scene_before_first_attempt:'已冻结初始状态，等你确认场景后再写第一版。 / Initial state frozen; confirm the scene and the first version gets written.',
      reset_confirmed:'复位已确认。 / Reset confirmed.'
    };
    return value?.message_zh || labels[value?.message] || value?.message || pretty(value);
  };
  /* Execution receipts: one card per public API call, with the failing line. */
  function feedbackBody(body, value) {
    const feedback = value || {};
    if (feedback.error) body.append(el('p', 'chat-error', errorText(feedback.error)));
    if (feedback.key_results) fold(body, '定位、几何与IK结果 / Localization, geometry & IK results', feedback.key_results);
    const calls = feedback.calls || [];
    if (calls.length) {
      const list = el('div', 'chat-calls');
      for (const call of calls) {
        const box = el('details', `chat-call ${call.ok ? 'passed' : 'failed'}`);
        const timing=call.tool==='plan_motion'?call.output?.diagnostics:null;
        const speed=timing?.speed_factor!=null?` · 速度 / Speed ${timing.speed_factor} · ${Number(timing.duration_s).toFixed(2)} s`:'';
        const label = `${call.ok ? '✓' : '!'} ${call.tool} · ${call.line ? '第 ' + call.line + ' 行 / line ' + call.line + ' · ' : ''}${call.ok ? '返回成功 / OK' : '失败 / Failed'}${speed}`;
        box.append(el('summary', '', label));
        const error = call.output?.error || call.error;
        if (error) box.append(el('p', 'chat-error', errorText(error)));
        fold(box, '调用参数 / Arguments', call.inputs || {});
        fold(box, '返回结果 / Returned value', call.output ?? call.error);
        list.append(box);
      }
      body.append(list);
    }
    if (feedback.stdout) fold(body, '程序输出 print / Printed output', feedback.stdout);
    fold(body, '程序 result / Program result', feedback.result ?? null);
    fold(body, '完整执行记录 / Full execution record', feedback);
    return body;
  }
  /* Shared with the experiment panel so both views read receipts identically. */
  window.capRender = {el, fold, pretty, errorText, feedbackBody};
  function fill(body, role, data) {
    body.replaceChildren();
    if (role === 'input') {
      let context = data.context?.context || {};
      try { if (typeof context === 'string') context = JSON.parse(context); } catch (_) {}
      if (context.task?.language) body.append(el('p', 'chat-prose', context.task.language));
      const observed = context.observation || {};
      body.append(el('p', 'chat-meta', `观测 / Observation: ${context.observation_id || '—'} · 相机 / Cameras: ${Object.keys(observed.cameras || {}).join(', ') || '—'}`));
      fold(body, '任务与机器人状态 / Task & robot state', {task:context.task, robot_state:observed.robot_state});
      fold(body, '相机观测与标定信息 / Camera observations', observed.cameras || {});
      fold(body, '提供给 AI 的系统说明 / System instructions', data.context?.system || '');
      fold(body, '完整输入（含 API 和历史反馈） / Full model input', context);
    } else if (role === 'ai') {
      body.append(el('p', 'chat-prose', data.decision?.reason || '尚未返回决策 / Awaiting a decision'));
      if (data.decision?.source) fold(body, '生成的 Python 程序 / Generated Python', data.decision.source, true);
    } else {
      feedbackBody(body, data.feedback);
    }
  }
  function card(row, role, cap, request) {
    const key = `${row.turn}:${role}`, old = cards.get(key);
    if (old?.revision === row.revision) return old.node;
    const node = old?.node || el('details', `chat-card chat-${role}`);
    const open = node.open;
    const heading = el('summary', 'chat-summary');
    const labels = {
      model_assistance_protocol_error:'模型协助请求格式错误：未提供有效的外部问题类别与证据。不是运动授权失败；请检查模型输出。 / Invalid assistance decision: missing external-problem type or evidence. This is not a motion authorization failure.',
      model_configuration_error_key_format:'模型配置错误：API key 含非 ASCII、空白或控制字符。请重新填写凭据；未进入程序检查。 / Model configuration error: the API key contains non-ASCII, whitespace or control characters. Re-enter the credential; program validation was not reached.',
      model_configuration_error_key_missing:'模型配置错误：未配置 API key。 / Model configuration error: API key is missing.',
      model_configuration_error_model_missing:'模型配置错误：未填写模型名称。 / Model configuration error: model name is missing.',
      model_configuration_error_request_header:'模型配置错误：请求头格式或编码无效。 / Model configuration error: invalid request header format or encoding.',
      model_configuration_error_http_401:'模型配置错误：服务端拒绝认证（HTTP 401）。请核对 key 与服务地址。 / Model configuration error: authentication rejected (HTTP 401). Check the key and endpoint.',
      model_configuration_error_http_403:'模型配置错误：服务端拒绝访问（HTTP 403）。请核对账户权限与服务地址。 / Model configuration error: access denied (HTTP 403). Check account permissions and endpoint.',
input:'我们提供的信息 / Context', ai:row.kind === 'assist' ? 'AI 请求协助 / Assistance' : row.kind === 'finish' ? 'AI 完成说明 / Completion claim' : 'AI 说明与程序 / AI program', feedback:'执行反馈 / Execution feedback'};
    const icons = {input:'IN', ai:'AI', feedback:'API'};
    heading.append(el('span', 'chat-avatar', icons[role]));
    const words = el('span', 'chat-words');
    words.append(el('strong', '', labels[role]), el('small', '', `第 ${row.turn + 1} 轮 / Turn ${row.turn + 1}${row.version ? ' · ' + row.version : ''}`));
    let synopsis = role === 'input' ? '任务、公开 API、相机观测、机器人状态与历史反馈 / Task, API, observations & feedback' :
      role === 'ai' ? row.reason || '等待 AI 返回 / Waiting for AI…' :
      row.failure?.message_zh || `${row.call_count} 次 API 调用 · ${row.program_ok ? '程序已运行完成；任务状态见顶部 / Program returned; see task status above' : '程序未正常完成 / Program did not finish normally'}`;
    words.append(el('span', 'chat-synopsis', synopsis)); heading.append(words);
    if (role === 'feedback') heading.append(el('span', `chat-tag ${row.failure || row.program_ok === false ? 'error' : 'success'}`, row.failure || row.program_ok === false ? '未通过 / Failed' : '已返回 / Returned'));
    const body = el('div', 'chat-body'); node.replaceChildren(heading, body); node.open = open;
    let loaded = false, loading = false;
    async function load() {
      if (loaded || loading || !node.open) return;
      loading = true; body.replaceChildren(el('p', '', '正在读取记录 / Loading record…'));
      const token = `${cap.session_id}:${row.turn}:${row.revision}`;
      try {
        if (!details.has(token)) details.set(token, request(`cap/turn?session=${encodeURIComponent(cap.session_id)}&turn=${row.turn}`));
        const data = await details.get(token); fill(body, role, data); loaded = true;
      } catch (error) {
        details.delete(token); body.replaceChildren(el('p', 'chat-error', error.message));
        const retry = el('button', '', '重新加载记录 / Reload record'); retry.onclick = () => load(); body.append(retry);
      } finally { loading = false; }
    }
    node.ontoggle = load;
    if (open) load();
    cards.set(key, {node, revision:row.revision}); return node;
  }
  window.renderCapConversation = (cap, request) => {
    const get = id => document.getElementById(id), view = cap.presentation;
    const title = view ? `${view.title} / ${view.title_en}` : cap.status === 'idle' ? '未启动 / Idle' : cap.status;
    get('cap-state-label').textContent = title;
    get('cap-outcome-title').textContent = title;
    get('cap-outcome').dataset.tone = view?.tone || 'neutral';
    get('cap-feedback').textContent = view?.reason || '选择任务后开始。 / Choose a task to start.';
    get('cap-next-step').textContent = view?.next_step || '无需重启程序。 / No restart needed.';
    get('cap-next-en').textContent = view?.next_step_en || '';
    get('cap-status').textContent = JSON.stringify({...cap, recent_feedback:undefined, conversation:undefined}, null, 2);
    get('cap-start').textContent = cap.status === 'running' ? '正在运行 / Running' : cap.session_id ? '新建任务 / Start new task' : '开始 / Start';
    get('cap-pause').disabled = cap.status !== 'running';
    const rows = cap.conversation || [], interventions=cap.continuations||[], nextSignature = JSON.stringify([cap.session_id, rows, interventions]);
    if (signature === nextSignature) return;
    signature = nextSignature;
    const chat = get('cap-chat'), atBottom = chat.scrollHeight - chat.scrollTop - chat.clientHeight < 50;
    if (session !== cap.session_id) {
      session = cap.session_id; cards.clear(); details.clear();
      const task = {physical_task:'pick-place',physical_probe:'probe',physical_readonly:'visual',visual_preparation:'visual'}[cap.task?.metadata?.mode] ||
        {'green-cube-to-plate':'pick-place','arm-tracking-probe':'probe','visual-preparation':'visual'}[cap.task?.task_name];
      if (task) { get('cap-task').value = task; get('cap-task').dataset.restored = '1'; }
    }
    const nodes = [];
    let nextIntervention=0;
    function appendInterventions(turn){
      while(nextIntervention<interventions.length && interventions[nextIntervention].before_turn<=turn){
        const entry=interventions[nextIntervention++], node=el('details','chat-card chat-input');
        node.append(el('summary','chat-summary','你 · 继续当前任务 / You · Continue current task'));
        const body=el('div','chat-body');body.append(el('p','chat-prose',entry.message));
        body.append(el('small','chat-meta',new Date(entry.time*1000).toLocaleString()));node.append(body);nodes.push(node);
      }
    }
    for (const row of rows) {
      appendInterventions(row.turn);
      nodes.push(card(row, 'input', cap, request), card(row, 'ai', cap, request));
      if (row.has_feedback) nodes.push(card(row, 'feedback', cap, request));
    }
    appendInterventions(Infinity);
    chat.replaceChildren(...(nodes.length ? nodes : [el('p', 'cap-empty', '尚无对话 / No conversation yet.')]));
    if (atBottom) chat.scrollTop = chat.scrollHeight;
    get('cap-collapse').onclick = () => chat.querySelectorAll('details').forEach(node => node.open = false);
  };
})();
