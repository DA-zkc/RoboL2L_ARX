/* Shared Controls / Experiments entry. Countdown and drag live in Runtime,
   independent of pointer, tab focus, browser timers and calibration batches. */
(() => {
  const panels = [...document.querySelectorAll('[data-manual-positioning]')];
  const api = '/api/hardware/manual-positioning/';
  let sessionId = '', pending = false, polling = false, state = null, message = '', statusError = '';
  const phases = {
    idle: '尚未开始 / Ready', armed: '正在准备 / Preparing',
    countdown: '请去持稳所选机械臂 / Go support the selected arm',
    drag: '可以手动摆位；完成后点击锁定当前位置 / Guide the arm; click Hold when finished',
    locking: '正在锁位并验证反馈，请继续持稳 / Verifying hold; keep supporting the arm',
    held: '已请求当前位置保持，请现场确认 / Hold commanded; confirm physically',
    cancelled: '倒计时已取消，未进入拖动 / Cancelled before drag',
    error: '操作未完成，请查看反馈 / Operation incomplete; see feedback'
  };
  const reasons = {
    experiment_busy: '实验正在执行，请先暂停并等待当前步骤结束 / Pause the experiment and wait for its current step',
    cap_session_busy: 'CaP 正在执行，请先暂停并等待 / Pause CaP and wait',
    station_faulted: '存在未解除的故障；示教锁位故障可在下方复验 / Fault pending; teaching hold faults can be rechecked below',
    station_paused: '控制台已暂停，请先恢复派发 / Console paused; resume dispatch first',
    station_busy: '设备正在执行其他操作 / Station busy',
    manual_positioning_requires_owner_maintenance_update: '手动摆位更新尚未加载到硬件 owner / Hardware owner update required'
  };
  async function request(url, body) {
    const response = await fetch(url, body === undefined ? {} : {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-Console-Request': '1', 'X-Session-Id': sessionId},
      body: JSON.stringify(body)
    });
    const value = await response.json();
    if (!response.ok) throw Error(reasons[value.error] || value.message_zh || value.error || response.statusText);
    return value;
  }
  function render() {
    const teach = state?.teach || {}, active = !!teach.active;
    for (const panel of panels) {
      panel.dataset.positioningActive = String(active && !statusError);
      panel.dataset.positioningRecovery = String(!statusError && !!(teach.can_recheck_hold || teach.can_acknowledge));
      const select = panel.querySelector('select');
      if (active && teach.side) select.value = teach.side;
      select.disabled = pending || active;
      panel.querySelector('[data-start]').disabled = pending || !state || !!statusError || active || !!state.blocked_reason;
      const hold = panel.querySelector('[data-hold]');
      hold.disabled = pending || !active || teach.phase === 'locking';
      hold.textContent = ['armed', 'countdown'].includes(teach.phase)
        ? '取消倒计时 / Cancel countdown' : '锁定当前位置 / Hold current position';
      const side = teach.side === 'left' ? '左臂 / Left · ' : teach.side === 'right' ? '右臂 / Right · ' : '';
      const countdown = teach.phase === 'countdown' ? `${Math.ceil(teach.countdown_s || 0)} s · ` : '';
      panel.querySelector('[data-state]').textContent = side + countdown + (phases[teach.phase] || '正在读取 / Loading');
      panel.querySelector('[data-feedback]').textContent = [message, statusError, teach.error,
        !active && (reasons[state?.blocked_reason] || state?.blocked_reason)].filter(Boolean).join(' · ');
      for (const action of ['recheck-hold', 'acknowledge-parked']) {
        const button = panel.querySelector(`[data-recover="${action}"]`);
        button.hidden = !(action === 'recheck-hold' ? teach.can_recheck_hold : teach.can_acknowledge);
        button.disabled = pending || !!statusError || active || ['experiment_busy', 'cap_session_busy', 'station_paused'].includes(state?.blocked_reason);
      }
    }
  }
  async function refresh() {
    if (polling) return;
    polling = true;
    try { state = await request(api + 'status'); statusError = ''; }
    catch (error) { statusError = '状态读取失败，请继续持稳；关闭页面不会结束拖动 / Status unavailable; keep supporting. Closing the page does not end guidance. ' + error.message; }
    finally { polling = false; render(); }
  }
  async function act(action, body) {
    if (pending) return;
    pending = true; message = ''; render();
    try { await request(api + action, body); }
    catch (error) { message = error.message; }
    finally { pending = false; await refresh(); render(); }
  }
  for (const panel of panels) {
    panel.innerHTML = `<h3>手动摆位与锁位恢复 <small>Manual positioning & hold recovery</small></h3>
      <p>点击后 10 秒进入重力补偿拖动，利用倒计时去持稳机械臂。无需按住鼠标，无自动结束时限。摆好后点击“锁定当前位置”。<br>Gravity-compensated guidance starts after 10 seconds. Support the arm during the countdown. No held key or time limit; click Hold when finished.</p>
      <div class="actions"><label>机械臂 / Arm <select><option value="left">左臂 / Left</option><option value="right">右臂 / Right</option></select></label>
      <button type="button" class="primary" data-start disabled>10 秒后手动摆位 / Guide after 10 s</button>
      <button type="button" data-hold disabled>锁定当前位置 / Hold current position</button></div>
      <p data-state role="status" aria-live="polite"></p><p data-feedback role="status"></p>
      <div class="actions"><button type="button" data-recover="recheck-hold" hidden>复验当前位置锁位 / Recheck hold</button>
      <button type="button" data-recover="acknowledge-parked" hidden>确认已支撑并复查 / Review supported arms</button></div>
      <p>切换页面不结束拖动；锁位后程序运动保持关闭。 / Changing pages keeps guidance active; program motion stays disabled after hold.</p>`;
    panel.querySelector('[data-start]').onclick = () => act('start', {side: panel.querySelector('select').value});
    panel.querySelector('[data-hold]').onclick = () => act('hold', {});
    for (const button of panel.querySelectorAll('[data-recover]')) button.onclick = () => {
      if (!state?.teach.side) return;
      if (!confirm('请确认当前两臂已稳妥支撑。复验只针对本次示教故障，不会自动开始拖动。 / Confirm both arms are supported now. Recheck only this teaching fault; no automatic drag.')) return;
      act(button.dataset.recover, {side: state.teach.side, confirmed: true});
    };
  }
  (async () => {
    try {
      const session = await request('/api/hardware/session');
      if (session.backend !== 'arx_x5_2023_dual') return;
      sessionId = session.session_id || '';
      for (const panel of panels) panel.hidden = false;
      await refresh();
      setInterval(refresh, 500);
    } catch (error) { message = error.message; render(); }
  })();
})();
