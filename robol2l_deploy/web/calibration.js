'use strict';
const el = id => document.getElementById(id);
const api = '/api/hardware/calibration/';
let stationId = '', session = {}, key = null, token = null, heartbeat = null;
let phase = 'idle', busy = false, preparing = false, polling = false;
const labels = {idle:'尚未示教 / Idle', armed:'等待使能键 / Waiting for key', pressed:'正在检查关节反馈，等显示重力补偿后再拖动 / Checking feedback, wait for gravity compensation',
  countdown:'控制面板正在倒计时摆位，请返回控制面板操作 / Positioning countdown; return to Controls',
  drag:'重力补偿中，继续扶臂 / Gravity compensation', locking:'正在锁位，继续扶臂 / Checking hold',
  held:'上次保持指令已完成，请现场确认 / Last hold commanded; confirm physically', cancelled:'示教已取消 / Cancelled',
  error:'示教异常，请继续扶臂 / Support the arm'};
function message(text) { el('notice').textContent = text; }
async function request(url, body) {
  const opts = body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json',
    'X-Console-Request':'1','X-Session-Id':stationId}, body:JSON.stringify(body)};
  const response = await fetch(url, opts);
  const value = await response.json();
  if (!response.ok) throw Error(value.error || response.statusText);
  return value;
}
function update() {
  const locked = key !== null || preparing || ['armed','pressed','countdown','drag','locking'].includes(phase);
  const count = side => (session.samples || []).filter(s => s.side === side).length;
  el('progress').textContent = '已记录 / Samples · 左 L ' + count('left') + '　右 R ' + count('right');
  el('side').disabled = busy || locked;
  // Do not disable a pressed HTML button; that can swallow its pointerup event.
  el('teach').disabled = !key && (busy || locked || !session.id || !el('support').checked);
  el('capture').disabled = busy || locked || !session.id || phase === 'error';
  el('solve').disabled = busy || locked;
  el('board').querySelector('button').disabled = busy || locked;
}
function report() {
  const box = el('report'), r = session.report;
  box.replaceChildren(); el('download').hidden = !r;
  if (!r) return;
  el('download').href = api + 'report?id=' + encodeURIComponent(session.id);
  const table = document.createElement('table');
  function row(values) {
    const tr = table.insertRow();
    for (const v of values) tr.insertCell().textContent = v;
  }
  row(['验算 / Validation','RMS mm','最大 / Max mm','RMS °','最大 / Max °','最大角点 RMS px']);
  const names = {left:'左腕 / Left', right:'右腕 / Right', global:'固定板重复性 / Global'};
  for (const [k,v] of Object.entries({...r.errors, leftCross:r.cross_view.left, rightCross:r.cross_view.right})) {
    row([names[k] || (k==='leftCross'?'左腕 ↔ 全局':'右腕 ↔ 全局'), ...['rms_mm','max_mm','rms_deg','max_deg','worst_detection_px'].map(f =>
      v[f] === undefined ? '—' : v[f].toFixed(2))]);
  }
  box.append(table);
  const matrices=document.createElement('details'), title=document.createElement('summary'), text=document.createElement('pre');
  title.textContent='查看变换矩阵 / View transforms (m)';
  text.textContent=Object.entries(r.transforms).map(([name,rows])=>
    name+'\n'+rows.map(row=>row.map(n=>n.toFixed(6)).join('  ')).join('\n')).join('\n\n');
  matrices.append(title,text);box.append(matrices);
  const p = document.createElement('p');
  p.textContent = '误差为留出姿态上的板位姿一致性。绝对抓取精度需独立参考物验收。 / Held-out consistency; validate absolute accuracy independently.';
  box.append(p);
}
function showSession(value) {
  const changed=session.report?.session!==value.report?.session || session.report?.sample_ids?.length!==value.report?.sample_ids?.length;
  if(session.id !== value.id)el('capture-result').textContent='';
  session=value;
  el('batch').textContent = session.id ? '当前批次 / Session '+session.id.slice(0,8)+' · '+
    session.board.cols+' × '+session.board.rows+' · '+session.board.square_mm+' / '+session.board.marker_mm+' mm · '+session.board.dictionary : '';
  if(changed)report(); update();
}
// A completed JPEG request releases its HTTP connection between frames.
async function previewCamera(role) {
  const image = el(role);
  const status = el(role + '-state');
  let source;
  for (;;) {
    if (document.hidden) {
      await new Promise(resolve => document.addEventListener('visibilitychange', resolve, {once:true}));
      continue;
    }
    try {
      if (!source) {
        const inventory = await request('/api/hardware/previews');
        source = inventory.roles[role];
        if (!source) throw Error('未绑定 / Unassigned');
      }
      const params = new URLSearchParams({role, source, t:Date.now()});
      await new Promise((resolve,reject) => {
        image.onload = resolve;
        image.onerror = () => reject(Error('未收到画面 / No frame'));
        image.src = '/api/hardware/previews/frame?' + params;
      });
      image.hidden = false;
      status.textContent = '实时 / Live';
    } catch (error) {
      image.hidden = true;
      status.textContent = error.message + ' · 正在重试 / Retrying';
      source = null;
    }
    await new Promise(resolve => setTimeout(resolve, source ? 150 : 2000));
  }
}
async function action(fn) {
  if (busy) return;
  busy=true; update(); message('');
  try { await fn(); } catch(e) { message(e.message); } finally { busy=false; update(); }
}
el('board').onsubmit = e => {
  e.preventDefault();
  if (session.id && !confirm('保存旧批次并新建？ / Archive current batch and start another?')) return;
  const board=Object.fromEntries(new FormData(e.target));
  for (const field of ['cols','rows','square_mm','marker_mm']) board[field]=Number(board[field]);
  board.legacy=board.legacy==='on';
  action(async()=>showSession(await request(api+'start',{board})));
};
el('capture').onclick=()=>action(async()=>{
  const side=el('side').value, label=side==='left'?'左臂 / Left':'右臂 / Right';
  const result=el('capture-result');
  result.textContent=label+' · 采集中 / Capturing…';
  try {
    const saved=await request(api+'capture',{id:session.id,side});
    showSession(saved);
    const count=saved.samples.filter(sample=>sample.side===side).length;
    result.textContent=label+' · 第 '+count+' 个样本已保存 / Sample '+count+' saved';
  } catch(error) {
    result.textContent=label+' · 采集未确认 / Capture unconfirmed · '+error.message;
  }
});
el('ack-parked').onclick=()=>action(async()=>{
  if (!confirm('两臂均已稳妥支撑？将读取编码器确认本次示教故障。恢复后由你手动开始示教。 / Both arms supported? Verify feedback and acknowledge this teaching fault.')) return;
  await request(api+'acknowledge-parked',{id:session.id,side:el('ack-parked').dataset.side,confirmed:true});
  el('support').checked=false;
  message('已确认本次软件故障。下次示教前请重新确认支撑。 / Software fault acknowledged.');
});
el('solve').onclick=()=>action(async()=>showSession(await request(api+'solve',{id:session.id})));
el('recheck-hold').onclick=()=>action(async()=>{
  const side=el('recheck-hold').dataset.side;
  const label=side==='left'?'左臂 / Left arm':'右臂 / Right arm';
  if (!confirm(label+' · 请稳妥支撑两臂。确认后将向故障臂发送一次当前位置锁位命令。 / Support both arms. Send one hold command to this arm at its measured position?')) return;
  el('support').checked=false;
  await request(api+'recheck-hold',{id:session.id,side,confirmed:true});
  await refresh();
});
el('side').onchange=()=>{el('support').checked=false;update();};
el('support').onchange=()=>{if (!el('support').checked) release(); update();};
async function release() {
  key=null; clearTimeout(heartbeat);
  const previous=token; token=null;
  if (previous) {
    try {await request(api+'release',{token:previous});} catch(e) {message(e.message);}
  }
  update();
}
async function press() {
  if (key || preparing || el('teach').disabled) return;
  const active={}; key=active; preparing=true; update();
  try {
    const prepared=await request(api+'prepare',{id:session.id,side:el('side').value,confirmed:true});
    if (key!==active) {await request(api+'release',prepared);return;}
    token=prepared.token;
    async function pulse() {
      if (key!==active) return;
      try {
        await request(api+'heartbeat',prepared);
        if (key===active) heartbeat=setTimeout(pulse,140);
      } catch(e) {if (key===active) {message(e.message); await release();}}
    }
    await pulse();
  } catch(e) {message(e.message);await release();}
  finally {preparing=false;update();}
}
const teach=el('teach');
teach.onpointerdown=e=>{if(e.button!==0)return;teach.setPointerCapture(e.pointerId);press();};
teach.onpointerup=teach.onpointercancel=teach.onlostpointercapture=()=>{if(key) release();};
teach.onkeydown=e=>{if(e.code==='Space'){e.preventDefault();if(!e.repeat)press();}};
window.addEventListener('keyup',e=>{if(e.code==='Space' && key)release();});
for(const event of ['blur','pagehide']) window.addEventListener(event,release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
async function refresh() {
  if(polling)return;
  polling=true;
  try {
    const s=await request(api+'status');
    phase=s.teach.phase;
    el('ack-parked').hidden=!s.teach.can_acknowledge;
    el('ack-parked').dataset.side=s.teach.side || '';
    el('recheck-hold').hidden=!s.teach.can_recheck_hold;
    el('recheck-hold').dataset.side=s.teach.side || '';
    const side=s.teach.side==='left'?'左 L · ':s.teach.side==='right'?'右 R · ':'';
    const details=s.teach.error_details || {};
    const error=[s.teach.error?.includes('arx_fault_or_offline')
      ? 'SDK 报告保护或关节离线，请托住手臂并松键 / SDK fault or offline joints. Support arm and release key.'
      : s.teach.error, details.fault, details.offline_joints?.length
        ? '离线关节 / Offline joints: '+details.offline_joints.join(', ') : ''].filter(Boolean).join(' · ');
    el('mode').textContent=side+(labels[phase]||phase)+(error?' · '+error:'');
    if(!busy) showSession(s.session);
    return s;
  } catch(e) {phase='error'; el('mode').textContent='状态未知，继续扶臂 / Status unavailable';message(e.message);await release();}
  finally {polling=false;}
}
for (const role of ['global','left_wrist','right_wrist']) previewCamera(role);
(async()=>{
  try {
    stationId=(await request('/api/hardware/session')).session_id || '';
    const s=await refresh(); if(!s)return;
    const dictionary=el('board').elements.dictionary;
    for(const name of s.dictionaries)dictionary.add(new Option(name,name));
    dictionary.value=dictionary.dataset.default;
    setInterval(refresh,500);
  } catch(e){message(e.message);}
})();
