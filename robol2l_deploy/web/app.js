'use strict';
const $ = id => document.getElementById(id);
let state = null, catalogue = null, robotState = null, observationId = null, planHandle = null;
let launcherURL = 'http://127.0.0.1:8770', currentBackend = null, firstRefresh = true;
const handles = {}, seen = new Set();
let calling = false, polling = false, ending = false, sessionId = null;
let modelServices = null, modelChoiceRestored = false;
let capState = null;
async function request(path, body) {
  const response = await fetch('/api/hardware/'+path, body === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json','X-Console-Request':'1','X-Session-Id':sessionId||''}, body:JSON.stringify(body)});
  const value = await response.json();
  if (!response.ok) throw Error(value.message_zh || value.error || response.status);
  return value;
}
function errorText(text) {
  const rules=[
    ['Connection to FCI refused','请在 Franka Desk 开启 FCI，再点重新连接。 / Enable FCI in Desk, then reconnect.'],
    ['session_changed','连接已切换，请重新打开面板。 / Connection changed; reopen the panel.'],
    ['Failed to fetch','连接已断开。点击上方连接入口重新进入。 / Disconnected; use the Connection link above.'],
    ['station_busy','正在处理上一项操作，请稍候。 / An operation is still running.'],
    ['invalid_api_key_env','环境变量名只能包含英文字母、数字和下划线，且不能以数字开头。 / Use letters, digits and underscores; do not start with a digit.'],
    ['station_faulted_or_paused','当前已暂停或出现控制故障，请查看状态。 / Paused or controller fault; check status.'],
    ['gripper_sweep_collision','夹爪开合规划预测模型相交；本次未发送动作。 / Predicted geometry intersection; no command sent.'],
    ['planning_collision_predicted','路径规划预测模型相交；本次未发送动作，具体部件与模型距离见反馈。 / Predicted geometry intersection; see planning evidence.'],
    ['Collision/clearance violation at path fraction','路径规划预测模型相交；本次未发送动作。 / Predicted geometry intersection; no command sent.'],
    ['library_curve_collision','轨迹规划预测模型相交；本次未发送动作。 / Predicted trajectory intersection; no command sent.'],
    ['gripper_target_not_reached','夹爪未到达目标宽度；目标、实测值和偏差见详情。 / Gripper target not reached; see measurements.'],
    ['Provider unavailable','感知服务未连接，请查看服务状态。 / Perception service unavailable.'],
    ['IK did not reach','IK 未找到可达的关节姿态，请调整目标。 / IK could not reach the requested pose.'],
    ['tuple index out of range','没有这个编号的识别结果，请先识别目标。 / No such candidate; detect an object first.'],
    ['realtime_prerequisites_missing','实时控制环境未就绪。 / Realtime control is unavailable.']
  ];
  for(const [key,label] of rules) if(String(text).includes(key))return label;
  return /[\u4e00-\u9fff]/.test(String(text))?text:'操作未完成 / Operation failed: '+text;
}
function notice(text) { text=text?errorText(text):''; $('notice').textContent=text; $('notice').hidden=!text; }
function show(value, label='结果 / Result') {
  $('output').textContent = typeof value === 'string' ? value : JSON.stringify(value,null,2);
  $('result-kind').textContent = label;
}
function act(fn) { return async () => { try { notice(''); await fn(); } catch (error) { notice(error.message); } }; }
// The operator's per-arm controls use the same typed planning and execution APIs as CaP.
const armLabels = {left:['左臂','Left arm'], right:['右臂','Right arm']};
const armCards = new Map();
let lastArmReceipt = null;
function clearArmPlans() {
  for (const item of armCards.values()) {
    item.handle=null; item.execute.disabled=true; item.info.textContent='';
  }
}
function makeArmCards(ports) {
  for (const [arm,[zh,en]] of Object.entries(armLabels)) {
    const card=document.createElement('article');
    card.className='card arm-card '+arm;
    card.innerHTML=`<header><h2>${zh} <small>${en}</small></h2><code></code></header>
      <dl class="joint-strip"></dl><div class="arm-pose"></div>
      <div class="arm-action"><label>关节 / Joint<select></select></label>
      <label>相对转角 / Δ°<input type="number" step="any" value="3" required></label>
      <button type="button" class="arm-plan">预览 / Plan</button>
      <button type="button" class="arm-execute primary" disabled>执行${zh} / Move ${arm}</button></div>
      <p class="arm-result" role="status"></p>`;
    card.querySelector('code').textContent=ports[arm];
    const select=card.querySelector('select'), strip=card.querySelector('dl');
    const values=[];
    for (let j=1;j<=6;j++) {
      select.add(new Option('J'+j,j-1));
      const pair=document.createElement('div'), name=document.createElement('dt'), val=document.createElement('dd');
      name.textContent='J'+j; val.textContent='—'; pair.append(name,val);strip.append(pair);values.push(val);
    }
    select.value='5';
    const input=card.querySelector('input'), plan=card.querySelector('.arm-plan'), execute=card.querySelector('.arm-execute');
    const info=card.querySelector('.arm-result');
    const entry={values, pose:card.querySelector('.arm-pose'), handle:null, plan, execute, input, select, info};
    armCards.set(arm,entry);
    function invalidate() { entry.handle=null; execute.disabled=true; info.textContent=''; }
    input.oninput=select.onchange=invalidate;
    plan.onclick=act(async()=>{
      invalidate();
      const amount=input.valueAsNumber, j=Number(select.value);
      if (!Number.isFinite(amount)||amount===0) throw Error('请输入非零转角 / Enter a nonzero angle');
      const current=await call('get_robot_state');
      const goal=current.value.joint_positions[arm].slice();
      const from=goal[j]*180/Math.PI;
      goal[j]+=amount*Math.PI/180;
      const reply=await call('plan_motion',{arm,goal});
      if(input.valueAsNumber!==amount||Number(select.value)!==j)return;
      entry.handle=reply.handle;
      info.textContent=`${zh} J${j+1} · ${from.toFixed(2)}° → ${(from+amount).toFixed(2)}° · ${reply.value.trajectory.dt_s.toFixed(2)} s`;
      updateControls();
    });
    execute.onclick=act(async()=>{
      const handle=entry.handle;
      if (!handle) return;
      clearArmPlans();
      await call('execute_trajectory',{trajectory:{$ref:handle,path:['trajectory']}});
      info.textContent=zh+'已到位 / '+en+' reached target';
      await call('get_robot_state');
    });
    $('arx-arms').append(card);
  }
}

const previewNames={global:['全局相机','Global'], left_wrist:['左腕相机','Left wrist'], right_wrist:['右腕相机','Right wrist']};
const previewTiles=new Map();
let previewActive=false, previewGeneration=0, inventoryTimer=null, previewsPaused=false;

function makePreviewTiles() {
  for(const [role,[zh,en]] of Object.entries(previewNames)){
    const card=document.createElement('article');
    card.className='preview-tile';
    card.innerHTML=`<header><h3>${zh} <small>${en}</small></h3><span class="feed-state"></span></header>
      <div class="viewfinder"><img hidden alt="${zh} / ${en}"><span class="feed-message"></span></div>
      <label>视频源 / Source<select aria-label="${zh}视频源 / ${en} source"></select></label>`;
    const tile={role, select:card.querySelector('select'), img:card.querySelector('img'),
      status:card.querySelector('.feed-state'), message:card.querySelector('.feed-message'),
      timer:null, sequence:0, source:null, options:null, available:false};
    tile.select.onchange=act(async()=>{
      tile.select.disabled=true;
      try {
        const settings=await request('previews/bind',{role,source:tile.select.value||null});
        if(previewActive) applyPreviewSettings(settings);
      } catch(error) {tile.select.value=tile.source||'';throw error;}
      finally { tile.select.disabled=false; }
    });
    previewTiles.set(role,tile);$('preview-grid').append(card);
    previewStatus(tile,'等待设备 / Waiting', '正在读取视频源 / Checking sources');
  }
}
function previewStatus(tile,status,message){
  tile.status.textContent=status; tile.status.dataset.live=String(!message);
  tile.message.textContent=message;tile.message.hidden=!message;tile.img.hidden=!!message;
}
function stopTile(tile){
  tile.sequence++;clearTimeout(tile.timer);
  tile.img.onload=tile.img.onerror=null;
  tile.img.removeAttribute('src');
  tile.source=null;tile.available=false;
}
function startTile(tile){
  const sequence=++tile.sequence;
  function next(){
    if(!previewActive||sequence!==tile.sequence)return;
    tile.img.src='/api/hardware/previews/frame?'+new URLSearchParams({role:tile.role,source:tile.source,t:Date.now()});
  }
  tile.img.onload=()=>{
    if(sequence!==tile.sequence)return;
    previewStatus(tile,'实时 / Live','');tile.timer=setTimeout(next,120);
  };
  tile.img.onerror=()=>{
    if(sequence!==tile.sequence)return;
    previewStatus(tile,'无画面 / No signal','暂未收到视频帧 / Waiting for camera');
    tile.timer=setTimeout(next,2000);
  };
  previewStatus(tile,'连接中 / Connecting','正在获取画面 / Opening stream');
  next();
}
function applyPreviewSettings({roles,devices}){
  for(const tile of previewTiles.values()){
    const source=roles[tile.role]||'', available=Object.hasOwn(devices,source);
    const signature=JSON.stringify([roles,devices]);
    if(tile.options!==signature){
      tile.options=signature;
      const choices=[new Option('未绑定 / Unassigned','')];
      for(const [key,label] of Object.entries(devices)){
        const option=new Option(label,key);
        option.disabled=Object.entries(roles).some(([role,id])=>role!==tile.role&&id===key);
        choices.push(option);
      }
      if(source&&!available)choices.push(new Option(source+' · 离线 / Offline',source));
      tile.select.replaceChildren(...choices);tile.select.value=source;
    }
    if(tile.source===source && tile.available===available)continue;
    stopTile(tile);tile.source=source;tile.available=available;
    if(available)startTile(tile);
    else if(source)previewStatus(tile,'离线 / Offline','已绑定的相机未连接 / Camera disconnected');
    else previewStatus(tile,'未绑定 / Unassigned','连接相机后选择视频源 / Connect and select a source');
  }
}
function syncPreviewWall(){
  const visible=previewTiles.size>0 && !document.hidden && !$('observe-panel').hidden && !previewsPaused;
  if(visible===previewActive)return;
  previewActive=visible;
  const generation=++previewGeneration;
  clearTimeout(inventoryTimer);
  for(const tile of previewTiles.values()) {stopTile(tile);previewStatus(tile,'暂停 / Paused','预览已暂停 / Preview paused');}
  if(!visible)return;
  async function poll(){
    try {
      const settings=await request('previews');
      if(generation===previewGeneration)applyPreviewSettings(settings);
    } catch (_) {
      if(generation===previewGeneration)for(const tile of previewTiles.values()){
        stopTile(tile);previewStatus(tile,'服务未就绪 / Unavailable','请检查预览服务 / Check preview service');
      }
    } finally {
      if(generation===previewGeneration)inventoryTimer=setTimeout(poll,4000);
    }
  }
  poll();
}
document.addEventListener('visibilitychange',syncPreviewWall);
$('preview-toggle').onclick=()=>{
  previewsPaused=!previewsPaused;
  $('preview-toggle').textContent=previewsPaused?'恢复预览 / Resume':'暂停预览 / Pause video';
  syncPreviewWall();
};
function initDualPanel(c){
  $('single-controls').hidden=true;$('dual-controls').hidden=false;
  $('calibrated-views').hidden=true;$('arx-views').hidden=false;
  makeArmCards(c.can_ports);makePreviewTiles();syncPreviewWall();
  $('refresh-arms').onclick=act(()=>call('get_robot_state'));
}

function displayRobot(value) {
  if (!value?.end_effector_poses) return;
  robotState = value;
  const arms = Object.keys(value.joint_positions);
  if (arms.length > 1) {
    for(const [arm,card] of armCards){
      const angles=value.joint_positions[arm], pose=value.end_effector_poses[arm];
      card.values.forEach((node,i)=>node.textContent=(angles[i]*180/Math.PI).toFixed(1)+'°');
      card.pose.textContent=pose.frame+' · TCP (m)   '+pose.position.map(n=>n.toFixed(3)).join('   /   ');
    }
    if(value.timestamp_s)$('arm-reading').textContent='采样 / Sample '+new Date(value.timestamp_s*1000).toLocaleTimeString();
    $('robot-mode').textContent = value.embodiment;
    $('tcp-value').textContent = arms.map(a => a+' ('+value.end_effector_poses[a].frame+') '+value.end_effector_poses[a].position.map(x=>x.toFixed(4)).join(' · ')).join(' | ');
    $('gripper-value').textContent = arms.map(a => a+' '+value.gripper_positions[a].toFixed(3)).join(' | ');
    return;
  }
  const pose = value.end_effector_poses.primary;
  $('robot-mode').textContent = value.embodiment || 'primary';
  $('tcp-value').textContent = pose.position.map(x=>Number(x).toFixed(4)).join(' · ');
  const width = value.gripper_positions.primary;
  $('gripper-value').textContent = (Number(width)*80).toFixed(1)+' mm';
}
function acceptJob(job) {
  show(job.result || {status:job.status,error:job.error},job.operation);
  if(job.operation.startsWith('manual:')) {
    $('action-result').hidden=false;
    const result=job.result,diagnostics=result?.execution?.diagnostics;
    $('action-result').textContent=result?.warning_only ? '移动完成，偏差见下方，可以继续。 / Move completed; measured deviation below.' : result?.ok ? (job.operation==='manual:move'?'移动完成 / Move completed':`夹爪到位 / Gripper reached · 目标 ${diagnostics?.target_width_mm?.toFixed(1)} mm → 实测 ${diagnostics?.actual_width_mm?.toFixed(1)} mm`) : errorText(result?.error?.message_zh||result?.error?.message||result?.error||job.error||'操作未完成 / Operation did not complete');
    if(diagnostics?.actual_width_mm!=null)$('gripper-value').textContent=diagnostics.actual_width_mm.toFixed(1)+' mm';
    if(!job.result?.ok&&!job.result?.warning_only)notice($('action-result').textContent);
    planHandle=null;return;
  }
  const reply = job.result;
  if (!reply?.handle) return;
  const op = reply.operation, value = reply.value;
  handles[op] = reply.handle;
  if (['execute_trajectory','set_gripper','open_gripper','close_gripper'].includes(op)) planHandle=null;
  if (op === 'get_robot_state') displayRobot(value);
  if (op === 'get_observation') {
    if(job.id.startsWith('startup-connect')){displayRobot(value.robot_state);return;}
    displayRobot(value.robot_state); observationId = reply.observation_id;
    planHandle=null; delete handles.segment_text; delete handles.mask_to_point_cloud; delete handles.generate_grasps;
    $('observation-value').textContent = observationId.slice(0,10);
    for (const img of $('cameras').querySelectorAll('img')) {
      img.src = '/api/hardware/image?observation='+encodeURIComponent(observationId)+'&camera='+encodeURIComponent(img.dataset.camera)+'&depth='+img.dataset.depth;
      img.dataset.frozen = observationId;
    }
    for (const label of $('cameras').querySelectorAll('.camera-state')) label.textContent='冻结 / Frozen · '+observationId.slice(0,8);
  }
  if (op === 'plan_motion' || op === 'plan_synchronized_motion') {
    planHandle = value.ok ? reply.handle : null;
    drawPreview(value.diagnostics?.preview_xyz || []);
    drawPreview(value.diagnostics?.preview_xyz || [],'simple-preview');
    $('plan-summary').textContent = value.ok ? ('已登记 / Registered · '+(value.diagnostics?.plan_id||'').slice(0,12)+' · '+(value.trajectory?.planner||'')+(value.diagnostics?.speed_factor!=null?` · 速度 / Speed ${value.diagnostics.speed_factor} · ${Number(value.diagnostics.duration_s).toFixed(2)} s`:'')+(value.diagnostics?.collision_checked===false?' · 未做模型碰撞检查 / Model collision unchecked':'')) : ('规划失败 / Planning failed · '+(value.error?.message_zh||value.error?.message||''));
  }
  if (['segment_text','localize_object','generate_grasps','select_grasp','estimate_geometry','mask_to_point_cloud'].includes(op)) {
    if(op==='select_grasp'&&value===null){$('perception-output').textContent='没有符合筛选条件的抓取；可选择最高得分或更换观察视角。 / No matching grasp; choose highest score or another view.';return;}
    const detail=value?.ok===false?errorText(value.error?.message_zh||value.error?.message):op==='segment_text'?`识别到 ${value.segmentations?.length||0} 个目标 / objects`:op==='generate_grasps'?`生成 ${value.grasps?.length||0} 个抓取候选 / grasps`:op==='localize_object'?`已定位 / Localized · ${JSON.stringify(value.geometry?.center||value.geometry?.pose||value.geometry)}`:'已完成 / Completed';
    $('perception-output').textContent=detail;
  }
}
async function call(operation, arguments_={}) {
  if (calling) throw Error('正在处理上一项操作 / An operation is already running');
  calling=true;updateControls();
  try {
    const id=crypto.randomUUID();
    await request('operator/call',{request_id:id,operation,arguments:arguments_});
    while (true) {
      const snapshot=await request('state');state=snapshot;
      const job=snapshot.jobs[id];
      if (job && !['accepted','running'].includes(job.status)) {
        seen.add(id);acceptJob(job);await loadCatalogue(false);
        if (job.status!=='completed') throw Error(job.error||job.status);
        if (job.result?.value?.ok===false)throw Error(job.result.value.error?.message_zh||job.result.value.error?.message||'操作未完成');
        return job.result;
      }
      await new Promise(resolve=>setTimeout(resolve,250));
    }
  } finally {calling=false;await refresh();}
}
function reference(operation,path=[]) {
  if (!handles[operation]) throw Error('请先调用 / First call '+operation);
  return {$ref:handles[operation],...(path.length?{path}: {})};
}
function pose() {return {position:['px','py','pz'].map(id=>Number($(id).value)),quaternion_wxyz:['qw','qx','qy','qz'].map(id=>Number($(id).value)),frame:'base'};}
function fillPose(value,offset=[0,0,0]) {
  ['px','py','pz'].forEach((id,i)=>$(id).value=(value.position[i]+offset[i]).toFixed(6));
  ['qw','qx','qy','qz'].forEach((id,i)=>$(id).value=value.quaternion_wxyz[i].toFixed(7));
}
function cameraPanels(names) {
  $('cameras').replaceChildren();$('perception-camera').replaceChildren();
  for (const name of names) {
    const option=new Option(name,name);$('perception-camera').add(option);
    // The dual ARX preview wall already owns the visible camera views.
    if ($('calibrated-views').hidden) continue;
    const card=document.createElement('article');card.className='camera-card';
    const heading=document.createElement('div');heading.className='camera-heading';
    const title=document.createElement('h3');title.textContent=name+' · RGB-D';
    const live=document.createElement('button');live.className='quiet';live.textContent='实时 / Live';
    heading.append(title,live);card.append(heading);
    const grid=document.createElement('div');grid.className='camera-images';
    for (const depth of [0,1]) {
      const figure=document.createElement('figure'),img=document.createElement('img'),caption=document.createElement('figcaption');
      img.dataset.camera=name;img.dataset.depth=String(depth);img.alt=name+(depth?' depth':' RGB');
      const stream=()=>{delete img.dataset.frozen;img.src='/api/hardware/stream?camera='+encodeURIComponent(name)+'&depth='+depth+'&t='+Date.now();};
      stream();caption.textContent=depth?'DEPTH · 0–2 m':'';figure.append(img,caption);
      if(depth){const detail=document.createElement('details'),summary=document.createElement('summary');summary.textContent='查看深度 / Depth';detail.append(summary,figure);grid.append(detail);}else grid.append(figure);
      img.onerror=()=>{label.textContent='相机暂不可用，请点实时画面重试。 / Camera unavailable; retry Live video.';};
      if (!depth) img.addEventListener('click',async event=>{
        try {
          if (!img.dataset.frozen) throw Error('请先冻结 RGB-D / Freeze RGB-D first');
          const rect=img.getBoundingClientRect();
          const x=Math.min(img.naturalWidth-1,Math.max(0,Math.floor((event.clientX-rect.left)*img.naturalWidth/rect.width)));
          const y=Math.min(img.naturalHeight-1,Math.max(0,Math.floor((event.clientY-rect.top)*img.naturalHeight/rect.height)));
          const args={observation_id:img.dataset.frozen,camera_name:name};
          if (event.shiftKey) {const left=Math.max(0,x-5),top=Math.max(0,y-5);args.box=[left,top,Math.min(10,img.naturalWidth-left),Math.min(10,img.naturalHeight-top)];}
          else args.pixel=[x,y];
          const value=await request(event.shiftKey?'measure':'localize',args);
          $('measurement').textContent=JSON.stringify({pixel:[x,y],...value},null,2);show(value,'人工测量 / Manual measurement');
        } catch(error) {notice(error.message);}
      });
      live.addEventListener('click',()=>{stream();label.textContent='实时 / Live';});
    }
    const label=document.createElement('p');label.className='camera-state';label.textContent='实时 / Live';
    card.append(grid,label);$('cameras').append(card);
  }
  if (names.includes('global')) $('perception-camera').value='global';
}
function drawPreview(points, id='preview') {
  const area=$(id);area.replaceChildren();area.className=points.length?'':'preview-empty';
  if (!points.length) {area.textContent='尚无可用轨迹 / No available trajectory';return;}
  for (const [a,b,label] of [[0,1,'XY'],[0,2,'XZ'],[1,2,'YZ']]) {
    const canvas=document.createElement('canvas');canvas.width=320;canvas.height=220;
    const ctx=canvas.getContext('2d');ctx.fillStyle='#f7fafb';ctx.fillRect(0,0,320,220);
    ctx.font='12px system-ui';ctx.fillStyle='#617685';ctx.fillText(label+' · m',15,22);
    const xs=points.map(p=>p[a]),ys=points.map(p=>p[b]);
    const lowX=Math.min(...xs),lowY=Math.min(...ys),span=Math.max(Math.max(...xs)-lowX,Math.max(...ys)-lowY,.01);
    ctx.strokeStyle='#0b8f7f';ctx.lineWidth=2;ctx.beginPath();
    points.forEach((p,i)=>{const x=35+(p[a]-lowX)/span*155,y=185-(p[b]-lowY)/span*155;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
    ctx.fillText('start '+points[0][a].toFixed(4)+', '+points[0][b].toFixed(4),15,208);area.append(canvas);
  }
}
function template(name) {
  const ref=(op,path=[])=>({$ref:handles[op]||'先调用 '+op+' / call first',...(path.length?{path}: {})});
  const camera=$('perception-camera').value||'global';
  const templates={get_robot_state:{},get_task_context:{},get_observation:{},
    segment_text:{camera_name:camera,text:$('query').value}, segment_points:{camera_name:camera,points_px:{$array:[[80,60]]}},
    localize_object:{query:$('query').value,camera_name:camera,target_frame:state?.capabilities?.perception_frame || 'base'},
    mask_to_point_cloud:{mask:ref('segment_text',['segmentations',0]),camera_name:camera,target_frame:state?.capabilities?.perception_frame || 'base'},
    crop_point_cloud:{point_cloud:ref('mask_to_point_cloud'),lower_xyz:[-.5,-.5,0],upper_xyz:[.5,.5,1]},
    estimate_geometry:{point_cloud:ref('mask_to_point_cloud')},
    generate_grasps:{camera_name:camera,mask:ref('segment_text',['segmentations',0]),max_candidates:5},
    select_grasp:{grasps:ref('generate_grasps'),strategy:'top_down'},
    solve_ik:{target_pose:{$pose:pose()}},plan_motion:{goal:{$pose:pose()}},
    execute_trajectory:{trajectory:ref('plan_motion',['trajectory'])},
    set_gripper:{position:.5},set_grippers:{positions:{primary:.5}},open_gripper:{},close_gripper:{},go_home:{},
    move_to_pose:{target_pose:{$pose:pose()}},
    move_to_joints:{target:ref('get_robot_state',['joint_positions','primary'])},
    plan_synchronized_motion:{targets:{primary:{$pose:pose()}}},
    move_synchronized:{targets:{primary:{$pose:pose()}}},
    step:{action:{$type:'RobotAction',arms:{primary:{$type:'ArmCommand',mode:'joint_position',
      target:ref('get_robot_state',['joint_positions','primary']),embodiment:robotState?.embodiment||state?.capabilities?.embodiment||'franka_fr3'}}}}};
  if (state?.capabilities?.robot_backend === 'arx_x5_2023_dual') {
    const joints = side => ref('get_robot_state',['joint_positions',side]);
    const targets = {left:joints('left'),right:joints('right')};
    const embodiment=state.capabilities.embodiment;
    const side=state.capabilities.calibrated_grippers?.[0] || 'left';
    const localPose=ref('get_robot_state',['end_effector_poses','left']);
    Object.assign(templates, {go_home:{arm:'both'}, solve_ik:{target_pose:localPose,arm:'left'},
      move_to_pose:{target_pose:localPose,arm:'left'}, set_gripper:{arm:side,position:.5},
      set_grippers:{positions:{left:.5,right:.5}},open_gripper:{arm:side},close_gripper:{arm:side},
      step:{action:{$type:'RobotAction',arms:{left:{$type:'ArmCommand',embodiment,
        mode:'joint_position',target:joints('left')}}}}, move_to_joints:{target:joints('left'),arm:'left'},
      plan_motion:{goal:joints('left'),arm:'left'},plan_synchronized_motion:{targets},
      move_synchronized:{targets},execute_trajectory:{trajectory:{$ref:planHandle||'先规划 / Plan first',path:['trajectory']}}});
  }
  return templates[name]||{};
}
function selectTool() {
  const tool=catalogue?.tools.find(item=>item.name===$('api-tool').value);
  if (!tool) return;
  $('signature').textContent=tool.name+tool.signature;
  $('api-arguments').value=JSON.stringify(template(tool.name),null,2);
  $('api-reason').textContent=tool.supported?'参数使用 RoboL2L 公开类型 / Uses RoboL2L public types':'当前部署不支持此接口 / Unsupported by the current deployment';
  if(tool.name==='go_home' && state?.capabilities?.robot_backend==='arx_x5_2023_dual')
    $('api-reason').textContent='默认双臂返回工作站 Home；left/right 仅移动该侧。实验初始位姿见复位面板。 / Both arms by default; select left/right for one arm. Experiment reset uses its own snapshot.';
  updateControls();
}
async function loadCatalogue(initial) {
  catalogue=await request('operator');
  if (initial) {
    for (const tool of catalogue.tools) $('api-tool').add(new Option(tool.name+(tool.supported?'':' · unsupported'),tool.name));
    $('api-tool').value='get_robot_state';selectTool();
    $('api-count').textContent=catalogue.tools.filter(x=>x.supported).length+' 个可用工具 / available tools';
    $('catalogue').replaceChildren();
    for (const tool of catalogue.tools) {
      const item=document.createElement('div');item.className='tool-card'+(tool.supported?'':' unavailable');
      const name=document.createElement('strong');name.textContent=tool.name;
      const label=document.createElement('span');label.textContent=tool.supported?(tool.motion?'运动 / Motion':'读取或计算 / Read or compute'):'未支持 / Unsupported';
      item.append(name,label);$('catalogue').append(item);
    }
  }
  $('references').replaceChildren();
  for (const ref of catalogue.references.slice().reverse()) {
    const div=document.createElement('div');div.className='reference';
    const title=document.createElement('span');title.textContent=ref.operation+' → '+ref.type;
    const id=document.createElement('code');id.textContent=ref.id;div.append(title,id);$('references').append(div);
  }
}
function updateControls() {
  if (!state) return;
  updateHandControls();
  const c=state.capabilities,busy=calling||handBusy||c.busy||!!catalogue?.cap_running;
  for(const card of armCards.values()){
    card.plan.disabled=busy;card.input.disabled=busy;card.select.disabled=busy;
    card.execute.disabled=!card.handle||busy||c.faulted||c.paused;
  }
  $('refresh-arms').disabled=busy;
  const canMove=!c.paused&&!c.faulted&&!busy;
  const blocked=busy||c.paused||c.faulted;
  $('execute').disabled=!planHandle||blocked;
  for(const id of ['move-offset','manual-gripper'])$(id).disabled=blocked;
  document.querySelectorAll('[data-manual-width]').forEach(button=>button.disabled=blocked);
  $('preview-offset').disabled=busy;$('disconnect-top').disabled=busy;
  $('pause').hidden=c.paused;$('resume').hidden=!c.paused;
  document.querySelectorAll('[data-motion]').forEach(button=>button.disabled=!canMove);
  for (const id of ['connect','state','observe','use-current','offset','ik','plan','segment','localize','project','geometry','grasps','select-grasp','perception-freeze','perceive-all','arm']) $(id).disabled=busy;
  const tool=catalogue?.tools.find(item=>item.name===$('api-tool').value);
  $('api-call').disabled=!tool?.supported||busy||(tool.motion&&!canMove);
  if (c.robot_backend === 'arx_x5_2023_dual') {
    for (const id of ['move-offset','preview-offset','use-current','offset','ik','plan','execute','manual-gripper','gripper','open-gripper','close-gripper','grasps','select-grasp','perceive-all']) $(id).disabled=true;
    if (!c.camera_extrinsics_calibrated) for (const id of ['segment','localize','project','geometry','perception-freeze']) $(id).disabled=true;
    document.querySelectorAll('[data-manual-width]').forEach(button=>button.disabled=true);
  }
  $('end-session').disabled=busy;
  const task=$('cap-task').value,model=$('cap-model').value,physical=['probe','pick-place','joint'].includes(task);
  $('joint-goal-label').hidden=task!=='joint';
  $('cap-start').disabled=(task==='joint' && (c.robot_backend!=='arx_x5_2023_dual'||!$('joint-goal').value.trim())) ||
    (c.robot_backend==='arx_x5_2023_dual' && task!=='joint') || busy||c.paused||c.faulted||
    (task==='mock'&&(!c.synthetic||!c.motion_enabled))||
    ((task==='visual'||physical)&&(c.synthetic||model==='fixture'));
  $('cap-continuation').hidden=!capState?.session_id||!['waiting_operator','paused','interrupted','error'].includes(capState.status);
  $('cap-continue').disabled=busy||!capState?.presentation?.can_continue||(!c.synthetic&&model==='fixture');
  $('cap-authorization').disabled=!physical;$('cap-distance').disabled=task!=='probe';
}
async function refresh() {
  if (polling||ending) return;
  polling=true;
  try {
    const [snapshot,cap]=await Promise.all([request('state'),request('cap/state')]);state=snapshot;capState=cap;
    if(sessionId&&state.session_id&&sessionId!==state.session_id){ending=true;window.location.assign(launcherURL);return;}
    const c=state.capabilities;
    if (catalogue) catalogue.cap_running=['ready','running'].includes(cap.status);
    const stationName=c.robot_backend?.startsWith('arx_')?'ARX X5':'FR3';
    $('backend').textContent=c.synthetic?'软件模拟 / Simulation':c.robot_backend+' · '+stationName;
    $('backend').className='badge'+(c.synthetic?' amber':'');
    const speed=c.motion_speed;$('motion-speed').hidden=!speed;
    if(speed){
      $('motion-speed').textContent=`速度默认 ${speed.default_speed_factor} · ${speed.max_speed_factor == null ? "SDK 规划 / SDK timing" : "最大系数 / Maximum factor " + speed.max_speed_factor}`;
      $('motion-speed').title='CaP 可逐次规划调整速度系数，实际值记录在规划反馈中。 / CaP may select speed per plan; accepted speed is recorded in planning feedback.';
    }
    $('simulation-banner').hidden=!c.synthetic;
    const connection=state.connection;
    let status=c.synthetic?'模拟模式 · 没有连接硬件 / Simulation · No hardware':stationName+' · '+(connection?.robot_state?'已读取状态 / State received':'等待读取 / Awaiting state');
    if(connection?.error)status=errorText(connection.error);
    if(c.busy)status+=' · 正在处理 / Working…';
    if(c.paused)status+=' · 已暂停 / Paused';
    if(c.faulted)status+=' · 控制故障 / Controller fault';
    $('status').textContent=status;
    if(currentBackend===null && c.robot_backend==='arx_x5_2023_dual')initDualPanel(c);
    const receipt=state.last_execution?.job_id || null;
    if(armCards.size && receipt!==lastArmReceipt) {
      clearArmPlans(); lastArmReceipt=receipt;
    }
    if(connection?.robot_state)displayRobot(connection.robot_state);
    $('policy').textContent=c.physical_session?JSON.stringify(c.physical_session,null,2):'';
    renderCapConversation(cap,request);
    if (currentBackend===null) {currentBackend=c.robot_backend||'mock';cameraPanels(c.cameras);}
    if (firstRefresh) {Object.keys(state.jobs).filter(id=>!id.startsWith('startup-connect')||!Object.keys(state.jobs).at(-1).startsWith('startup-connect')).forEach(id=>seen.add(id));firstRefresh=false;}
    for (const job of Object.values(state.jobs)) {
      if (seen.has(job.id)||['accepted','running'].includes(job.status)) continue;
      seen.add(job.id);acceptJob(job);
    }
    renderEndpoint(state.last_execution);renderJobs();updateControls();
  } finally {polling=false;}
}
function renderEndpoint(feedback) {
  const value=feedback?.accuracy;
  const perArm=feedback?.diagnostics?.per_arm;
  if(perArm){
    $('endpoint-feedback').hidden=false;
    const checks=feedback.diagnostics.goal_checks;
    $('endpoint-title').textContent=feedback.result_ok?'双臂执行回执 / Dual-arm receipt':
      checks?'未达到策略精度 / Goal tolerance missed':'双臂执行停止 / Dual-arm execution stopped';
    $('endpoint-summary').textContent=Object.entries(perArm).map(([arm,r])=>arm+' · '+r.dispatch+
      (r.max_error_rad==null?'':' · '+(r.max_error_rad*180/Math.PI).toFixed(2)+'°')).join(' | ');
    $('endpoint-delta').textContent='';
    $('endpoint-next').textContent=checks && !feedback.result_ok && feedback.can_continue?
      '可从实测位置重新规划。 / Replan from the measured positions.':feedback.error_zh||feedback.error||'';
    $('endpoint-joints').textContent=JSON.stringify(checks?{per_arm:perArm,goal_checks:checks}:perArm,null,2);
    return;
  }
  $('endpoint-feedback').hidden=!value&&!feedback?.error;
  if(!value){
    if(feedback?.error){
      const retryable=feedback.can_continue&&!state.capabilities.faulted;
      $('endpoint-title').textContent=retryable?'本次动作未开始 / Motion not started':'动作已停止 / Motion stopped';
      $('endpoint-summary').textContent=feedback.error_zh||feedback.error;
      $('endpoint-delta').textContent='';
      $('endpoint-next').textContent=retryable?'可以重新读取状态、重新规划后继续。 / Read the current state and replan to continue.':'没有自动恢复或继续派发。 / No automatic recovery or further dispatch.';
      $('endpoint-joints').textContent=JSON.stringify(feedback.diagnostics||{},null,2);
    }
    return;
  }
  const n=(x,d=3)=>Number(x).toFixed(d);
  $('endpoint-title').textContent=feedback.warning_only?'动作已结束 · 精度提示 / Completed · Accuracy warning':'终点偏差 / Endpoint deviation';
  $('endpoint-summary').textContent=`位置 ${n(value.position_error_mm)} mm · 姿态 ${n(value.orientation_error_deg)}° · 最大关节偏差 ${n(value.max_joint_error_deg)}° (${n(value.max_joint_error_rad,6)} rad)`;
  $('endpoint-delta').textContent='实际 − 目标 / Actual − target：'+['X','Y','Z'].map((axis,i)=>axis+' '+(value.position_delta_mm[i]>=0?'+':'')+n(value.position_delta_mm[i])+' mm').join(' · ');
  $('endpoint-next').textContent=feedback.can_continue&&!state.capabilities.faulted?'可以直接进行下一步；下一次从实际位置重新规划。 / You can continue; the next move uses the measured position.':'当前操作已停止，请查看控制器状态。 / Stopped; check controller status.';
  $('endpoint-joints').textContent=value.joint_delta_rad.map((v,i)=>`J${i+1}: ${n(v,6)} rad (${n(v*180/Math.PI)}°)`).join('\n')+'\n偏差仅作反馈，无项目精度门槛 / Measured deviations; no project accuracy gate';
}
function renderJobs() {
  $('job-list').replaceChildren();
  for (const job of Object.values(state.jobs).slice(-25).reverse()) {
    const row=document.createElement('div');row.className='job-row';
    const text=document.createElement('div'),name=document.createElement('strong'),status=document.createElement('small');
    name.textContent=job.operation;status.textContent=job.status+' · '+job.id.slice(0,8);text.append(name,status);
    const open=document.createElement('button');open.textContent='查看 / View';open.onclick=()=>show(job,job.operation);row.append(text,open);$('job-list').append(row);
  }
}
async function evidence() {
  const result=await request('evidence');$('events').textContent=JSON.stringify(result.events,null,2);
  $('artifact-list').replaceChildren();$('overlays').replaceChildren();
  for (const path of result.artifacts) {
    const link=document.createElement('a');link.href='/api/hardware/artifact?path='+encodeURIComponent(path);link.textContent=path;link.target='_blank';link.rel='noopener';$('artifact-list').append(link);
    if (path.endsWith('overlay.png')&&$('overlays').children.length<6) {
      const container=document.createElement('div'),img=document.createElement('img'),caption=document.createElement('a');img.src=link.href;img.alt='分割覆盖图 / Segmentation overlay';caption.href=link.href;caption.target='_blank';caption.rel='noopener';caption.textContent=path;container.append(img,caption);$('overlays').append(container);
    }
  }
}
async function manualAction(action, arguments_) {
  if(calling)return;
  calling=true;updateControls();notice('');$('action-result').hidden=false;
  $('action-result').textContent='正在处理 / Working…';
  let succeeded=false;
  try {
    const id=crypto.randomUUID();
    await request('manual/action',{request_id:id,action,arguments:arguments_});
    while(true){
      const snapshot=await request('state');state=snapshot;const job=snapshot.jobs[id];
      if(job&&!['accepted','running'].includes(job.status)){
        seen.add(id);acceptJob(job);succeeded=!!(job.result?.ok||job.result?.can_continue);
        if(job.status!=='completed')throw Error(job.error||job.status);
        break;
      }
      await new Promise(resolve=>setTimeout(resolve,250));
    }
  } finally {calling=false;await refresh();}
  if(succeeded)await call('get_robot_state');
}
function offsetMetres(){return ['move-x','move-y','move-z'].map(id=>Number($(id).value)/1000);}
$('move-offset').onclick=act(()=>manualAction('move',{delta_m:offsetMetres()}));
$('manual-gripper').onclick=act(()=>manualAction('gripper',{position:Number($('manual-width').value)/80}));
for(const button of document.querySelectorAll('[data-manual-width]'))button.onclick=act(async()=>{$('manual-width').value=button.dataset.manualWidth;await manualAction('gripper',{position:Number(button.dataset.manualWidth)/80});});
$('preview-offset').onclick=act(async()=>{
  const reply=await call('get_observation');fillPose(reply.value.robot_state.end_effector_poses.primary,offsetMetres());
  await call('plan_motion',{goal:{$pose:pose()}});
});
$('live-all').onclick=()=>cameraPanels(state.capabilities.cameras);
$('connect').onclick=act(()=>call('get_observation'));
$('state').onclick=act(()=>call('get_robot_state'));
$('observe').onclick=$('perception-freeze').onclick=act(()=>call('get_observation'));
$('use-current').onclick=act(async()=>{const reply=await call('get_robot_state');fillPose(reply.value.end_effector_poses.primary);});
$('offset').onclick=act(async()=>{const reply=await call('get_robot_state');fillPose(reply.value.end_effector_poses.primary,['dx','dy','dz'].map(id=>Number($(id).value)/1000));});
$('ik').onclick=act(()=>call('solve_ik',{target_pose:{$pose:pose()}}));
$('plan').onclick=act(async()=>{planHandle=null;await call('plan_motion',{goal:{$pose:pose()}});});
$('execute').onclick=act(()=>manualAction('move',{target_pose:pose()}));
$('gripper').onclick=act(()=>call('set_gripper',{position:Number($('width').value)}));
$('open-gripper').onclick=act(()=>call('open_gripper'));
$('close-gripper').onclick=act(()=>call('close_gripper'));
for (const [id,paused] of [['pause',true],['resume',false]]) $(id).onclick=act(async()=>{await request('pause',{paused});await refresh();});
$('arm').onclick=act(async()=>{show(await request('operator/authorize',{reason:$('authorization').value,probe_distance_m:Number($('probe-distance').value),confirmed:true}),'动作授权 / Motion authorization');await refresh();});
$('disarm').onclick=act(async()=>{await request('operator/revoke',{});await refresh();});
$('segment').onclick=act(async()=>{await call('segment_text',{camera_name:$('perception-camera').value,text:$('query').value});await evidence();});
$('localize').onclick=act(async()=>{await call('localize_object',{query:$('query').value,camera_name:$('perception-camera').value,target_frame:state?.capabilities?.perception_frame || 'base'});await evidence();});
$('perceive-all').onclick=act(async()=>{
  const camera=$('perception-camera').value;
  $('perception-output').textContent='正在冻结 RGB-D / Capturing RGB-D…';
  await call('get_observation');
  $('perception-output').textContent='SAM3 正在识别目标 / Segmenting target…';
  const segmented=await call('segment_text',{camera_name:camera,text:$('query').value});await evidence();
  if(segmented.value.segmentations.length!==1)throw Error(`识别到 ${segmented.value.segmentations.length} 个目标，请细化描述后重试。 / Ambiguous target; refine the description.`);
  $('candidate').value=0;
  const mask=reference('segment_text',['segmentations',0]);
  const cloud=await call('mask_to_point_cloud',{mask,camera_name:camera,target_frame:state?.capabilities?.perception_frame || 'base'});
  await call('estimate_geometry',{point_cloud:{$ref:cloud.handle}});
  $('perception-output').textContent='已定位，Contact-GraspNet 正在生成抓取姿态 / Located; generating grasps…';
  await call('generate_grasps',{camera_name:camera,mask,max_candidates:Number($('grasp-count').value)});await evidence();
});
$('project').onclick=act(()=>call('mask_to_point_cloud',{mask:reference('segment_text',['segmentations',Number($('candidate').value)]),camera_name:$('perception-camera').value,target_frame:state?.capabilities?.perception_frame || 'base'}));
$('geometry').onclick=act(()=>call('estimate_geometry',{point_cloud:reference('mask_to_point_cloud')}));
$('grasps').onclick=act(async()=>{await call('generate_grasps',{camera_name:$('perception-camera').value,mask:reference('segment_text',['segmentations',Number($('candidate').value)]),max_candidates:Number($('grasp-count').value)});await evidence();});
$('select-grasp').onclick=act(()=>call('select_grasp',{grasps:reference('generate_grasps'),strategy:'top_down'}));
$('api-tool').onchange=selectTool;
$('api-call').onclick=act(()=>call($('api-tool').value,JSON.parse($('api-arguments').value)));
$('cap-task').onchange=updateControls;
$('joint-goal').oninput=updateControls;
$('cap-model').onchange=act(async()=>{
  modelChoiceRestored=true;
  renderModelStatus();updateControls();
  await request('cap/model',{mode:$('cap-model').value});
});
$('cap-start').onclick=act(async()=>{
  const task=$('cap-task').value,body={model_mode:$('cap-model').value,task};
  if (task==='joint') body.goal=$('joint-goal').value;
  if (['probe','pick-place','joint'].includes(task)) Object.assign(body,{authorization:$('cap-authorization').value,probe_distance_m:Number($('cap-distance').value),confirmed:true});
  show(await request('cap/start',body),'CaP');await refresh();
});
$('cap-pause').onclick=act(async()=>{await request('cap/pause',{});await refresh();});
$('cap-continue').onclick=act(async()=>{
  calling=true;updateControls();
  try {
    show(await request('cap/continue',{session:capState.session_id,model_mode:$('cap-model').value,message:$('cap-message').value}),'CaP');
    $('cap-message').value='';
  } finally {calling=false;await refresh();}
});
$('cap-prepare').onclick=act(async()=>show(await request('cap/prepare',{})));
$('refresh-evidence').onclick=act(evidence);
$('end-session').onclick=$('disconnect-top').onclick=act(async()=>{
  ending=true;notice('正在结束控制台会话 / Ending console session…');
  try {
    const session=await request('session');
    if(session.backend==='arx_x5_2023_dual'&&session.hardware_lifecycle?.mode!=='resident_arx_owner_v1')throw Error('当前服务未加载常驻控制保护。请先可靠支撑双臂，再通过新版启动器的硬件维护结束会话。 / Support both arms and use the updated launcher maintenance flow.');
    await request('session/end',{confirmed:true});window.location.assign(launcherURL);
  }
  catch(error){ending=false;throw error;}
});
for (const button of document.querySelectorAll('[data-panel]')) button.onclick=()=>{
  history.replaceState(null,'','#'+button.dataset.panel);
  document.querySelectorAll('.page-panel').forEach(panel=>panel.hidden=panel.id!==button.dataset.panel);
  document.querySelectorAll('[data-panel]').forEach(item=>item.classList.toggle('active',item===button));
  syncPreviewWall();
  $('page-title').replaceChildren(button.querySelector('span').cloneNode(true));
  document.querySelector('.tools-menu').open=false;
  if (button.dataset.panel==='evidence-panel'||button.dataset.panel==='perception-panel') evidence().catch(error=>notice(error.message));
  if (['perception-panel','cap-panel'].includes(button.dataset.panel))serviceStatus().catch(error=>notice(error.message));
};
function renderModelStatus(){
  if(!modelServices)return;
  const mode=$('cap-model').value;
  if(mode==='codex'){
    const c=modelServices.codex;
    $('model-status').textContent=c
      ? `Codex 额度 / Codex plan · ${c.model} · ${c.reasoning} · `+(c.ready
        ? 'ChatGPT 已登录，无需 API key / Signed in with ChatGPT · 剩余额度请在 Codex 查看 / View remaining usage in Codex'
        : (c.message_zh||c.error)+' / Codex unavailable')
      : '正在读取 Codex 登录状态 / Checking Codex login…';
  }else if(mode==='fixture'){
    $('model-status').textContent='软件测试模型，无真实模型调用或额度消耗 / Software fixture; no model usage';
  }else{
    const m=modelServices.model;
    const missing=[];
    if(!m.base_url)missing.push('API 地址 / API URL');
    if(!m.model)missing.push('模型名称 / Model name');
    if(!m.key_loaded)missing.push('服务端密钥 / Server key');
    $('model-status').textContent=missing.length
      ? `API 配置缺少：${missing.join('、')}。请打开模型 API 配置${modelServices?.codex?.ready?'，或选择使用 Codex 额度':''}。 / Missing API settings; open settings or select Codex.`
      : `API · ${m.model} · ${m.reasoning||'—'} · ${m.base_url} · 密钥已加载 / Key loaded`;
  }
}
async function serviceStatus(){
  const value=await request('services');
  modelServices=value;
  if(!modelChoiceRestored){$('cap-model').value=value.selected_model||'configured';modelChoiceRestored=true;}
  $('provider-status').textContent=Object.entries(value.perception).map(([name,entry])=>name+'：'+(entry.ready?'已加载 / Ready':'未就绪 / Unavailable')).join(' · ');
  renderModelStatus();updateControls();
}
(async()=>{
  try {const session=await request('session');launcherURL=session.launcher_url;sessionId=session.session_id;$('real-link').href=launcherURL;$('launcher-link').href=launcherURL;$('launcher-top').href=launcherURL;await refresh();await loadCatalogue(true);if(!$('cap-task').dataset.restored)$('cap-task').value=state.capabilities.robot_backend==='arx_x5_2023_dual'?'joint':state.capabilities.synthetic?'mock':'visual';if(state.capabilities.synthetic)$('cap-model').value='fixture';updateControls();}
  catch(error) {notice(error.message);}
  const selectedPanel=Array.from(document.querySelectorAll('[data-panel]')).find(button=>'#'+button.dataset.panel===location.hash);
  if(selectedPanel)selectedPanel.click();
  serviceStatus().catch(error=>notice(error.message));
  setInterval(()=>refresh().catch(error=>notice(error.message)),1200);
})();

const handSteps = [-1, -.5, -.25, -.10, -.05, 0, .05, .10, .25, .5, 1];
const handSlider = $('cal-slider');
const handSide = $('cal-hand');
const handDelta = $('cal-delta');
const handExecute = $('cal-execute');
const handInputMode = $('cal-input-mode');
let handBusy = false;
let handPointer = null;

function showHandOffset() {
  const step = handSteps[Number(handSlider.value) + 5];
  const text = (step > 0 ? '+' : '') + step.toFixed(2) + ' rad';
  $('cal-offset').textContent = text;
  handSlider.setAttribute('aria-valuetext', text);
  return step;
}
function resetHandSlider() {
  handPointer = null;
  handSlider.value = '0';
  showHandOffset();
}
function updateHandControls() {
  const caps = state?.capabilities;
  const busy = !caps || !sessionId || handBusy || calling || caps.busy || !!catalogue?.cap_running;
  const blocked = busy || caps?.paused || caps?.closed;
  for (const button of document.querySelectorAll('[data-release-arm]')) {
    const calibrated = caps?.calibrated_grippers?.includes(button.dataset.releaseArm);
    button.disabled = blocked || caps?.faulted || !calibrated;
    button.title = calibrated ? '' : '该侧夹爪尚未标定 / This gripper is not calibrated';
  }
  handSide.disabled = busy;
  handSlider.disabled = blocked || caps.faulted;
  handDelta.disabled = handSlider.disabled;
  handExecute.disabled = handSlider.disabled;
  handInputMode.disabled = handSlider.disabled;
  if (handSlider.disabled) resetHandSlider();
  for (const button of $('hand-calibration-actions').querySelectorAll('button')) {
    if (button.dataset.handAction === 'acknowledge') {
      button.hidden = !caps?.faulted;
      button.disabled = blocked || !caps?.faulted;
    } else button.disabled = blocked || caps?.faulted;
  }
  $('cal-hand-state').textContent = caps?.faulted
    ? '运动已闭锁。确认现场情况后，选择上次点动的夹爪，点「复查并确认本侧夹爪故障」。 / Motion locked. Review the selected gripper fault.'
    : caps?.paused ? '已暂停 / Paused' : '';
}
handSlider.addEventListener('input', showHandOffset);
handDelta.addEventListener('input', () => handDelta.setCustomValidity(''));
handInputMode.addEventListener('change', () => {
  handDelta.value = '';
  handDelta.min = handInputMode.value === 'target' ? '0' : '-5';
  handDelta.setCustomValidity('');
});
handExecute.addEventListener('click', () => {
  if (handExecute.disabled) return;
  const delta = handDelta.valueAsNumber;
  const absolute = handInputMode.value === 'target';
  handDelta.setCustomValidity(Number.isFinite(delta) && (absolute || delta !== 0)
    ? '' : '请输入有效角度；增量须非零 / Enter a finite angle; relative changes must be nonzero');
  if (handDelta.reportValidity()) void handleHandAction(absolute ? 'target' : 'jog', delta);
});
handSlider.addEventListener('pointerdown', event => {
  if (handSlider.disabled || !event.isPrimary || event.button !== 0) return;
  handPointer = event.pointerId;
  handSlider.setPointerCapture(event.pointerId);
});
handSlider.addEventListener('pointerup', event => {
  if (handPointer !== event.pointerId) return;
  const delta = showHandOffset();
  resetHandSlider();
  if (delta) void handleHandAction('jog', delta);
});
handSlider.addEventListener('keydown', event => {
  if (event.key === 'Escape') resetHandSlider();
  if (event.key !== 'Enter' || event.repeat) return;
  event.preventDefault();
  const delta = showHandOffset();
  resetHandSlider();
  if (delta) void handleHandAction('jog', delta);
});
for (const name of ['pointercancel', 'lostpointercapture', 'blur']) {
  handSlider.addEventListener(name, resetHandSlider);
}
window.addEventListener('blur', resetHandSlider);
handSide.addEventListener('change', () => {
  resetHandSlider();
  $('cal-hand-feedback').textContent = '';
});
$('hand-calibration-actions').addEventListener('click', event => {
  const action = event.target.dataset.handAction;
  if (action) void handleHandAction(action);
});
for (const button of document.querySelectorAll('[data-release-arm]')) {
  button.addEventListener('click', async () => {
    if (button.disabled) return;
    const arm = button.dataset.releaseArm;
    const label = arm === 'left' ? '左爪 / Left gripper' : '右爪 / Right gripper';
    const feedback = $(button.dataset.releaseFeedback || 'gripper-release-feedback');
    handBusy = true;
    updateControls();
    clearArmPlans();
    feedback.textContent = label + ' · 正在张开，等候反馈 / Opening, awaiting feedback…';
    try {
      const reply = await call('open_gripper', {arm});
      const receipt = reply.value.diagnostics?.grippers?.[arm];
      if (!receipt?.target_reached || !Number.isFinite(receipt.measured_rad)) {
        throw Error('未确认夹爪到达开位，请查看反馈 / Open position was not verified; inspect feedback');
      }
      feedback.textContent = label + ' · 已到开位 / Open position reached · ' +
        receipt.measured_rad.toFixed(3) + ' rad';
    } catch (error) {
      feedback.textContent = label + ' · ' + errorText(error.message);
    } finally {
      handBusy = false;
      updateControls();
    }
  });
}
async function handleHandAction(action, delta) {
  if (handBusy || !sessionId || (['jog', 'target'].includes(action) && handSlider.disabled)) return;
  const side = handSide.value;
  if (action === 'acknowledge' && !window.confirm(
    '两臂已支撑、夹爪空载，已检查滑轨和线缆，无卡阻、异响或报警？确认后检查静止反馈并解除本次标定闭锁。 / Confirm both arms supported and the empty gripper clear and alarm-free.')) return;
  const feedback = $('cal-hand-feedback');
  handBusy = true;
  updateHandControls();
  feedback.textContent = '等候反馈 / Awaiting feedback…';
  try {
    let value;
    if (action === 'jog' || action === 'target') {
      const job = await request('gripper-calibration/jog', {
        request_id: crypto.randomUUID(), side,
        ...(action === 'target' ? {target_rad: delta} : {delta_rad: delta})
      });
      let receipt = job;
      while (['accepted', 'running'].includes(receipt.status)) {
        await new Promise(resolve => setTimeout(resolve, 300));
        const current = await request('state');
        if (current.session_id !== sessionId) throw Error('session_changed');
        receipt = current.jobs[job.id];
        if (!receipt) throw Error('missing_gripper_receipt');
      }
      if (receipt.status !== 'completed' || !receipt.result?.ok) {
        throw Error(receipt.error || receipt.result?.error?.message || receipt.status);
      }
      const d = receipt.result.diagnostics;
      feedback.textContent = (side === 'left' ? '左 / Left' : '右 / Right') +
        ' · ' + d.before_rad[side].toFixed(3) + ' → ' + d.measured_rad.toFixed(3) +
        ' rad · 目标 / Target ' + d.target_rad.toFixed(3) + ' rad' +
        (d.empty_closed_verified ? ' · 空载闭位 ≤ 0.1 rad / Empty closed position verified' : '');
    } else {
      if (action === 'acknowledge') value = await request('gripper-calibration/acknowledge', {side, confirmed:true});
      else if (action === 'open_rad' || action === 'closed_rad') {
        value = await request('gripper-calibration/record', {side, endpoint: action});
      } else value = await request('gripper-calibration/' + action, {side});
      feedback.textContent = value.measured_rad
        ? '左 / Left ' + value.measured_rad.left.toFixed(3) + ' rad · 右 / Right ' + value.measured_rad.right.toFixed(3) + ' rad'
        : JSON.stringify(value, null, 2);
    }
  } catch (error) {
    feedback.textContent = errorText(error.message);
  } finally {
    try { await refresh(); } finally { handBusy = false; updateHandControls(); }
  }
}
