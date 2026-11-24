'use strict';
const $=id=>document.getElementById(id);let pending=false, initialized=false;
function errorText(message){
  if(message.includes('launcher_update_required'))return '启动器仍是旧版本，尚未加载断开保护。请勿悬空重载或断开；实验结束、双臂可靠支撑后，再加载新版启动器并维护重启。 / The old launcher is still running. Finish the experiment and support both arms before migration.';
  if(message.includes('support_confirmation_required'))return '这次操作会停止硬件控制。请先可靠支撑双臂，再使用下方“硬件维护”。 / Support both arms, then use Hardware maintenance below.';
  if(message.includes('model_changed_requires_supported_maintenance'))return '硬件模型已改变，需要先支撑双臂再维护重启。 / Support both arms and restart hardware to load the changed model.';
  if(message.includes('station_busy')||message.includes('cap_session_busy'))return '机械臂正在处理操作，请完成后再切换。 / Finish the current operation before switching.';
  if(message.includes('Address already in use')||message.includes('runtime_still_closing'))return '设备正在断开，请稍后点击连接。 / Devices are closing; connect again shortly.';
  if(message.includes('Failed to fetch'))return '连接服务未启动。请运行 scripts/start-studio.sh。 / Start scripts/start-studio.sh.';
  return message;
}
async function request(path,body){const response=await fetch('/api/launcher/'+path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Console-Request':'1'},body:JSON.stringify(body)});const value=await response.json();if(!response.ok)throw Error(value.error||response.status);return value;}
function notice(text){$('notice').hidden=!text;$('notice').textContent=text;}
async function refresh(){
  const state=await request('state'),mock=state.session.backend==='mock';
  $('active-session').hidden=!state.running;
  $('session-title').textContent=mock?'当前为软件模拟 / Software simulation':(state.session.backend||'')+' · 真机面板 / Physical controls';
  $('session-detail').textContent=mock?'这里的状态和图像是模拟数据。点击上方“连接真机”即可切回机械臂。 / Click Connect robot above to use your real robot.':state.runtime?.closed?'正在断开 / Disconnecting…':state.runtime?.busy?'正在处理 / Working…':'可以打开面板，或直接选择另一个控制库。 / Open controls or select another library.';
  $('open-session').href=(state.session.url||'')+'/hardware';
  const busy=!!state.runtime?.busy||!!state.runtime?.cap_running;
  const legacyArx=state.session.backend==='arx_x5_2023_dual'&&state.runtime?.hardware_lifecycle?.mode!=='resident_arx_owner_v1';
  if(busy)$('hardware-supported').checked=false;
  $('stop-session').disabled=pending||busy||legacyArx;
  $('reload-session').disabled=pending||busy||legacyArx;
  $('hardware-maintenance').hidden=state.session.backend!=='arx_x5_2023_dual';
  $('maintenance-reload').disabled=pending||busy||!$('hardware-supported').checked||!state.lifecycle_protocol;
  $('maintenance-stop').disabled=pending||busy||!$('hardware-supported').checked||!state.lifecycle_protocol;
  if(legacyArx&&!pending)notice(state.lifecycle_protocol?'旧服务仍需支撑确认后维护迁移，不能普通重载。 / The old service requires supported maintenance migration.':errorText('launcher_update_required'));
  $('connect-real').disabled=pending||busy;$('backend-choice').disabled=pending||busy;
  for(const button of document.querySelectorAll('[data-backend]'))button.disabled=pending||busy;
  if(!initialized&&state.session.backend&&!mock){$('backend-choice').value=state.session.backend;initialized=true;}
  return state;
}
async function connect(backend){pending=true;notice(backend==='mock'?'正在打开软件模拟 / Opening simulation…':'正在连接工作站 / Connecting station…');try{const current=await refresh();if(current.session.backend==='arx_x5_2023_dual'&&backend!=='arx_x5_2023_dual'&&!current.session.hardware_disconnected)throw Error('hardware_support_confirmation_required');const state=await request('connect',{backend});window.location.assign(state.session.url+'/hardware');}catch(error){notice(errorText(error.message));}finally{pending=false;await refresh();}}
$('connect-real').onclick=()=>connect($('backend-choice').value);
for(const button of document.querySelectorAll('[data-backend]'))button.onclick=()=>connect(button.dataset.backend);
function requireRetainedOwner(state){if(state.session.backend==='arx_x5_2023_dual'&&state.runtime?.hardware_lifecycle?.mode!=='resident_arx_owner_v1')throw Error('hardware_support_confirmation_required');}
$('stop-session').onclick=async()=>{pending=true;notice('正在结束控制台会话 / Ending console session…');try{requireRetainedOwner(await refresh());await request('stop',{confirmed:true});notice('会话已结束。常驻 ARX 控制器仍保持运行。 / Session ended. Resident ARX control remains running.');}catch(error){notice(errorText(error.message));}finally{pending=false;await refresh();}};
$('reload-session').onclick=async()=>{pending=true;try{requireRetainedOwner(await refresh());await request('reload',{});notice('控制台已重载 / Console reloaded');}catch(error){notice(errorText(error.message));}finally{pending=false;await refresh();}};
$('hardware-supported').onchange=()=>refresh().catch(error=>notice(errorText(error.message)));
for(const action of ['reload','stop'])$('maintenance-'+action).onclick=async()=>{
  if(!$('hardware-supported').checked)return;
  pending=true;
  try{const current=await request('state');if(!current.lifecycle_protocol)throw Error('launcher_update_required');await request('maintenance/'+action,{hardware_supported:true});notice(action==='reload'?'硬件维护重启完成 / Hardware restarted':'硬件控制已停止 / Hardware control stopped');}
  catch(error){notice(errorText(error.message));}
  finally{$('hardware-supported').checked=false;pending=false;await refresh();}
};
refresh().catch(error=>notice(errorText(error.message)));setInterval(()=>{if(!pending)refresh().catch(error=>notice(errorText(error.message)));},2500);
