Object.assign(pairs, {
 capTitle:['Franka CAP · 设备与代码','Franka CAP · Devices & programs'],
 capReadOnly:['读取与离线规划','Observation & offline planning'],
 capHint:['FR3、腕部 D435 RGB-D、全局 Gemini RGB-D 相机。采集前请释放标定面板对相机的占用。本轮不发送运动指令。','FR3, wrist D435 RGB-D, and global Gemini RGB-D. Release cameras from calibration panels before capture. No motion commands in this deployment.'],
 capState:['读取机器人与夹爪','Read robot & gripper'],capObserve:['采集双相机','Capture both cameras'],
 capPlan:['规划向上 5 mm（不运动）','Plan 5 mm up (no motion)'],
 capProgramTitle:['运行 run(api) 代码','Run a run(api) program'],capRun:['运行读取／规划代码','Run observation / planning code'],
 capWorking:['正在读取或计算…','Reading or computing…'],capWrist:['腕部 D435 · RGB-D','Wrist D435 · RGB-D'],
 capGlobal:['全局 Gemini · RGB-D','Global Gemini · RGB-D'],
 boundaryTitle:['设备工具区：读取与离线规划','Device tools: observation and offline planning'],
 boundaryText:['此工具区保持只读。主实验按所载任务的运行策略执行；PHYSICAL 表示已授权的真实运动模式。','These device tools remain read-only. The main experiment follows the loaded task policy; PHYSICAL denotes authorized real motion.'],
});
const capButtons=['cap-state','cap-observe','cap-plan','cap-run'];
async function capRequest(name,body={}) {
 const response=await fetch('/api/cap/'+name,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Request':'1'},body:JSON.stringify(body)});
 const data=await response.json();if(!response.ok)throw Error(data.error);return data;
}
async function capJob(work) {
 capButtons.forEach(id=>$(id).disabled=true);$('cap-result').textContent=t('capWorking');
 try {const data=await work();$('cap-result').textContent=JSON.stringify(data,null,2);return data;}
 catch(error){$('cap-result').textContent=error.message;}
 finally{capButtons.forEach(id=>$(id).disabled=false);}
}
$('cap-state').onclick=()=>capJob(()=>capRequest('state'));
$('cap-observe').onclick=()=>capJob(async()=>{
 const data=await capRequest('observe');const obs=data.result;const images=$('cap-images');images.replaceChildren();
 for(const [name,cam] of Object.entries(obs.cameras)){
  const figure=el('figure'),image=el('img');image.alt=name;image.src='/api/cap/image?observation='+encodeURIComponent(obs.id)+'&camera='+encodeURIComponent(name);
  const label=el('figcaption',t(name==='wrist'?'capWrist':'capGlobal')+' · '+cam.serial);figure.append(image,label);images.append(figure);
 }return data;
});
$('cap-plan').onclick=()=>capJob(async()=>{
 const source='def run(api):\n    state = api.get_robot_state()\n    target = api.offset_pose(state["end_effector_pose"], [0, 0, 0.005])\n    return api.plan_motion(target)';
 return capRequest('program',{source});
});
$('cap-run').onclick=()=>capJob(()=>capRequest('program',{source:$('cap-source').value}));
fetch('/api/cap/capabilities').then(r=>{if(r.ok){$('cap-card').hidden=false;translate();}}).catch(()=>{});
