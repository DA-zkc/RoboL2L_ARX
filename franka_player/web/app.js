const $=id=>document.getElementById(id);
let lang=localStorage.getItem('franka-language')||'zh', state=null, configLoaded=false, lastRender="";
const pairs={
recoverPath:['恢复 → 重新观察','Recover → observe'],pausePath:['暂停 → 继续观察','Pause → observe'],errorPath:['异常 → 人工处理','Error → operator'],runtime_validation_failed:['站点检查未通过，请检查适配器配置','Station validation failed; inspect adapter configuration'],observation_unavailable:['观测缺失或过期，请处理后重新观察','Observation is missing or stale; resolve and re-observe'],station_change_requires_end:['切换站点前请先结束当前尝试','End the current attempt before switching stations'],workspace:['实验工作区','EXPERIMENT WORKSPACE'],newExp:['＋ 加载实验','＋ Load experiment'],
loopback:['仅本机 / SSH 隧道','Loopback / SSH tunnel'],noTraining:['只做实验 · 不启动训练环路','Experiments only · No training loop'],
eyebrow:['机器人实验控制台 / 01','ROBOT EXPERIMENT CONSOLE / 01'],title:['让每一步执行都有依据','Every action, grounded.'],
boundaryTitle:['先接入，再执行','Connect before executing'],
boundaryText:['演示模式不连接机器人或模型。真实模式需安装适配器；暂停只阻止后续派发，不能替代硬件急停。','Demo never connects to robots or models. Live mode requires an installed adapter. Pause stops subsequent dispatch; it is not a hardware E-stop.'],
current:['当前实验','CURRENT EXPERIMENT'],mode:['运行模式','Mode'],attempt:['尝试','Attempt'],actions:['动作调用','Actions'],
pipeline:['实验流程','Experiment pipeline'],flowHint:['观察 → 推理 → 动作 → 验证；失败交回 Agent 判断。人工恢复后重新观察，不自动回 Home。','Observe → reason → act → verify. Failures return to the agent. Re-observe after human intervention; no automatic Home.'],
start:['开始新尝试','Start new attempt'],pause:['请求暂停','Request pause'],resume:['继续实验','Resume'],stop:['结束本次尝试','End attempt'],next:['下一个实验 →','Next experiment →'],
helpRequired:['等待操作者处理','Waiting for operator'],operatorNote:['处理情况与继续指令','Intervention and continuation instruction'],confirmHelp:['确认处理，重新观察','Confirm & re-observe'],
runtimeSettings:['API 与模型配置','API & model configuration'],boundarySettings:['只在非运行状态修改','Editable only when idle'],
api:['API 地址 · OpenAI-compatible','API base · OpenAI-compatible'],model:['模型 ID','Model ID'],reasoning:['推理深度','Reasoning effort'],keyEnv:['密钥环境变量名','API key environment variable'],budget:['每次尝试动作上限','Action limit per attempt'],station:['站点配置标识','Station configuration ID'],
secretHint:['这里只保存环境变量名，不接收或保存密钥。模型须支持 JSON 输出；推理深度须与 API 兼容。','Only the environment variable name is stored, never the secret. Model must support JSON output and the selected reasoning effort.'],
save:['保存运行配置','Save configuration'],guidance:['导入的 AC / C 经验','Imported AC / C guidance'],guidanceHint:['按实验冻结原文与 SHA-256；作为 Agent 上下文，不运行 AC/C 训练。','Text and SHA-256 frozen per experiment. Used as agent context; no AC/C training.'],
criteria:['物理成功条件','Physical success criteria'],evidence:['当前尝试 · 执行证据','Current attempt · Evidence'],export:['导出记录','Export records'],timeline:['事件时间线','Event timeline'],persistent:['持久保存','Persisted'],
footer:['RoboL2L / Franka Single Arm Player · 硬件接口、标定与动作限位须由站点适配器提供','RoboL2L / Franka Single Arm Player · Hardware, calibration and motion limits belong to the station adapter'],
loadTitle:['加载实验','Load experiment'],loadHint:['建立独立任务，选择经验文件。也可载入实验 JSON；不会自动开始运动。','Create an independent task and attach guidance, or load experiment JSON. Loading never starts motion.'],
name:['实验名称','Experiment name'],goal:['任务指令','Task instruction'],demo:['演示 · 无硬件 / 无模型','Demo · No hardware / model'],live:['真实 · 需要适配器','Live · Adapter required'],
demoOutcome:['演示结局','Demo outcome'],complete:['完成','Complete'],assist:['请求人工帮助','Request operator help'],
importAC:['导入 AC · Markdown / TXT','Import AC · Markdown / TXT'],importC:['导入 C · Markdown / TXT','Import C · Markdown / TXT'],loadJSON:['载入实验定义 JSON','Load experiment definition JSON'],load:['保存并载入','Save & load'],
emptyExp:['尚未加载实验','No experiment loaded'],emptyGoal:['从左侧加载任务与 AC/C 经验，开始搭建一次实验。','Load a task and AC/C guidance from the sidebar to begin.'],
emptyGuidance:['未导入经验 · 使用基础任务上下文','No guidance imported · Base task context only'],emptyEvidence:['尚无执行证据','No execution evidence yet'],emptyEvents:['等待实验事件','Waiting for experiment events'],
connected:['控制台已连接','Console connected'],disconnected:['连接中断','Console disconnected'],saved:['已保存','Saved'],
confirmStart:['确认开始一次新的尝试？真实模式下，请先确认现场就绪、运动区域无人。已有尝试会保留归档。','Start a new attempt? In live mode, confirm the scene is ready and the workspace is clear. Previous attempts are archived.'],
confirmEnd:['结束本次尝试？这不会移动机器人或自动摆场。','End this attempt? This will not move the robot or reset the scene.'],
ready:['就绪','Ready'],observing:['观察','Observe'],reasoningState:['推理','Reason'],acting:['执行','Act'],verifying:['验证','Verify'],completed:['完成','Completed'],paused:['已暂停','Paused'],pausing:['等待原子调用结束','Waiting for atomic call'],waiting_operator:['等待人工','Awaiting operator'],error:['错误','Error'],interrupted:['重启中断','Interrupted'],stopped:['已结束','Ended'],
experiment_loaded:['实验已加载','Experiment loaded'],experiment_selected:['已切换实验','Experiment selected'],config_saved:['配置已保存','Configuration saved'],started:['开始尝试','Attempt started'],resumed:['继续尝试','Attempt resumed'],pause_requested:['已请求暂停','Pause requested'],observed:['已获取观测','Observation received'],decision:['Agent 决策','Agent decision'],action_returned:['工具调用返回','Tool call returned'],operator_confirmed:['操作者已确认','Operator confirmed'],
demo_result:['演示流程结果（非物理结果）','Demo result (not a physical outcome)'],demo_action:['演示工具调用','Demo tool call'],action_budget_reached:['已达到动作预算，请检查并结束尝试或调整预算后继续','Action budget reached. Inspect and end attempt, or adjust budget before resuming.'],
hardware_adapter_missing:['尚未安装 Franka 真机适配器；真实模式无法启动','Franka hardware adapter is not installed; live mode cannot start'],
live_config_missing:['请填写 API、模型和站点配置','Configure API, model and station first'],api_key_missing:['服务器中缺少指定的 API 密钥环境变量','API key environment variable is missing on the server'],
operation_in_flight:['当前调用尚未结束，请等待暂停完成','An operation is still in flight; wait for pause to complete'],finish_current_first:['请先结束当前暂停／等待中的尝试，再切换实验','End the paused/waiting attempt before switching experiments'],
no_next_experiment:['队列中没有下一个实验','No next experiment in queue'],operator_message_required:['请说明人工处理情况','Describe the operator intervention'],name_goal_required:['请填写实验名称和任务','Enter an experiment name and goal'],criteria_required:['请填写成功条件','Enter success criteria'],model_request_failed:['模型请求失败；请核对地址、模型与推理设置，响应正文未记录','Model request failed; check endpoint, model and reasoning. Response body was not logged.'],
runtime_error:['运行接口发生错误；检查站点适配器','Runtime error; inspect the station adapter'],process_restarted:['进程已重启，需重新开始尝试','Process restarted; start a new attempt'],invalid_request:['请求格式无效','Invalid request'],
guidanceFileTooLarge:['经验文件超过 100 KB','Guidance file exceeds 100 KB'],jsonInvalid:['实验 JSON 格式无效','Invalid experiment JSON']
};
Object.assign(pairs, {
 confirmPhysicalStart:['开始真实抓放实验？Agent 将自主生成 CaP 并移动机械臂和夹爪。启动后不自动重试执行故障；请保持工作区无人、急停可用。','Start the physical experiment? The agent will generate CaP and move the robot and gripper. Execution faults stop without automatic retry; keep the workspace clear and E-stop available.'],
 physicalFlowHint:['CaP：实时观察 → 自主生成代码 → 有界真实执行 → 实测反馈 → 物理结果验证。','CaP: live observation → autonomous code generation → guarded physical execution → measured feedback → physical verification.'],
 physical_execution_fault_no_retry:['真实执行发生故障，已停止；不会自动重试','Physical execution fault; stopped without automatic retry'],
 loadZIP:['选择实验 ZIP','Choose experiment ZIP'], uploadZIP:['上传并加载 ZIP','Upload & load ZIP'], chooseZIP:['请先选择 ZIP 文件','Choose a ZIP file first'],
 zipHint:['ZIP 保留任务、AC/C 和参考资料；校验后加载，不执行附带代码、不替换本站标定。','ZIP preserves task, AC/C and references. Import validates files; it neither runs attached code nor installs calibration.'],
 loadHint:['优先上传实验 ZIP；也可手动填写或载入 JSON。加载不会开始执行。','Upload an experiment ZIP, or use the form / JSON. Loading never starts execution.'],
 validated:['离线验收通过','Offline validation passed'], code_generated:['模型生成代码','Model generated code'], program_returned:['程序执行反馈','Program feedback'],
 invalid_model_response:['模型响应格式错误，返回修正','Invalid model response; feedback returned'], incomplete_evidence:['验收证据不足','Insufficient validation evidence'],
 cap_round_budget_reached:['已达到 CaP 模型轮次上限','CaP model-round budget reached'],
 confirmCapStart:['开始 CaP 读取、感知与离线规划验收？会调用配置的模型并提交当前相机图像，不发送机械臂或夹爪运动指令。','Start CaP observation, perception and offline planning validation? The configured model receives current camera images; no robot or gripper motion is sent.'],
 budget:['每次尝试模型轮次上限','Model-round limit per attempt'], actions:['执行轮次','Execution rounds'],
 flowHint:['CaP：观察 → 模型生成代码 → 受限执行 → 反馈／修正 → 离线验收。真实运动禁用。','CaP: observe → model generates code → bounded execution → feedback / correction → offline validation. Physical motion disabled.'],
 bundle_size_limit:['ZIP 超过容量限制（压缩 20 MiB / 解压 50 MiB）','ZIP exceeds size limits (20 MiB compressed / 50 MiB expanded)'],
 bundle_invalid:['ZIP 格式无效','Invalid ZIP bundle'],bundle_schema_invalid:['ZIP manifest 版本或入口不正确','Unsupported manifest version or entry point'],
 bundle_hash_mismatch:['ZIP 文件校验失败','ZIP checksum mismatch'],bundle_manifest_mismatch:['ZIP 文件与 manifest 不一致','ZIP files do not match the manifest'],bundle_unsafe_path:['ZIP 含不允许的文件路径','ZIP contains unsafe file paths'],
});
function t(k){return pairs[k]?.[lang==='zh'?0:1]||k}
function status(s){return t(s==='reasoning'?'reasoningState':s)}
function el(tag,text,cls){const x=document.createElement(tag);if(text!==undefined)x.textContent=text;if(cls)x.className=cls;return x}
function toast(text){$('toast').textContent=text;$('toast').classList.add('show');clearTimeout(window.toastTimer);window.toastTimer=setTimeout(()=>$('toast').classList.remove('show'),4500)}
function translate(){document.documentElement.lang=lang==='zh'?'zh-CN':'en';document.querySelectorAll('[data-i]').forEach(x=>x.textContent=t(x.dataset.i));$('language').textContent=lang==='zh'?'EN':'中文';if(state)render()}
async function api(path,body){try{const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Request':'1'},body:JSON.stringify(body||{})});const value=await r.json();if(!r.ok)throw new Error(value.error);state=value;render();return true}catch(e){toast(t(e.message));return false}}
function selected(){return state?.experiments.find(e=>e.id===state.selected)}
function render(){
 const signature=JSON.stringify(state)+lang;if(signature===lastRender)return;lastRender=signature;
 const exp=selected(), busy=state.busy, active=exp?.status;
 const physical=exp?.execution_policy==='guarded_motion';
 document.querySelector('[data-i=flowHint]').textContent=t(physical?'physicalFlowHint':'flowHint');
 $('connection').textContent=t('connected');
 $('exp-name').textContent=exp?.name||t('emptyExp');$('exp-goal').textContent=exp?.goal||t('emptyGoal');
 $('mode').textContent=exp?(exp.mode==='demo'?'DEMO':exp.execution_policy==='guarded_motion'?'CaP · PHYSICAL':'CaP · READ/PLAN'):'—';$('attempt').textContent=exp?.attempt||'—';$('action-count').textContent=exp?.actions??'—';$('status').textContent=status(active||'ready');
 const list=$('experiments');list.replaceChildren();state.experiments.forEach(e=>{const b=el('button',e.name,e.id===state.selected?'selected':'');b.append(el('small',(e.mode==='demo'?'DEMO · ':'CaP · ')+status(e.status)));b.disabled=busy||['paused','waiting_operator'].includes(active);b.onclick=()=>api('select',{id:e.id});list.append(b)});
 const flow=$('flow');flow.replaceChildren();['ready','observing','reasoning','acting','verifying','completed'].forEach((s,i)=>{const n=el('div',undefined,'node'+((active===s||(s==='completed'&&active==='validated'))?' active':''));n.append(el('small','0'+(i+1)),el('b',status(s==='completed'&&exp?.mode==='live'&&exp?.execution_policy!=='guarded_motion'?'validated':s)));flow.append(n)});
 const branches=$('branches');branches.replaceChildren();[['recoverPath',['waiting_operator']],['pausePath',['paused','pausing']],['errorPath',['error','interrupted']]].forEach(([key,states])=>branches.append(el('div',t(key),'branch'+(states.includes(active)?' active':''))));
 $('start').disabled=!exp||busy||!['ready','completed','validated','stopped','error','interrupted'].includes(active);
 $('pause').disabled=!busy||active==='pausing';$('resume').disabled=busy||active!=='paused';$('stop').disabled=!exp||busy||['stopped','completed','validated'].includes(active);
 $('next').disabled=busy||['paused','waiting_operator'].includes(active)||state.experiments.findIndex(e=>e.id===state.selected)>=state.experiments.length-1;
 $('new-exp').disabled=busy||['paused','waiting_operator'].includes(active);
 $('assistance').hidden=active!=='waiting_operator';$('help-reason').textContent=t(exp?.detail||'');$('confirm-help').disabled=busy;
 ['base_url','model','reasoning','api_key_env','station','max_actions','save-config'].forEach(id=>$(id).disabled=busy);
 if(!configLoaded){Object.entries(state.config).forEach(([k,v])=>{if($(k))$(k).value=v});configLoaded=true}
 $('criteria').textContent=exp?.success_criteria||'—';$('guidance-count').textContent=exp?.guidance.length||0;
 const guide=$('guidance');guide.replaceChildren();if(!exp?.guidance.length)guide.append(el('div',t('emptyGuidance'),'empty'));else exp.guidance.forEach(g=>{const d=el('details');d.append(el('summary',g.kind+' · '+g.name),el('p','SHA-256 '+g.sha256,'hint'),el('pre',g.text));guide.append(d)});
 const evidence=$('evidence');evidence.replaceChildren();if(!exp?.history.length)evidence.append(el('div',t('emptyEvidence'),'empty'));else exp.history.slice(-12).reverse().forEach(h=>{const d=el('details');d.append(el('summary',({observation:t('observed'),decision:t('decision'),tool_result:t('action_returned'),program_decision:t(h.value?.kind==='program'?'code_generated':'decision'),program_result:t('program_returned'),operator:t('operator_confirmed')})[h.kind]||h.kind),el('pre',JSON.stringify(h.value||h.text,null,2)));evidence.append(d)});
 const events=$('events');events.replaceChildren();const items=state.events.filter(e=>!exp||e.experiment===exp.id||e.code==='config_saved').slice(-35).reverse();if(!items.length)events.append(el('div',t('emptyEvents'),'empty'));items.forEach(e=>{const div=el('div',undefined,'event');div.append(el('time',new Date(e.time*1000).toLocaleTimeString(lang==='zh'?'zh-CN':'en-GB')),el('b',status(e.code)));if(e.detail)div.append(el('p',t(e.detail)));events.append(div)});
}
$('language').onclick=()=>{lang=lang==='zh'?'en':'zh';localStorage.setItem('franka-language',lang);translate()};
$('new-exp').onclick=()=>$('new-dialog').showModal();$('close-dialog').onclick=()=>$('new-dialog').close();
$('start').onclick=()=>{if(confirm(t(selected()?.execution_policy==='guarded_motion'?'confirmPhysicalStart':selected()?.mode==='live'?'confirmCapStart':'confirmStart')))api('start',{confirmed:true})};
$('pause').onclick=()=>api('pause');$('resume').onclick=()=>api('resume');$('stop').onclick=()=>{if(confirm(t('confirmEnd')))api('stop')};$('next').onclick=()=>api('select',{next:true});
$('confirm-help').onclick=async()=>{if(await api('resume',{message:$('operator-note').value}))$('operator-note').value=''};
$('config-form').onsubmit=async event=>{event.preventDefault();const config={};['base_url','model','reasoning','api_key_env','station'].forEach(k=>config[k]=$(k).value.trim());config.max_actions=Number($('max_actions').value);if(await api('config',config))toast(t('saved'))};
let importedGuidance=[], importedExtras={};
$('json-file').onchange=async()=>{try{const file=$('json-file').files[0];if(!file)return;const p=JSON.parse(await file.text());if(!p.name||!p.goal||!p.success_criteria)throw Error();$('new-name').value=p.name;$('new-goal').value=p.goal;$('new-criteria').value=p.success_criteria;$('new-mode').value=p.mode||'demo';$('demo-outcome').value=p.demo_outcome||'complete';importedGuidance=p.guidance||[];importedExtras={pipeline:p.pipeline,execution_policy:p.execution_policy,validation_criteria:p.validation_criteria};toast(t('saved'))}catch{toast(t('jsonInvalid'))}};
$('new-form').onsubmit=async event=>{event.preventDefault();try{const guidance=[...importedGuidance];for(const [id,kind] of [['ac-file','AC'],['c-file','C']]){const file=$(id).files[0];if(file){if(file.size>100000)throw Error('guidanceFileTooLarge');guidance.push({kind,name:file.name,text:await file.text()})}}
 const ok=await api('experiments',{...importedExtras,name:$('new-name').value,goal:$('new-goal').value,success_criteria:$('new-criteria').value,mode:$('new-mode').value,demo_outcome:$('demo-outcome').value,guidance});if(ok){$('new-dialog').close();$('new-form').reset();importedGuidance=[];importedExtras={}}}catch(e){toast(t(e.message))}};
$('load-zip').onclick=async()=>{
 const file=$('zip-file').files[0];if(!file){toast(t('chooseZIP'));return}
 if(file.size>20*1024*1024){toast(t('bundle_size_limit'));return}
 $('load-zip').disabled=true;
 try{
  const r=await fetch('/api/experiments/import',{method:'POST',headers:{'Content-Type':'application/zip','X-Console-Request':'1'},body:file});
  const value=await r.json();if(!r.ok)throw Error(value.error);
  state=value;render();$('new-dialog').close();$('new-form').reset();importedGuidance=[];importedExtras={};toast(t('experiment_loaded'));
 }catch(e){toast(t(e.message))}finally{$('load-zip').disabled=false}
};
$('export').onclick=async()=>{const response=await fetch('/api/export');if(!response.ok)return;const blob=new Blob([JSON.stringify(await response.json(),null,2)],{type:'application/json'});const a=el('a');a.href=URL.createObjectURL(blob);a.download='franka-experiments.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
async function poll(){try{const r=await fetch('/api/state');if(!r.ok)throw Error();state=await r.json();render()}catch{$('connection').textContent=t('disconnected')}finally{setTimeout(poll,700)}}
translate();poll();
