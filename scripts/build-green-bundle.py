from pathlib import Path
import json,hashlib,zipfile,copy
root=Path(__file__).resolve().parents[1]
out=root/'real_expriment';out.mkdir(exist_ok=True)
task=json.loads((root/'examples/green-cube-to-plate.json').read_text())
task.update(pipeline='cap',execution_policy='read_perceive_plan',validation_criteria='本轮只验证 FR3/夹爪状态读取、腕部与全局 RGB-D、绿色物体候选与盘子可见表面感知，以及从当前姿态出发的局部离线规划（例如向上 5 mm）。记录模型生成代码、执行结果及反馈；不得以此声明实际抓取放置成功。')
(root/'examples/green-cube-to-plate.json').write_text(json.dumps(task,ensure_ascii=False,indent=2)+'\n')
def encode(x):return (json.dumps(x,ensure_ascii=False,indent=2)+'\n').encode()
readme='''# 绿色立方体放入盘子 / Green cube into plate

最终物理目标：抓起桌面绿色立方体，将其放入盘子内侧承托区域，释放并稳定放置。
本次阶段：read_perceive_plan。只读取、感知、离线规划，不发送机器人或夹爪运动。
AC 与 C 文件刻意留空。它们是可选的冻结经验，不运行任何训练过程。

主流程：实时双相机观测 → 模型生成 def run(api) 代码 → 受限执行 → 结果/错误反馈 → 模型修正或离线验收。
先用图像确认绿色候选和盘子，再通过对齐深度得到可见表面点。颜色检测不能独立证明立方体身份。
离线规划仅验证小范围规划能力，例如当前末端向上 5 mm；不把此局部探针描述成完整抓放路径。

## 内容与复用
- experiment.json：任务、物理成功条件、本轮验收条件、运行方式、空 guidance。
- guidance/AC.md、guidance/C.md：空指导文件。
- reference/：本站硬件信息、站点与标定快照、SDK/接口文档、无密钥模型配置。
- snapshots/：此前真实 RGB-D 观测归档，已经过期，仅参考；主流程必须重新采集。
- examples/：只读示例代码，仅参考，不在上传时执行。
- manifest.json：版本、入口、上下文/指导文件与所有内容 SHA-256。

在面板“加载实验”中选 ZIP，再点“上传并加载 ZIP”。载入不会自动开始，不安装 ZIP 中的标定或站点配置。
模型密钥不在包内，服务器通过 ROBOT_MODEL_API_KEY 环境变量取得。
站点：fr3-single；模型：gpt-6-astra；推理：medium。以后可复制此格式生成新任务包。
'''
files={'experiment.json':encode(task),'TASK.md':readme.encode(),'guidance/AC.md':b'','guidance/C.md':b''}
for name in ('station.json','wrist-calibration.json','global-calibration.json'):
 files['reference/'+name]=(root/'config'/name).read_bytes()
hardware={'robot':{'model':'Franka FR3','ip':'172.16.0.2','gripper':'Franka Hand'},'wrist':{'model':'RealSense D435','serial':'261822078629','rgbd':[640,480]},'global':{'model':'Orbbec Gemini 435Le','serial':'CP4W863000KR','rgbd':[1280,800]},'depth_units':'meters','alignment':'depth_to_color','coordinates':'base; configured EE; quaternion wxyz','motion_enabled':False,'calibration_policy':'reference only; never install from ZIP','scene_geometry_reviewed':False}
files['reference/hardware.json']=encode(hardware)
files['reference/model.json']=encode({'base_url':'https://urtoken.ai/v1','model':'gpt-6-astra','reasoning':'medium','api_key_env':'ROBOT_MODEL_API_KEY','credentials_included':False})
for name in ('observe.py','plan_5mm_up.py','localize_pixel.py'):
 files['examples/'+name]=(root/'examples/cap'/name).read_bytes()
files['reference/cap-sdk.md']=(root/'docs/cap-sdk.md').read_bytes()
files['reference/cap-system-prompt.txt']=__import__('franka_player.cap_pipeline',fromlist=['SYSTEM']).SYSTEM.encode()
obs=json.loads((root/'runs/setup/gemini-rgbd-observation.json').read_text())['result']
obs=copy.deepcopy(obs);obs.update(fresh=False,archive_only=True,note='Historical physical capture. Reacquire before perception/planning.');obs.pop('directory',None)
for name,cam in obs['cameras'].items():
 for key in ('rgb_path','depth_path'):
  source=Path(cam[key]);target='snapshots/'+name+'/'+source.name;files[target]=source.read_bytes();cam[key]=target
files['snapshots/observation.json']=encode(obs)
manifest={'schema':'franka-experiment-bundle/v1','experiment':'experiment.json','context_files':['TASK.md','reference/hardware.json'],'guidance_files':[{'kind':'AC','path':'guidance/AC.md'},{'kind':'C','path':'guidance/C.md'}],'files':{k:hashlib.sha256(v).hexdigest() for k,v in sorted(files.items())}}
files['manifest.json']=encode(manifest)
path=out/'green-cube-to-plate.zip'
with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as archive:
 for name,body in files.items():archive.writestr(name,body)
from franka_player.bundle import parse_bundle
payload,metadata,context=parse_bundle(path.read_bytes());assert payload['guidance']==[]
(out/'README.md').write_text('实验 ZIP 放在本目录。当前任务：green-cube-to-plate.zip。\n在面板“加载实验”中选择 ZIP，点击“上传并加载 ZIP”。包内保留任务和参考信息，不含密钥。\n本轮仅做读取、感知、离线规划；离线验收通过不代表真实抓放成功。\n')
print(json.dumps({'zip':str(path),'bytes':path.stat().st_size,'files':len(files),'sha256':metadata['sha256']},ensure_ascii=False))
