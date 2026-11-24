"""Human-readable Chinese errors alongside stable API codes and diagnostics."""
import re


def collision_prediction_message(message, details=None):
    """Describe only known model checks; a substring is not collision evidence."""
    text = str(message)
    details = details or {}
    known = ('planning_collision_predicted', 'Collision/clearance violation at path fraction ',
             'library_curve_collision', 'gripper_sweep_collision')
    if details.get('source') != 'geometric_prediction' and not any(key in text for key in known):
        return None
    hits = details.get('collisions', [])
    fraction = details.get('path_fraction')
    # Old receipts remain intact. Translate their recorded values for display.
    if not hits:
        pattern = r"'links':\s*\['([^']+)',\s*'([^']+)'\],\s*'distance_m':\s*([-+\d.eE]+)"
        hits = [dict(links=[a,b], distance_m=float(d)) for a,b,d in re.findall(pattern, text)]
    if fraction is None:
        match = re.search(r'path fraction ([\d.]+)', text)
        if match:fraction = float(match[1])
    phase = '夹爪开合检查' if 'gripper_sweep_collision' in text else '路径规划'
    parts = [phase+'未通过：几何模型预测相交']
    if fraction is not None:parts.append(f'路径 {fraction*100:.1f}% 处')
    if hits:
        hit = min(hits, key=lambda h: h['distance_m'])
        names = {'wrist_camera':'腕部相机包络', 'support':'桌面模型', 'held_object':'持物模型'}
        parts.append(' ↔ '.join(names.get(n, '深度障碍盒 '+n if n.startswith('observed_') else n) for n in hit['links']))
        distance = hit['distance_m']*1000
        parts.append(f'模型重叠 {abs(distance):.3f} mm' if distance<0 else f'模型间距 {distance:.3f} mm')
    else:parts.append('该记录未保存相交部件或距离')
    return '；'.join(parts)+'。本次未发送动作；这是规划预测，不是实机碰撞报警。'


RULES = (
    ('hardware_support_confirmation_required', '此操作会停止机械臂硬件控制。请先可靠支撑双臂，再到启动器“硬件维护”中确认；空闲不代表可以悬空断开。'),
    ('arx_owner_model_changed_requires_supported_maintenance', '硬件模型已改变。请可靠支撑双臂后执行维护重启，普通控制台重载不会替换正在运行的控制器。'),
    ('arx_owner_reply_unknown_no_retry', '硬件控制进程回复丢失，结果未知，未重试动作。请停止派发并检查控制器状态。'),
    ('arx_owner_connection_lost_new_session_required', '控制台与硬件控制进程连接已失效，请新建会话；不会自动恢复运动。'),
    ('model_configuration_error_key_format', '模型配置错误 / Model configuration error：API key 含非 ASCII、空白或控制字符，请重新填写。未保存此项修改 / Invalid key characters; changes were not saved'),
    ('model_configuration_error_key_missing', '模型配置错误 / Model configuration error：API key 未配置 / Missing API key'),
    ('model_configuration_error_model_missing', '模型配置错误 / Model configuration error：模型名称未配置 / Missing model name'),
    ('model_configuration_error_', '模型配置错误 / Model configuration error：请核对凭据、服务地址及访问权限 / Check credentials, endpoint and permissions'),
    ('model_endpoint_missing', '当前选择 API 模型，但 API 地址未配置；请打开“模型 API 配置”填写，或在模型入口选择“使用 Codex 额度”。任务尚未启动'),
    ('model_name_missing', 'API 模型名称未配置；请打开“模型 API 配置”填写模型名。任务尚未启动'),
    ('invalid_model_endpoint', 'API 地址格式无效，需要完整的 http(s) 服务地址，例如 https://urtoken.ai/v1；请检查模型 API 配置。任务尚未启动'),

    ("speed_factor must be at least", "所选速度系数低于当前 panda_py 轨迹生成器的原生下限；原始错误包含具体数值"),
    ("Motion activation belongs to Runtime", "运动启用由工作台 Runtime 管理；站点配置保持只读启动"),
    ('step_observation_failed_after_motion', '机械臂动作已有成功回执，但随后读取相机失败；请检查相机，动作不会重放'),
    ('action_must_name_primary', '单臂动作必须使用 primary，并传入 RobotAction 与 ArmCommand'),
    ('positions_must_name_primary', '夹爪字典必须使用 primary，例如 {"primary": 0.5}'),
    ('arm_unavailable', '该机械臂不存在；本站单臂名称为 primary'),
    ('home_robot_identity_changed', 'Home 记录属于其他机器人或关节定义，请更新本站 Home 配置'),
    ('joint_target_outside_native_range', '目标关节角超出 FR3 原生关节行程，请调整目标'),
    ('library_curve_joint_margin', '原生时间曲线超出 FR3 关节行程，请调整目标或路径'),
    ('robot_identity_changed', '轨迹的机器人型号或关节名称与当前设备不一致'),

    ('grasp_target_missing_or_ambiguous', '尚未登记唯一抓取目标，请重新感知并生成目标的抓取候选；夹爪指令未发送，可修订后继续'),
    ('cap_not_continuable', '当前任务无法继续：请等待正在运行的任务结束；已完成的任务可另开新任务'),
    ('cap_session_changed', '当前 CaP 会话已改变，请刷新后继续当前会话'),
    ('cap_task_mismatch', '继续操作必须使用原任务与原设备模式'),
    ('runtime_fault_latched', '控制器故障或动作结果未知，已停止；继续按钮不会清除该故障'),
    ('codex_usage_limit_reached', 'Codex 额度或请求速率已达限制，请在 Codex 查看额度后重试；没有自动切换到付费 API'),
    ('codex_chatgpt_login_required', 'Codex 的 ChatGPT 登录不可用，请在本机执行 codex login 登录后重试；无需重启机器人'),
    ('codex_upgrade_required_for_model', 'Codex CLI 版本不支持当前模型，需要更新 CLI 后重试'),
    ('codex_model_unavailable', '当前 Codex 账号无法使用所选模型，请检查模型与订阅权限'),
    ('codex_cli_not_installed', '未找到官方 Codex CLI，请安装后使用 ChatGPT 登录'),
    ('codex_cli_unavailable', '无法启动 Codex CLI，请检查服务器上的安装与权限'),
    ('invalid_codex_model_configuration', 'Codex 模型或思考深度配置无效'),
    ('codex_request_timeout', 'Codex 请求超时，本次未生成新程序；可重新运行 CaP，无需重启机器人'),
    ('codex_request_failed', 'Codex 请求失败，请检查网络、ChatGPT 登录及额度后重新运行 CaP；没有切换到 API'),
    ('codex_unexpected_tool_activity', 'Codex 返回了程序生成之外的工具活动，本次结果未交给机器人执行'),
    ('codex_turn_not_completed', 'Codex 没有返回完整结果，可重新运行 CaP；无需重启机器人'),
    ('speed_factor_out_of_range', '速度系数超出允许范围，本次未规划或执行；默认值、上限和请求值见详情'),
    ('franky_trajectory_start_state_changed', '轨迹启动前的状态校验未通过，本次轨迹未开始；请重新读取状态并规划'),
    ('configured_turn_budget_exhausted', '本轮达到配置的运行轮数，任务尚未确认完成'),
    ('process_restarted_no_replay', '工作台进程曾重启，任务记录已保留；点击“继续当前任务”后重新观察，不会自动重放动作'),
    ('observation_verification_or_model_transport_failed', '读取观测、核验任务或调用模型时发生错误，请查看错误类型和接口反馈'),
    ('cartesian_reflex', 'FR3 触发接触反射保护（cartesian_reflex），运动已中止；未自动恢复，请检查现场接触情况'),
    ('motion aborted by reflex', 'FR3 触发反射保护，运动已中止；具体控制器错误见详情'),
    ('Net Exception', '设备网络连接中断，请重新连接；不会重发动作'),
    ('NetworkException', '设备网络连接中断，请重新连接；不会重发动作'),
    ('gripper_target_not_reached', '夹爪未到达要求宽度，详情包含目标、实测值与偏差'),
    ('grasp_not_confirmed', '夹爪未确认抓住物体，请根据实测开度和图像调整抓取'),
    ('Connection to FCI refused', 'FR3 拒绝 FCI 连接，请在 Franka Desk 开启 FCI'),
    ('IK did not reach', '逆运动学未找到满足目标的关节姿态，请修改目标位姿'),
    ('IK solver did not converge', '逆运动学求解器未收敛，请根据求解反馈调整目标或初始姿态'),
    ('empty_start_required', '新任务启动时夹爪报告正在持物；若是在继续原任务，请使用“继续当前任务”'),
    ('Spline violates joint margin', '轨迹超出机器人关节行程，未发送动作，请调整目标位姿'),
    ('Provider unavailable', '感知服务无法连接，请查看 SAM3 / Contact-GraspNet 服务状态'),
    ('provider_unavailable', '感知服务无法连接，请查看服务状态'),
    ('No segmentation', '图像中未识别到该目标，请调整文本描述或观察视角'),
    ('no_segmentation', '图像中未识别到该目标，请调整文本描述或观察视角'),
    ('no segmentation', '目标识别置信度不足，请调整文本描述或观察视角'),
    ('no grasp candidates', '当前视角没有生成可用抓取，请换相机或重新观察'),
    ('insufficient', '有效观测数据不足，请重新采集或换相机'),
    ('grasp_target', '缺少当前观测的抓取目标，请先识别目标并生成抓取候选'),
    ('reference_from_previous_observation', '该结果来自上一帧，请用当前观测重新计算'),
    ('api_key_missing', 'API 密钥未加载到服务器环境；可在模型入口选择已登录的“使用 Codex 额度”，或为服务器提供 API 密钥。任务尚未启动或本次模型请求未发送'),
    ('model_request_failed', '模型服务请求失败，请查看 HTTP 状态码并检查接口配置'),
    ('station_busy', '上一项操作仍在进行，请等待完成'),
    ('session_changed', '设备会话已切换，请从连接页重新进入'),
    ('station_faulted_or_paused', '当前已暂停或存在未处理的执行异常，请查看设备状态'),
    ('paused', '当前已暂停，点击继续后可派发下一项操作'),
    ('expired', '观测或计划已过期，请重新观察并规划'),
    ('stale', '观测或计划与当前状态不一致，请重新观察并规划'),
    ('start_state_changed', '机器人位置已改变，请从实测位置重新规划'),
    ('robot_not_idle', '机器人未处于空闲状态或控制器存在错误，请在 Franka Desk 解除错误或退出引导模式后重试'),
    ('invalid_reference_path', '结果引用无效，请先完成前一步并选择有效候选'),
    ('tuple index out of range', '没有该编号的候选，请先查看识别结果'),
    ('unsupported', '当前设备尚不支持此操作'),
    ('outcome_unknown', '动作结果无法确认，已停止后续派发'),
)


def chinese_error(message, details=None):
    text = str(message)
    prediction = collision_prediction_message(text, details)
    if prediction:return prediction
    if any('\u4e00' <= c <= '\u9fff' for c in text):return text
    for key, label in RULES:
        if key in text:return label
    return '操作未完成，请查看错误详情：' + text
