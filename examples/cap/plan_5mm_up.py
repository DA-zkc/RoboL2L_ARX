def run(api):
    state = api.get_robot_state()
    target = api.offset_pose(state["end_effector_pose"], [0, 0, 0.005])
    plan = api.plan_motion(target)
    return plan
