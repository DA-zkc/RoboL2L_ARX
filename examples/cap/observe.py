def run(api):
    state = api.get_robot_state()
    observation = api.get_observation()
    return {"state": state, "observation": observation}
