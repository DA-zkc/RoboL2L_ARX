def run(api):
    observation = api.get_observation("wrist")
    # This is a visible-surface diagnostic, not an object selection or grasp command.
    point = api.localize_pixel(observation["id"], "wrist", [320, 240])
    return point
