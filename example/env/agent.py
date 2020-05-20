class Agent(object):
    def init(self, env):
        raise NotImplementedError()

    @property
    def action_spec(self) -> dict:
        raise NotImplementedError()

    @property
    def observation_spec(self) -> dict:
        raise NotImplementedError()

    @property
    def observation(self) -> dict:
        raise NotImplementedError()

    def set_action(self, action):
        raise NotImplementedError()

    @property
    def metadata(self) -> dict:
        return dict()

    def step(self, env, step: int):
        raise NotImplementedError()


class Sensor(Agent):
    def action_spec(self) -> dict:
        return {}

    @property
    def observation_spec(self) -> dict:
        raise NotImplementedError()

    @property
    def observation(self) -> dict:
        raise NotImplementedError()

    def set_action(self, action):
        return {}

    @property
    def metadata(self) -> dict:
        return dict()


class AlwaysOnSensor(Sensor):
    def set_interval(self, interval: int):
        raise NotImplementedError()
