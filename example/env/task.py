class Task:
    def init(self, env) -> None:
        """called to initialize the task with environment.

        This function can be used to add objects to the env.
        """
        raise NotImplementedError()

    def reset(self, env) -> None:
        """Reset the task to its initial state.

        This function is called at anytime after init
        """
        raise NotImplementedError()

    def close(self, env) -> None:
        """Called before the environment is about to be destroyed."""
        pass

    def before_step(self, env) -> None:
        """Called before the control step of the environment."""
        raise NotImplementedError()

    def after_step(self, env) -> None:
        """Called after the control step of the environment."""
        raise NotImplementedError()

    def before_substep(self, env) -> None:
        """Called before each physical step of the environment"""
        raise NotImplementedError()

    def after_substep(self, env) -> None:
        """Called after each physical step of the environment"""
        raise NotImplementedError()

    @property
    def parameters(self) -> dict:
        """parameters needed to specify current task."""
        raise NotImplementedError()

    def get_trajectory_status(self) -> dict:
        """Give feedback to the current status of the task.

        e.g. success, score, failure, reason for failure
        """
        raise NotImplementedError()

    @property
    def description(self) -> str:
        """Human readable description for the task."""
        return ''
