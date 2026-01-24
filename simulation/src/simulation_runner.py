import genesis as gs
from simulation.src.base_entity import BaseEntity


class SimulationRunner:
    def __init__(
        self,
        agent: BaseEntity,
        # scenario: list[BaseEntity],
        show_viewer=False,
        dt=0.01,
        log_level="warning",
    ):
        gs.init(precision="32", logging_level=log_level, backend=gs.cpu)

        self.scene = gs.Scene(
            show_viewer=show_viewer,
            sim_options=gs.options.SimOptions(dt=dt, gravity=(0, 0, -9.81)),
        )

        # add entities / scenario
        self.scene.add_entity(
            gs.morphs.Plane(),
        )

        # add agent
        self.agent = agent
        self.agent.add_to_scene(self.scene)

        # finally build scene
        self.scene.build()
        print("Done building Genesis scene!")

    def get_observations(self):
        return self.agent.get_observation()

    # def get_groundtruths(self):
    #   return self.agent.get_groundtruth()

    def set_inputs(self, inputs: list):
        self.agent.step(inputs)

    def step(self, inputs: list):
        self.set_inputs(inputs)
        self.scene.step()
