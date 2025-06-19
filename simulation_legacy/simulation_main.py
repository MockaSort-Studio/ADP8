import simulation
import os
import simulation_scenario
import simulation_agent
import pybullet
import os

URDF_PATH = os.path.dirname(__file__) + "/robot_model/spot.xml"


class TestScenario(simulation_scenario.BaseSimulationScenario):
    def __init__(self) -> None:
        super().__init__("SimpleTestScenario")

    def create_environment(self):
        shift = [0, -0.0, 0]
        meshScale = [0.1, 0.1, 0.1]
        visualShapeId = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_BOX,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, 0.4, 0],
            halfExtents=[1, 1, 0.5],
            visualFramePosition=shift,
        )
        collisionShapeId = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX,
            collisionFramePosition=shift,
            halfExtents=[1, 1, 0.5],
        )
        rangex = 5
        rangey = 5
        for i in range(rangex):
            for j in range(rangey):
                s = pybullet.createMultiBody(
                    baseMass=1000,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=collisionShapeId,
                    baseVisualShapeIndex=visualShapeId,
                    basePosition=[((-rangex / 2) + i) * 5, (-rangey / 2 + j) * 5, 1],
                    useMaximalCoordinates=True,
                )


def main():
    try:
        sim_parameters = simulation.SimulationParameters()
        sim_client = simulation.SimulationClient(sim_parameters)
        agent = simulation_agent.SimulationAgent("Marley", URDF_PATH)
        scenario = TestScenario()
        sim_client.start()
        sim_client.set_simulation_scenario(scenario)
        sim_client.set_simulated_agent(agent)
        while True:
            sim_client.step()
    except KeyboardInterrupt:
        pass
    finally:
        sim_client.shutdown()


if __name__ == "__main__":
    main()
