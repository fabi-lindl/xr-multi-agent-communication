import pytest
import rclpy
from rosagent.operation.rosagent import RosAgent
from rosrouter.router.rosrouter import RosRouter


@pytest.fixture
def init():
    rclpy.init()

@pytest.fixture(autouse=True)
def run_rosrouter(init):
    rosrouter = RosRouter()
    rosrouter.run()

def test_hs():
    # rclpy.init()

    # rosrouter = RosRouter()
    # rosrouter.run()
    # rclpy.spin(rosrouter)

    rosagent = RosAgent()
    # rosagent.run()

    # rclpy.shutdown()

    assert 2 + 2 == 5
