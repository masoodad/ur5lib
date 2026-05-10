import unittest
from ur5lib.io.simulator import UR5Sim

class TestUR5Base(unittest.TestCase):

    def test_initialization(self):
        robot = UR5Sim()
        self.assertIsNotNone(robot)
        self.assertEqual(robot.mode, "sim")

if __name__ == "__main__":
    unittest.main()
