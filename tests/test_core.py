import unittest
from ur5lib.core import UR5Base

class TestUR5Base(unittest.TestCase):

    def test_initialization(self):
        robot = UR5Base()
        self.assertIsNotNone(robot)
        self.assertEqual(robot.dialect, "SQL")  # example property

if __name__ == "__main__":
    unittest.main()
