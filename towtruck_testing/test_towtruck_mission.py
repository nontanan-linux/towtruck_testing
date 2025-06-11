import unittest
from unittest.mock import patch, MagicMock
from towtruck_mission_func import TowtruckMission
import rclpy
import time

class TestTowtruckMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TowtruckMission()

    def tearDown(self):
        if self.node is not None:
            self.node.destroy_node()
        time.sleep(0.1)  # Small delay to ensure proper cleanup

    @patch("towtruck_mission_func.requests.post")
    def test_login_success(self, mock_post):
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {"access_token": "mock_token"}
        mock_post.return_value = mock_response
        success = self.node.login("testuser", "testpassword")
        self.assertTrue(success)
        self.assertEqual(self.node.token, "mock_token")
        self.assertEqual(self.node.headers["Authorization"], "Bearer mock_token")

    @patch("towtruck_mission_func.requests.post")
    def test_login_failure(self, mock_post):
        mock_response = MagicMock()
        mock_response.status_code = 401
        mock_response.json.return_value = {"error": "Invalid credentials"}
        mock_post.return_value = mock_response
        success = self.node.login("wronguser", "wrongpassword")
        self.assertFalse(success)
        self.assertIsNone(self.node.token)

    @patch("towtruck_mission_func.requests.post")
    def test_login_no_response(self, mock_post):
        mock_post.return_value = None  # Simulate no response from server
        success = self.node.login("admin2", "admin") 
        self.assertFalse(success)
        self.assertIsNone(self.node.token)

if __name__ == "__main__":
    unittest.main()
