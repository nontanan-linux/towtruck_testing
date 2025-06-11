import unittest
from unittest.mock import patch, MagicMock
import rclpy
from rclpy.node import Node
from towtruck_mission_func import TowtruckMission


class TestTowtruckMission(unittest.TestCase):
    @patch.object(TowtruckMission, 'request_func')
    def test_login_success(self, mock_request_func):
        # Initialize ROS 2 node
        rclpy.init(args=None)
        node = TowtruckMission()
        # Call the login method
        login_status, login_response = node.login('admin2', 'admin8')
        # Mock the successful response from the server
        mock_response = MagicMock()
        mock_response.status_code = login_response.status_code
        mock_response.json.return_value = login_response.get("access_token")
        mock_request_func.return_value = mock_response
        # Check if the login was successful and token was set
        self.assertTrue(login_status)
        self.assertEqual(node.token, "fake_token")
        self.assertTrue("Authorization" in node.headers)
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

    # @patch.object(TowtruckMission, 'request_func')
    # def test_login_failure(self, mock_request_func):
    #     # Mock the failed response from the server
    #     mock_response = MagicMock()
    #     mock_response.status_code = 401  # Unauthorized
    #     mock_request_func.return_value = mock_response
    #     # Initialize ROS 2 node
    #     rclpy.init(args=None)
    #     node = TowtruckMission()
    #     # Call the login method
    #     login_successful = node.login('admin2', 'admin')
    #     # Check if the login failed and token was not set
    #     self.assertFalse(login_successful)
    #     self.assertIsNone(node.token)
    #     # Clean up
    #     node.destroy_node()
    #     rclpy.shutdown()

    # @patch("requests.post")
    # def test_logout_success(self, mock_post):
    #     # Mock successful logout response
    #     mock_response = MagicMock()
    #     mock_response.status_code = 200
    #     mock_post.return_value = mock_response

    #     # Initialize ROS 2 node
    #     rclpy.init(args=None)
    #     node = TowtruckMission()
        
    #     # Call the logout method
    #     node.logout()
        
    #     # Check if logout message was logged (as no exception is raised, we assume it's successful)
    #     mock_post.assert_called_once()
        
    #     # Clean up
    #     node.destroy_node()
    #     rclpy.shutdown()

    # @patch("requests.post")
    # def test_logout_failure(self, mock_post):
    #     # Mock failed logout response
    #     mock_response = MagicMock()
    #     mock_response.status_code = 500  # Internal Server Error
    #     mock_post.return_value = mock_response

    #     # Initialize ROS 2 node
    #     rclpy.init(args=None)
    #     node = TowtruckMission()
        
    #     # Call the logout method
    #     node.logout()
        
    #     # Check if the error message was logged
    #     mock_post.assert_called_once()
        
    #     # Clean up
    #     node.destroy_node()
    #     rclpy.shutdown()

    # @patch.object(TowtruckMission, 'login')
    # @patch.object(TowtruckMission, 'publish_status')
    # def test_check_login(self, mock_publish_status, mock_login):
    #     # Mock login behavior
    #     mock_login.return_value = True
        
    #     # Initialize ROS 2 node
    #     rclpy.init(args=None)
    #     node = TowtruckMission()
        
    #     # Call the check_login method
    #     node.check_login()
        
    #     # Ensure login was called and status was published
    #     mock_login.assert_called_once()
    #     mock_publish_status.assert_called_once_with("Logged in successfully")
        
    #     # Clean up
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
    # unittest.TextTestRunner(verbosity=2)
