""" Shell Reconstruction ROS2 Service """
import numpy as np
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from shell_recon_interfaces.srv import DoShellRecon
from shell.reconstructor import ShellReconstructor


class ShellReconService(Node):
    """Shell reconstruction ros2 service"""

    def __init__(self, node_name: str):
        """
        Description:
            init service node
        Input:
            @param node_name <str>
                name of the service node
        """
        super().__init__(node_name)
        self.srv = self.create_service(
            DoShellRecon, "do_shell_recon", self.do_shell_recon
        )
        self.shell_reconstructor = ShellReconstructor()

    def do_shell_recon(self, request, response):
        """
        Description:
            request handler for performing shell reconstruction
        Input:
            @param: request 
            @param: response
                both request and response confirms to DoShellRecon service interface
                 parameters
        Output:
            @param: response
        """
        masked_depth = rnp.numpify(request.masked_depth)
        camera_k = np.array(request.camera_k).reshape((3, 3))
        number_points = int(request.number_points)
        np_pcd = self.shell_reconstructor.reconstruct(masked_depth, camera_k, number_points)
        data = np.zeros(
            np_pcd.shape[0],
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        data["x"] = np_pcd[:, 0]
        data["y"] = np_pcd[:, 1]
        data["z"] = np_pcd[:, 2]
        response.shell_pcd = rnp.msgify(PointCloud2, data)
        return response


def main(args=None):
    rclpy.init(args=args)
    shell_recon_service = ShellReconService("shell_recon_service")
    rclpy.spin(shell_recon_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
