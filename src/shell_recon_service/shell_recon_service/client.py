""" Shell Reconstruction ROS2 Client """
import argparse
from pathlib import Path
from shutil import rmtree
import pickle
from argparse import ArgumentParser
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import Image
from shell_recon_interfaces.srv import DoShellRecon


class ShellReconClient(Node):
    """Shell Reconstruction ROS2 Client Node"""

    def __init__(self, node_name: str) -> None:
        """
        Description:
            init client node and waits for shell reconstruction service
        Input:
            @param node_name <str>
                name of the client node
        """
        super().__init__(node_name)
        self.cli = self.create_client(DoShellRecon, "do_shell_recon")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for service ...")
        self.req = DoShellRecon.Request()

    def get_shell_recon(
        self, masked_depth: np.ndarray, camera_k: np.ndarray, number_points: int = 3000
    ) -> None:
        """
        Description:
            sends a reconstruction request whose response can be accessed via self.future
        Input:
            @param: masked_depth <np.ndarray> (D>=ShellReconstructor.depth_crop_width,
                 D>=ShellReconstructor.depth_crop_width)
                masked depth image of the target object for which shell reconstruction is required.
                Height and Width should be equal (D).
                D should be >= ShellReconstructor.depth_crop_width (see
                 ShellReconstructor.reconstruct)
            @param camera_k <np.ndarray>
                3x3 camera intrinsics matrix
            @param number_poitns <int>
                number of points to sample from the shell reconstruction mesh for the return point-
                    cloud
        """
        self.req.masked_depth = rnp.msgify(Image, masked_depth, "64FC1")
        self.req.camera_k = camera_k.flatten().tolist()
        self.req.number_points = number_points
        self.future = self.cli.call_async(self.req)


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--pkl_data_path",
        type=str,
        required=True,
        help="Path to pkl data (see shell_reconstruction repository demo_data/real_data.pkl",
    )
    args = parser.parse_args()
    rclpy.init(args=None)
    shell_recon_cli = ShellReconClient("shell_recon_client")

    # setup results directory
    data_path = Path(args.pkl_data_path)
    results_dir = data_path.parent / "reconstruction_ros_service"
    if results_dir.exists():
        rmtree(results_dir)
    results_dir.mkdir()

    def get_shell_recon_via_service(masked_depth, camera_k):
        shell_recon_cli.get_shell_recon(masked_depth, camera_k)
        np_pcd = None
        while rclpy.ok():
            rclpy.spin_once(shell_recon_cli)
            if shell_recon_cli.future.done():
                try:
                    response = shell_recon_cli.future.result()
                    shell_pcd = rnp.numpify(response.shell_pcd)
                    shell_recon_cli.get_logger().info("shell pcd received!")
                    np_pcd = np.empty((shell_pcd.shape[0], 3))
                    np_pcd[:, 0] = shell_pcd["x"]
                    np_pcd[:, 1] = shell_pcd["y"]
                    np_pcd[:, 2] = shell_pcd["z"]
                    break
                except Exception as e:
                    shell_recon_cli.get_logger().info(
                        f"failure occurred while reading data: {e}"
                    )
            else:
                shell_recon_cli.get_logger().info(
                    "waiting for response from service ..."
                )
        return np_pcd

    # read data
    with open(data_path, "rb") as pkl_file:
        data_dict = pickle.load(pkl_file)
    depth = np.asarray(data_dict["depth"]).astype(np.float64)
    camera_k = np.asarray(data_dict["camera_k"])
    for object_index_in_mask in data_dict["object_indexs_in_mask"]:
        # perform shell reconstruction for every object in the scene
        masked_depth = depth * (data_dict["mask"] == object_index_in_mask)
        masked_depth = masked_depth.astype(np.float64)
        np_pcd = get_shell_recon_via_service(masked_depth, camera_k)
        if np_pcd is not None:
            mesh_path = results_dir / Path(
                f"reconstruct_real_{object_index_in_mask}.ply"
            )
            o3dpcd = o3d.geometry.PointCloud()
            o3dpcd.points = o3d.utility.Vector3dVector(np.copy(np_pcd))
            o3d.io.write_point_cloud(str(mesh_path), o3dpcd)
            shell_recon_cli.get_logger().info(
                f"Shell reconstruction saved in the file {mesh_path}."
            )
        else:
            shell_recon_cli.get_logger().info(
                f"returned np_pcd is None for {object_index_in_mask}"
            )
    shell_recon_cli.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
