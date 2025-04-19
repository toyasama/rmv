from typing import List
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from threading import Thread, Lock
from time import sleep
from ..tf_management.graph import TransformGraph
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

qos_profile_transient = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

class TFManager():
    def __init__(self, node: Node, transform_graph: TransformGraph) -> None:
        """
        Gestionnaire pour la transformation TF avec structures avancées.
        """
        super().__init__()
        self.node = node

        self.transform_graph = transform_graph
        
        
        self.node.create_subscription(TFMessage, '/tf', self.tfCallback, 10)
        self.node.create_subscription(TFMessage, '/tf_static', self.tfStaticCallback, qos_profile_transient)

        self.node.get_logger().info("TFManager initialized successfully.")

    def tfCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations dynamiques.
        """
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations statiques.
        """
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=True)
            

