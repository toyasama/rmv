from geometry_msgs.msg import Transform, Pose
import tf_transformations as tf

class TransformUtils:
    @staticmethod
    def invertTransform(transform: Transform) -> Transform:
        """
        Compute the inverse of a given transform.

        Args:
            transform (Transform): The transform to invert.

        Returns:
            Transform: The inverted transform.
        """
        matrix = tf.translation_matrix([transform.translation.x, transform.translation.y, transform.translation.z])
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        matrix[:3, :3] = tf.quaternion_matrix(rotation)[:3, :3]
        inverted_matrix = tf.inverse_matrix(matrix)

        inverted_translation = tf.translation_from_matrix(inverted_matrix)
        inverted_rotation = tf.quaternion_from_matrix(inverted_matrix)

        inverted_transform = Transform()
        inverted_transform.translation.x, inverted_transform.translation.y, inverted_transform.translation.z = inverted_translation
        inverted_transform.rotation.x, inverted_transform.rotation.y, inverted_transform.rotation.z, inverted_transform.rotation.w = inverted_rotation

        return inverted_transform

    @staticmethod
    def combineTransforms(transform_1: Transform, transform_2: Transform) -> Transform:
        """
        Combine two transformations.

        Args:
            transform_1 (Transform): The first transform.
            transform_2 (Transform): The second transform.

        Returns:
            Transform: The combined transform.
        """
        matrix1 = TransformUtils.transformToMatrix(transform_1)
        matrix2 = TransformUtils.transformToMatrix(transform_2)
        combined_matrix = tf.concatenate_matrices(matrix1, matrix2)
        return TransformUtils.matrixToTransform(combined_matrix)

    @staticmethod
    def transformToMatrix(transform: Transform):
        """Convert a Transform into a transformation matrix."""
        matrix = tf.translation_matrix([transform.translation.x, transform.translation.y, transform.translation.z])
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        matrix[:3, :3] = tf.quaternion_matrix(rotation)[:3, :3]
        return matrix

    @staticmethod
    def matrixToTransform(matrix) -> Transform:
        """Convert a transformation matrix back into a Transform."""
        transform = Transform()
        translation = tf.translation_from_matrix(matrix)
        rotation = tf.quaternion_from_matrix(matrix)
        transform.translation.x, transform.translation.y, transform.translation.z = translation
        transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = rotation
        return transform
    
    @staticmethod
    def poseToMatrix(pose: Pose):
        """
        Convert a Pose into a transformation matrix.

        Args:
            pose (Pose): The Pose to convert.

        Returns:
            numpy.ndarray: The 4x4 transformation matrix.
        """
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Create transformation matrix
        matrix = tf.translation_matrix(position)
        matrix[:3, :3] = tf.quaternion_matrix(orientation)[:3, :3]
        return matrix

    @staticmethod
    def matrixToPose(matrix) -> Pose:
        """
        Convert a transformation matrix back into a Pose.

        Args:
            matrix (numpy.ndarray): The 4x4 transformation matrix.

        Returns:
            Pose: The corresponding Pose.
        """
        translation = tf.translation_from_matrix(matrix)
        rotation = tf.quaternion_from_matrix(matrix)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
        return pose

    @staticmethod
    def transformPoseToParentFrame(pose: Pose, transform: Transform) -> Pose:
        """
        Transform a Pose from a child frame (frame 2) to the parent frame (frame 1)
        using the inverse of a given Transform.

        Args:
            pose (Pose): The Pose in the child frame (frame 2).
            transform (Transform): The Transform from the parent frame (frame 1) to the child frame (frame 2).

        Returns:
            Pose: The Pose in the parent frame (frame 1).
        """
        pose_matrix = TransformUtils.poseToMatrix(pose)
        transform_matrix = TransformUtils.transformToMatrix(transform)

        transformed_matrix = tf.concatenate_matrices(transform_matrix, pose_matrix)

        transformed_pose = TransformUtils.matrixToPose(transformed_matrix)
        return transformed_pose
   