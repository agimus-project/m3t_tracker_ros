from collections import deque
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
from typing import Generator, Union

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT, Time


@dataclass
class ImageQueueData:
    """Class storing all data relevant to the tracked images.
    Contains time stamp, color image and optional depth image.
    """

    stamp: Time
    frame_id: str
    color_image: npt.NDArray[np.uint8]
    color_camera_k: npt.NDArray[np.float64]
    depth_image: Union[None, npt.NDArray[np.float16]]
    depth_camera_k: Union[None, npt.NDArray[np.float64]]
    depth2color_pose: Union[None, npt.NDArray[np.float32]]


class ImageTimeBuffer:
    """Class handling time buffering for ImageQueueData structures."""

    def __init__(self, node: Node, timeout: float = 5.0) -> None:
        """Initializes the class.

        :param node: ROS node used to obtain time base for filtration.
        :type node: rclpy.Node
        :param timeout: Timeout in seconds used to filter too old
            values in the buffer, defaults to 5.0.
        :type timeout: float, optional
        """
        self._timeout = Duration(seconds=timeout)
        self._queue = deque()
        self._node = node

    def __len__(self) -> int:
        """Returns length of the queue.

        :return: Length of the queue
        :rtype: int
        """
        return len(self._queue)

    def __getitem__(self, idx: int) -> ImageQueueData:
        """Returns value of element in the queue at given index.

        :param idx: Index from which value should be obtained
        :type idx: int
        :return: Image data at given index
        :rtype: ImageQueueData
        """
        return self._queue[idx]

    def __next__(
        self, starting_stamp: Union[Time, None] = None
    ) -> Generator[ImageQueueData, None, None]:
        """Iterates over images from the buffer starting from the one with the closest
        time stamp to passed one. If ``None`` passed iterates from the beginning.

        :param stamp: Time stamp used to find closes image.
        :type starting_stamp: Union[Time, None], optional
        :raises ValueError: Time stamp is older than oldest value in the queue.
        :yield: Image from buffer
        :rtype: Generator[ImageQueueData, None, None]
        """
        if (self._node.get_clock().now() - starting_stamp) > self._timeout:
            t = starting_stamp.seconds_nanoseconds()
            raise ValueError(
                "When obtaining closest image from buffer "
                f"expected stamp '{t[0]}.{t[1]}' is too old!"
            )

        start_idx = self._queue.index(
            min(
                self._queue,
                key=lambda val: abs((val.stamp - starting_stamp).nanoseconds),
            )
        )
        for i in range(start_idx, len(self._queue)):
            yield self._queue[i]

        self._queue.clear()
        return

    @property
    def timeout(self) -> float:
        """Getter for buffer timeout value. Returns the timeout in seconds.

        :return: Timeout in seconds.
        :rtype: float
        """
        return self._timeout.nanoseconds / CONVERSION_CONSTANT

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        """Setter for buffer timeout value. If new timeout is smaller then old one
        length of the queue is updated.

        :param timeout: Time in seconds.
        :type timeout: float
        """
        old = self._timeout
        self._timeout = Time(seconds=timeout)
        if old > self._timeout:
            self._remove_too_old()

    def _remove_too_old(self) -> None:
        """Removes all images that are older than timeout."""
        now = self._node.get_clock().now()
        # Remove all images that are too old
        while len(self._queue) > 0 and (now - self._queue[0].stamp) > self._timeout:
            self._queue.popleft()

    def _is_queue_sorted(self) -> bool:
        """Sorts the image queue in time.

        :return: ``True`` if queue is sorted.
        :rtype: bool
        """
        return all(
            self._queue[i].stamp <= self._queue[i + 1].stamp
            for i in range(len(self._queue) - 1)
        )

    def append(self, image_data: ImageQueueData) -> None:
        """Inserts new image to the queue.

        :param images: New image to insert into queue.
        :type images: ImageQueueData
        """
        self._queue.append(image_data)
        if not self._is_queue_sorted():
            self._queue = sorted(self._queue, key=lambda val: val.stamp)

        self._remove_too_old()

    def get_closest(self, stamp: Time) -> int:
        """Returns image from the queue with a time stamp closest
        to one provided in the parameter.

        :param stamp: Time stamp used to find closes image.
        :type stamp: rclpy.Time
        :raises ValueError: Time stamp is older than oldest value in the queue.
        :return: Index of the closets detection in the buffer.
        :rtype: int
        """
        if (self._node.get_clock().now() - stamp) > self._timeout:
            t = stamp.seconds_nanoseconds()
            raise ValueError(
                "When obtaining closest image from buffer "
                f"expected stamp '{t[0]}.{t[1]}' is too old!"
            )

        return self._queue.index(
            min(self._queue, key=lambda val: abs((val.stamp - stamp).nanoseconds))
        )
