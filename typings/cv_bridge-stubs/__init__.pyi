from robocell_sim.ros.messages import Image
from typing import Literal, overload

import numpy as np
import numpy.typing as npt

class CvBridge:
    @overload
    def imgmsg_to_cv2(
        self, img_msg: object, desired_encoding: Literal["rgb8"]
    ) -> npt.NDArray[np.uint8]: ...
    @overload
    def imgmsg_to_cv2(
        self, img_msg: object, desired_encoding: Literal["16UC1"]
    ) -> npt.NDArray[np.uint16]: ...
    def cv2_to_imgmsg(
        self,
        cvim: npt.NDArray[np.uint8] | npt.NDArray[np.uint16],
        encoding: str = "passthrough",
        header: object | None = None,
    ) -> Image: ...
