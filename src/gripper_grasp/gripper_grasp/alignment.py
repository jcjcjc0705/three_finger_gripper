"""Is the object in front of the hand, rather than off to one side.

This is deliberately crude. It compares a central patch of the image against a
border ring and calls it aligned when the two differ enough -- in brightness, or
in how much texture each holds. It does not know what an object is, and it will
be fooled by a busy background.

Its job is to hold the shape of the interface until something better replaces
it. Swap the body of `check` for a detector and nothing else moves.

Pure numpy on purpose: pulling in cv_bridge or OpenCV to compute a mean would
add them to the runtime image for no reason.
"""

import numpy as np


def _luma(msg):
    """Grey image from a sensor_msgs/Image, or None if the encoding is unknown."""
    height, width = msg.height, msg.width
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    encoding = msg.encoding.lower()

    if encoding in ('mono8', '8uc1'):
        return raw[:height * width].reshape(height, width).astype(np.float32)

    if encoding in ('rgb8', 'bgr8'):
        pixels = raw[:height * width * 3].reshape(height, width, 3)
        return pixels.mean(axis=2, dtype=np.float32)

    if encoding in ('yuv422', 'yuv422_yuy2', 'yuyv'):
        # Y is every other byte; chroma is not worth unpacking for this.
        pairs = raw[:height * width * 2].reshape(height, width, 2)
        return pairs[:, :, 0].astype(np.float32)

    return None


class CentreAlignment:
    """Aligned when the middle of the frame stands out from its border."""

    def __init__(self, roi=0.35, threshold=0.12):
        self._roi = roi
        self.threshold = threshold

    def check(self, msg):
        """Returns (aligned, score in 0..1). Score is 0 when unreadable."""
        grey = _luma(msg)
        if grey is None or grey.size == 0:
            return False, 0.0

        height, width = grey.shape
        half_h = int(height * self._roi / 2)
        half_w = int(width * self._roi / 2)
        if half_h < 2 or half_w < 2:
            return False, 0.0

        cy, cx = height // 2, width // 2
        centre = grey[cy - half_h:cy + half_h, cx - half_w:cx + half_w]

        border = np.concatenate([
            grey[:half_h, :].ravel(), grey[-half_h:, :].ravel(),
            grey[:, :half_w].ravel(), grey[:, -half_w:].ravel(),
        ])
        if centre.size == 0 or border.size == 0:
            return False, 0.0

        spread = max(float(grey.std()), 1.0)
        brightness = abs(float(centre.mean()) - float(border.mean())) / spread

        # Texture, as mean absolute gradient. An object usually carries edges the
        # background at this distance does not, or the other way round; either
        # difference counts.
        def texture(patch):
            if patch.shape[0] < 2 or patch.shape[1] < 2:
                return 0.0
            return float(np.abs(np.diff(patch, axis=0)).mean() +
                         np.abs(np.diff(patch, axis=1)).mean())

        centre_texture = texture(centre)
        frame_texture = max(texture(grey), 1e-3)
        contrast = abs(centre_texture - frame_texture) / frame_texture

        score = min(1.0, 0.5 * brightness + 0.5 * contrast)
        return score >= self.threshold, score
