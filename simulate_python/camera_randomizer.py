import cv2
import numpy as np


class DepthRandomizer:
    def __init__(self, **kwargs):
        self.gaussian_std = kwargs.get("gaussian_std", 0.001)
        self.dropout_rate = kwargs.get("dropout_rate", 0.0)
        self.quantization = kwargs.get("quantization", 0.0)
        self.edge_distortion = kwargs.get("edge_distortion", False)
        self.edge_shift_std = kwargs.get("edge_shift_std", 0.02)
        self.edge_kernel_size = kwargs.get("edge_kernel_size", 5)
        self.systematic_missing_rate = kwargs.get("systematic_missing_rate", 0.0)
        self.missing_patch_size = kwargs.get("missing_patch_size", (2, 10))

        self._rng = np.random.RandomState()

    def randomize(self, depth):
        depth = depth.copy().astype(np.float32)
        depth = self._gaussian_noise(depth)
        depth = self._dropout(depth)
        depth = self._quantize(depth)
        depth = self._edge_distortion_func(depth)
        depth = self._systematic_missing(depth)
        return depth

    def _gaussian_noise(self, depth):
        if self.gaussian_std <= 0:
            return depth
        noise_scale = self.gaussian_std * (depth ** 2)
        noise = self._rng.randn(*depth.shape).astype(np.float32)
        depth = depth + noise * noise_scale
        depth = np.maximum(depth, 0)
        return depth

    def _dropout(self, depth):
        if self.dropout_rate <= 0:
            return depth
        valid = depth > 0
        mask = self._rng.rand(*depth.shape) >= self.dropout_rate
        mask = mask.astype(np.float32)
        depth = depth * mask
        depth[~mask.astype(bool) & valid] = 0
        return depth

    def _quantize(self, depth):
        if self.quantization <= 0:
            return depth
        depth = np.round(depth / self.quantization) * self.quantization
        return depth

    def _edge_distortion_func(self, depth):
        if not self.edge_distortion:
            return depth
        valid = (depth > 0).astype(np.float32)
        if cv2.countNonZero(valid) == 0:
            return depth

        grad_x = cv2.Sobel(valid, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(valid, cv2.CV_32F, 0, 1, ksize=3)
        edge = (np.abs(grad_x) + np.abs(grad_y)) > 0.1

        if not np.any(edge):
            return depth

        shift_x = self._rng.randn(*depth.shape).astype(np.float32) * self.edge_shift_std
        shift_y = self._rng.randn(*depth.shape).astype(np.float32) * self.edge_shift_std
        ks = self.edge_kernel_size
        shift_x = cv2.GaussianBlur(shift_x, (ks, ks), sigmaX=ks / 3.0)
        shift_y = cv2.GaussianBlur(shift_y, (ks, ks), sigmaX=ks / 3.0)

        edge_mask = cv2.dilate(edge.astype(np.uint8), np.ones((ks, ks), np.uint8)).astype(np.float32)

        depth = depth + shift_x * edge_mask * (depth > 0)
        depth = np.maximum(depth, 0)
        return depth

    def _systematic_missing(self, depth):
        if self.systematic_missing_rate <= 0:
            return depth
        patch_min, patch_max = self.missing_patch_size
        h, w = depth.shape
        num_patches = int(h * w * self.systematic_missing_rate / ((patch_min + patch_max) / 2) ** 2)
        for _ in range(max(num_patches, 1)):
            ph = self._rng.randint(patch_min, patch_max + 1)
            pw = self._rng.randint(patch_min, patch_max + 1)
            y0 = self._rng.randint(0, max(h - ph, 1))
            x0 = self._rng.randint(0, max(w - pw, 1))
            depth[y0 : y0 + ph, x0 : x0 + pw] = 0
        return depth
