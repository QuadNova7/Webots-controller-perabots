"""
Floor Color Classifier Module
Classifies the floor underneath the camera into:
- COLOR_WHITE (Section 1 floor)
- COLOR_BLACK (Section 2 floor)
- COLOR_RED (Start marker, Chamber transition, End marker)
- COLOR_BLUE (Section 2 Blue tile)
- COLOR_GREEN (Section 2 Green tile)
- COLOR_UNKNOWN
Uses multi-pixel sampling across the image center and confidence voting.
"""
from collections import deque
import config

class ColorClassifier:
    def __init__(self, camera):
        self.camera = camera
        self.history = deque(maxlen=5)

    def classify(self, img=None):
        """
        Classifies floor color from current camera image.
        Returns (dominant_color_id, confidence, avg_rgb).
        """
        if not self.camera:
            return config.COLOR_UNKNOWN, 0.0, (0, 0, 0)

        if img is None:
            img = self.camera.getImage()
        if not img:
            return config.COLOR_UNKNOWN, 0.0, (0, 0, 0)

        w = self.camera.getWidth()
        h = self.camera.getHeight()

        # Sample a 7x7 grid around center
        cx = w // 2
        cy = h // 2
        sample_radius = 5

        counts = {
            config.COLOR_WHITE: 0,
            config.COLOR_BLACK: 0,
            config.COLOR_RED:   0,
            config.COLOR_BLUE:  0,
            config.COLOR_GREEN: 0,
            config.COLOR_UNKNOWN: 0
        }

        total_r = 0
        total_g = 0
        total_b = 0
        total_samples = 0

        # Sample pixels
        for dx in range(-sample_radius, sample_radius + 1, 2):
            for dy in range(-sample_radius, sample_radius + 1, 2):
                px = cx + dx
                py = cy + dy
                if 0 <= px < w and 0 <= py < h:
                    r = self.camera.imageGetRed(img, w, px, py)
                    g = self.camera.imageGetGreen(img, w, px, py)
                    b = self.camera.imageGetBlue(img, w, px, py)

                    total_r += r
                    total_g += g
                    total_b += b
                    total_samples += 1

                    # Classify pixel
                    pixel_color = self._classify_pixel(r, g, b)
                    counts[pixel_color] += 1

        if total_samples == 0:
            return config.COLOR_UNKNOWN, 0.0, (0, 0, 0)

        avg_rgb = (total_r // total_samples, total_g // total_samples, total_b // total_samples)

        # Find dominant color
        dominant = config.COLOR_UNKNOWN
        max_count = 0
        for color, count in counts.items():
            if color != config.COLOR_UNKNOWN and count > max_count:
                max_count = count
                dominant = color

        confidence = max_count / float(total_samples)

        # Require at least 40% agreement
        if confidence < 0.40:
            dominant = config.COLOR_UNKNOWN

        self.history.append(dominant)
        return dominant, confidence, avg_rgb

    def get_filtered_color(self, img=None):
        """
        Returns stable color based on temporal majority filter.
        """
        curr, conf, rgb = self.classify(img)
        if not self.history:
            return curr, conf, rgb

        # Count frequencies in recent history
        freq = {}
        for c in self.history:
            freq[c] = freq.get(c, 0) + 1

        best_c = curr
        best_count = 0
        for c, count in freq.items():
            if c != config.COLOR_UNKNOWN and count > best_count:
                best_count = count
                best_c = c

        return best_c, conf, rgb

    def _classify_pixel(self, r, g, b):
        """Classify single RGB pixel based on verified ground-truth camera values."""
        # RED: Start marker, chamber transition tile, end marker (R ~ 133, G ~ 1, B ~ 1)
        if r >= 70 and max(g, b) <= 25 and r > 2.5 * max(g, b):
            return config.COLOR_RED

        # GREEN: Maze green tile (R ~ 1, G ~ 82, B ~ 1)
        if g >= 55 and max(r, b) <= 25 and g > 2.5 * max(r, b):
            return config.COLOR_GREEN

        # BLUE: Maze blue tile (R ~ 1, G ~ 1, B ~ 69)
        if b >= 45 and max(r, g) <= 25 and b > 2.5 * max(r, g):
            return config.COLOR_BLUE

        # WHITE: Section 1 floor (all channels > 80, small difference)
        if r > 80 and g > 80 and b > 80 and abs(r - g) < 35 and abs(r - b) < 35:
            return config.COLOR_WHITE

        # BLACK: Section 2 default floor (all channels in [15, 50], small difference)
        if 15 <= r <= 50 and 15 <= g <= 50 and 15 <= b <= 50 and abs(r - g) < 15 and abs(r - b) < 15:
            return config.COLOR_BLACK

        return config.COLOR_UNKNOWN
