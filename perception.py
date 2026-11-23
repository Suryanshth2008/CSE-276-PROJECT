"""
Perception Module for Vision-Based Outdoor UGV Navigation.

Implements robust computer vision algorithms to identify candidate traversable
paths from outdoor video frames, estimate path centerlines and lookahead waypoints,
calculate confidence scores, and categorize detection failures.
"""
import cv2
import numpy as np


class PathDetectionResult:
    """Encapsulates the perception output for a single frame."""
    def __init__(self, is_detected, center, contour=None, area=0.0,
                 lookahead_waypoint=None, heading_angle_deg=0.0,
                 confidence=0.0, failure_reason="", mask=None, roi_y=0):
        self.is_detected = is_detected
        self.center = center  # (cx, cy) in global image coordinates
        self.contour = contour  # Global image coordinate contour
        self.area = float(area)
        self.lookahead_waypoint = lookahead_waypoint  # (wx, wy) further up the path
        self.heading_angle_deg = float(heading_angle_deg)
        self.confidence = float(confidence)  # 0.0 to 1.0
        self.failure_reason = failure_reason  # None, NO_PIXELS, TOO_SMALL, DISCONNECTED
        self.mask = mask  # Binary mask
        self.roi_y = roi_y


class TraversablePathDetector:
    """
    Outdoor traversable path detector combining multi-channel color segmentation,
    shadow compensation, morphological structuring, and bottom-anchored connectivity.
    """
    def __init__(self, hsv_lower=(0, 0, 35), hsv_upper=(180, 100, 230),
                 min_area=1500, roi_fraction=0.42, use_shadow_compensation=True):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = int(min_area)
        self.roi_fraction = float(roi_fraction)
        self.use_shadow_compensation = use_shadow_compensation
        self.morph_kernel = np.ones((5, 5), np.uint8)

    def process_frame(self, frame):
        """
        Detects candidate traversable path in the lower image ROI.
        Returns a PathDetectionResult object.
        """
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_fraction)
        roi = frame[roi_y:, :]
        roi_h = roi.shape[0]

        # 1. Color Space Transformation & Preprocessing
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2. Candidate Color Masking
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 3. Shadow & Illumination Compensation (if enabled)
        if self.use_shadow_compensation:
            # Outdoor shadows shift Value while Hue/Saturation remain close to ground
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            l_channel = lab[:, :, 0]
            # CLAHE on L-channel to boost contrast in shaded areas
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            cl = clahe.apply(l_channel)
            # Retain shadow pixels if they share a-b color space with path
            shadow_mask = cv2.inRange(cl, 20, 110)
            combined_mask = cv2.bitwise_or(mask, cv2.bitwise_and(shadow_mask, mask))
            mask = combined_mask

        # 4. Morphological Filtering (Remove isolated gravel noise and bridge small gaps)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        # 5. Contour Extraction
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return PathDetectionResult(
                is_detected=False, center=None, failure_reason="NO_CANDIDATE_PIXELS",
                mask=mask, roi_y=roi_y
            )

        # Filter by minimum area
        candidates = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not candidates:
            return PathDetectionResult(
                is_detected=False, center=None, failure_reason="CONTOUR_BELOW_MIN_AREA",
                mask=mask, roi_y=roi_y
            )

        # 6. Bottom-Anchored Filtering
        # For a ground rover, the candidate traversable region must touch or be nearest
        # to the bottom of the image (the terrain immediately in front of the wheels)
        bottom_threshold = roi_h - int(roi_h * 0.20)
        anchored_candidates = [
            c for c in candidates
            if (cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3]) >= bottom_threshold
        ]
        # Prefer bottom-anchored candidates over floating background blobs (trees/clouds)
        selected_pool = anchored_candidates if anchored_candidates else candidates
        selected_pool.sort(key=cv2.contourArea, reverse=True)
        primary_contour = selected_pool[0]

        # 7. Moment & Centroid Calculation
        moments = cv2.moments(primary_contour)
        if moments["m00"] <= 0:
            return PathDetectionResult(
                is_detected=False, center=None, failure_reason="ZERO_MOMENT_AREA",
                mask=mask, roi_y=roi_y
            )

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"]) + roi_y
        area = float(cv2.contourArea(primary_contour))

        # Convert contour coordinates to global image frame
        global_contour = primary_contour.copy()
        global_contour[:, 0, 1] += roi_y

        # 8. Lookahead Waypoint & Path Heading Estimation
        # Slice contour into upper half (far lookahead) and lower half (near lookahead)
        x_box, y_box, w_box, h_box = cv2.boundingRect(primary_contour)
        far_y_roi = y_box + int(h_box * 0.25)
        # Find horizontal center of contour at far_y_roi
        mask_slice = mask[far_y_roi, :]
        nonzero_xs = np.where(mask_slice > 0)[0]
        if len(nonzero_xs) > 0:
            far_cx = int(np.mean(nonzero_xs))
            far_cy = far_y_roi + roi_y
        else:
            far_cx, far_cy = cx, cy

        # Calculate heading angle between near centroid and far lookahead point
        dx = far_cx - cx
        dy = -(far_cy - cy)  # Inverted Y for cartesian image orientation
        heading_angle = np.degrees(np.arctan2(dx, max(dy, 1)))

        # 9. Confidence Scoring
        # Metric combining normalized contour area, solidity, and bottom contact
        hull = cv2.convexHull(primary_contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / max(hull_area, 1.0)
        is_bottom_anchored = len(anchored_candidates) > 0

        # Confidence: solidity (0-1) * anchor_weight * area_factor
        area_factor = min(1.0, area / 20000.0)
        confidence = (0.5 * solidity + 0.3 * (1.0 if is_bottom_anchored else 0.4) + 0.2 * area_factor)
        confidence = float(np.clip(confidence, 0.05, 1.0))

        return PathDetectionResult(
            is_detected=True,
            center=(cx, cy),
            contour=global_contour,
            area=area,
            lookahead_waypoint=(far_cx, far_cy),
            heading_angle_deg=heading_angle,
            confidence=confidence,
            failure_reason="",
            mask=mask,
            roi_y=roi_y
        )
