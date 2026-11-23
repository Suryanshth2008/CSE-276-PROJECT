"""
Navigation Module for Vision-Based Outdoor UGV.

Implements the formal navigation state machine, reference path tracking,
cross-track error calculation, and conservative steering decisions.
"""
from collections import deque
import numpy as np


class NavigationState:
    INIT = "INIT"
    PATH_FOUND = "PATH_FOUND"
    PATH_UNCERTAIN = "PATH_UNCERTAIN"
    STOP_NO_PATH = "STOP_NO_PATH"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class NavigationDecision:
    FORWARD = "FORWARD"
    TURN_LEFT = "TURN LEFT"
    TURN_RIGHT = "TURN RIGHT"
    STOP = "STOP / PATH UNCERTAIN"


class UGVNavigationEngine:
    """
    Consumes perception results, maintains a robust state machine,
    computes reference path deviation, and outputs safe qualitative steering decisions.
    """
    def __init__(self, deadband=0.08, min_confidence=0.30, max_uncertain_frames=5):
        self.deadband = float(deadband)
        self.min_confidence = float(min_confidence)
        self.max_uncertain_frames = int(max_uncertain_frames)

        self.current_state = NavigationState.INIT
        self.consecutive_lost_frames = 0
        self.history_errors = deque(maxlen=20)
        self.reference_center_x = None  # None = Image center (w // 2)

    def set_reference_path(self, target_center_x):
        """Set a dynamic reference centerline rather than default image center."""
        self.reference_center_x = target_center_x

    def step(self, perception_result, frame_width):
        """
        Processes one perception frame and returns:
        - decision: FORWARD, TURN LEFT, TURN RIGHT, STOP
        - state: Current navigation state
        - normalized_error: Normalized offset (-1.0 to +1.0)
        - reason_code: Diagnostic message
        """
        ref_x = self.reference_center_x if self.reference_center_x is not None else (frame_width / 2.0)
        half_w = frame_width / 2.0

        if not perception_result.is_detected or perception_result.confidence < self.min_confidence:
            self.consecutive_lost_frames += 1

            if self.consecutive_lost_frames <= self.max_uncertain_frames:
                # Brief flicker / shadow grace period
                self.current_state = NavigationState.PATH_UNCERTAIN
                # Maintain last known directional bias with reduced speed suggestion
                last_err = self.history_errors[-1] if self.history_errors else 0.0
                reason = f"LOW_CONFIDENCE ({perception_result.confidence:.2f}) [Grace Frame {self.consecutive_lost_frames}/{self.max_uncertain_frames}]"
                decision = NavigationDecision.STOP if self.consecutive_lost_frames >= 3 else self._error_to_decision(last_err)
                return decision, self.current_state, last_err, reason
            else:
                self.current_state = NavigationState.STOP_NO_PATH
                reason = f"PATH_LOST: {perception_result.failure_reason or 'BELOW_CONFIDENCE'}"
                return NavigationDecision.STOP, self.current_state, 0.0, reason

        # Valid Path Detected
        self.consecutive_lost_frames = 0
        self.current_state = NavigationState.PATH_FOUND

        cx, cy = perception_result.center
        # Normalized error relative to reference center:
        # Negative = Path is to the LEFT
        # Positive = Path is to the RIGHT
        raw_error = (cx - ref_x) / half_w
        normalized_error = float(np.clip(raw_error, -1.0, 1.0))

        # Integrate far-field heading angle to anticipate turns early
        if perception_result.heading_angle_deg != 0.0:
            heading_bias = (perception_result.heading_angle_deg / 45.0) * 0.15
            fused_error = float(np.clip(normalized_error + heading_bias, -1.0, 1.0))
        else:
            fused_error = normalized_error

        self.history_errors.append(fused_error)
        decision = self._error_to_decision(fused_error)
        reason = f"TRACKING (Confidence: {perception_result.confidence * 100:.0f}%)"

        return decision, self.current_state, fused_error, reason

    def _error_to_decision(self, error):
        if error < -self.deadband:
            return NavigationDecision.TURN_LEFT
        elif error > self.deadband:
            return NavigationDecision.TURN_RIGHT
        else:
            return NavigationDecision.FORWARD
