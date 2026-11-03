"""Render shared read-only telemetry without opening another serial device."""
def panel_lines(snapshot):
    snapshot = snapshot or {}
    values = snapshot.get("values") if snapshot.get("connected") else None
    if not values:
        return ["SENSORS: OFFLINE / STALE", "ROLL: --   PITCH: --   YAW: --", "ToF: --"]
    def angle(name):
        value = values.get(name)
        return "--" if value is None else f"{value:+.2f} deg"
    distance = values.get("distance_mm")
    return ["SENSORS: LIVE", f"ROLL: {angle('roll')}   PITCH: {angle('pitch')}",
            f"YAW: {angle('yaw')}   ToF: {'--' if distance is None else str(distance) + ' mm'}",
            "IMU: " + ("OK" if all(values.get(k) is not None for k in ('roll','pitch','yaw')) else "INVALID") +
            "   ToF: " + ("OK" if distance is not None else "INVALID")]


def annotate_sensors(image, snapshot, cv2):
    """Draw sensor text directly over the video without a background panel."""
    lines = panel_lines(snapshot)
    height = image.shape[0]
    line_height = 25
    top = max(0, height - len(lines)*line_height - 18)
    color = (70, 230, 100) if snapshot and snapshot.get('connected') else (70, 160, 255)
    for index, line in enumerate(lines):
        cv2.putText(image, line, (12, top+23+index*line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, .60, color, 1, cv2.LINE_AA)
    return image
