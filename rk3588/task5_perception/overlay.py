"""ASCII labels for board OpenCV; colors refer to measured regions."""
import cv2


def annotate(image, observations, state):
    result = image.copy()
    cv2.rectangle(result, (0, 0), (result.shape[1], 62), (0, 0, 0), -1)
    cv2.putText(result, 'TASK5 | ' + state['status'] + ' | NO MOTOR CONTROL',
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    cv2.putText(result, 'Color = approximate ROI; track ID is not identity',
                (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
    for item in observations:
        selected = state.get('confirmed') and state.get('observation_index') == item.index
        pen = (0, 255, 0) if selected else (0, 255, 255)
        x1, y1, x2, y2 = item.box
        cv2.rectangle(result, (x1, y1), (x2, y2), pen, 3 if selected else 2)
        title = '#%d %s %.2f' % (item.index, item.label, item.score)
        if selected:
            title += ' LOCK ID=%d' % state['track_id']
        colors = ' '.join('%s:%s %.0f%%' % (name, value['name'], value['fraction']*100)
                          for name, value in item.colors.items())
        for n, line in enumerate((title, colors)):
            y = max(82, min(result.shape[0]-8, y1 + 20 + n*20))
            x = max(0, min(x1, result.shape[1]-min(900, len(line)*9)))
            cv2.putText(result, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
            cv2.putText(result, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, pen, 1)
    return result
