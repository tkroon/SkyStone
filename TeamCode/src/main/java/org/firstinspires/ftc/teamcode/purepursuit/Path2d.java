package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.ArrayList;
import java.util.List;

public class Path2d {
    private List<float[]> path;

    public Path2d () {
        path = new ArrayList<>();
        defaultPath();
    }

    public float[] get(int index) {
        return path.get(index);
    }

    public int size() {
        return path.size();
    }

    public void draw(Canvas canvas) {
        // draw the path
        canvas.setStroke("black");
        for (int i = 0; i < path.size(); i++) {
            float[] point = path.get(i);
            canvas.fillCircle(point[0], point[1], 1);
            // if it isn't the first point, connect it to its predecessor
            if (i > 0) {
                float[] previousPoint = path.get(i - 1);
                canvas.setStrokeWidth(1);
                canvas.strokeLine(point[0], point[1], previousPoint[0], previousPoint[1]);
            }
        }
    }

    private void defaultPath() {
        path.add(new float[]{0, 0});
        path.add(new float[]{0, 24});
        path.add(new float[]{24, 24});
        path.add(new float[]{48, 24});
        path.add(new float[]{48, 0});
        path.add(new float[]{24,-48});
        path.add(new float[]{-48,24});
    }
}
