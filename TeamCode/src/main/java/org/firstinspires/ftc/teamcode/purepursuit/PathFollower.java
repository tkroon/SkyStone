package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.dashboard.canvas.Canvas;

public class PathFollower {
    // follower coordinates
    private Pose2d pose;

    // follower speed
    private float speed;
    private float radius = 1;

    public PathFollower(float x, float y, float speed, float heading) {
        this.pose = new Pose2d(x, y, heading);
        this.speed = speed;
    }

    /**
     * Moves the follower towards a point by the follower's speed.
     *
     * @param x The x value of the coordinate towards which to move.
     * @param y The y value of the coordinate towards which to move.
     */
    public void moveFollowerTowardsPoint(float x, float y, float heading) {
        // move the point to origin (the follower's coordinates)
        float offsetX = x - pose.getX();
        float offsetY = y - pose.getY();

        // normalize the vector
        float distanceToPoint = (float) Math.sqrt(offsetX * offsetX + offsetY * offsetY);
        float normalizedX = offsetX / distanceToPoint;
        float normalizedY = offsetY / distanceToPoint;

        // move towards the point at a certain speed
        pose.sumX(normalizedX * speed);
        pose.sumY(normalizedY * speed);
        pose.setHeading(heading);
    }

    public void draw(Canvas canvas) {
        canvas.setStroke("blue");
        canvas.fillCircle(pose.getX(), pose.getY(),radius);
    }

    /**
     * Returns the coordinates of the follower.
     *
     * @return A float[2], with arr[0] being the x value and arr[1] being the y value.
     */
    public Pose2d getFollowerPosition() {
        return pose;
    }
}