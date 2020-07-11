package org.firstinspires.ftc.teamcode.purepursuit;

import java.util.Objects;

public class Pose2d {
    float x = 0;
    float y = 0;
    float heading = 0;
    public Pose2d(){};
    public Pose2d(float x, float y, float heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose2d(double x, double y, double heading) {
        this.x = (float)x;
        this.y = (float)y;
        this.heading = (float)heading;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public void setPose(float x, float y, float heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void setHeading(float heading) {
        this.heading = heading;
    }

    public float getHeading() {
        return heading;
    }

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void sumX(float x) {
        this.x += x;
    }

    public void sumY(float y) {
        this.y += y;
    }

    @Override
    public String toString() {
        return "Pose2d{" +
                "x=" + x +
                ", y=" + y +
                ", heading=" + heading +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Pose2d pose2d = (Pose2d) o;
        return Objects.equals(x, pose2d.x) &&
                Objects.equals(y, pose2d.y) &&
                Objects.equals(heading, pose2d.heading);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, heading);
    }
}
