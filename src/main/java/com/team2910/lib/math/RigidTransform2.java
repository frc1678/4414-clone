package com.team2910.lib.math;

import com.team2910.lib.util.Interpolable;

import java.io.Serializable;
import java.util.Objects;

/**
 * A rigid transform is a type of transformation that represents both a translation and a rotation.
 *
 * @since 0.2
 */
public final class RigidTransform2 implements Serializable, Interpolable<RigidTransform2>, com.team254.lib.util.Interpolable<RigidTransform2> {
    public static final RigidTransform2 ZERO = new RigidTransform2(Vector2.ZERO, Rotation2.ZERO);

    public static RigidTransform2 identity(){
        return ZERO;
    }

    private static final long serialVersionUID = 1701732846641084965L;

    private final static double kEps = 1E-9;

    /**
     * The translation of the transform
     * @since 0.2
     */
    public final Vector2 translation;

    /**
     * The rotation of the transform
     * @since 0.2
     */
    public final Rotation2 rotation;

    /** 
     * Empty RigidTransform2
     * 
     */
    public RigidTransform2() {
        translation = new Vector2();
        rotation = new Rotation2();
    }

    /**
     * Create a new rigid transform from a translation and a rotation.
     *
     * @param translation The translation
     * @param rotation    The rotation
     * @since 0.2
     */
    public RigidTransform2(Vector2 translation, Rotation2 rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    private static Vector2 intersectionInternal(RigidTransform2 a, RigidTransform2 b) {
        final double t = ((a.translation.x - b.translation.x) * b.rotation.tan + b.translation.y - a.translation.y) /
                (a.rotation.sin - a.rotation.cos * b.rotation.tan);
        return a.translation.add(Vector2.fromAngle(a.rotation).scale(t));
    }

    /**
     * Adds the effects of this rigid transform and another rigid transform together.
     *
     * @param other The rigid transform to apply
     * @return A rigid transform with the effects of both this and another rigid transform
     * @since 0.2
     */
    public RigidTransform2 transformBy(RigidTransform2 other) {
        return new RigidTransform2(translation.add(other.translation.rotateBy(rotation)), rotation.rotateBy(other.rotation));
    }

    /**
     * Gets the rigid transform that would undo the effects of this transform.
     *
     * @return The inverse of this transform
     * @since 0.2
     */
    public RigidTransform2 inverse() {
        Rotation2 inverseRotation = rotation.inverse();
        return new RigidTransform2(translation.inverse().rotateBy(inverseRotation), inverseRotation);
    }

    /**
     * Gets the point of intersection between this rigid transform and another.
     *
     * @param other The other rigid transform
     * @return The point of intersection between the two transforms
     * @since 0.2
     */
    public Vector2 intersection(RigidTransform2 other) {
        if (rotation.isParallel(other.rotation)) {
            return new Vector2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        if (Math.abs(rotation.cos) < Math.abs(other.rotation.cos)) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    public static RigidTransform2 exp(final com.team254.lib.geometry.Twist2d twist2d) {
        double sin_theta = Math.sin(twist2d.dtheta);
        double cos_theta = Math.cos(twist2d.dtheta);
        double s, c;
        if (Math.abs(twist2d.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * twist2d.dtheta * twist2d.dtheta;
            c = .5 * twist2d.dtheta;
        } else {
            s = sin_theta / twist2d.dtheta;
            c = (1.0 - cos_theta) / twist2d.dtheta;
        }
        return new RigidTransform2(new Vector2(twist2d.dx * s - twist2d.dy * c, twist2d.dx * c + twist2d.dy * s),
            new Rotation2(cos_theta, sin_theta, false));
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "{T: " + translation + ", R: " + rotation + "}";
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof RigidTransform2)) {
            return false;
        }

        RigidTransform2 other = (RigidTransform2) obj;

        return translation.equals(other.translation) && rotation.equals(other.rotation);
    }

    @Override
    public int hashCode() {
        return Objects.hash(translation, rotation);
    }

    @Override
    public RigidTransform2 interpolate(RigidTransform2 other, double t) {
        return new RigidTransform2(translation.interpolate(other.translation, t),
                rotation.interpolate(other.rotation, t));
    }

    public Vector2 getTranslation() {
        return translation;
    }

    public static RigidTransform2 fromTranslation(final Vector2 translation) {
        return new RigidTransform2(translation, new Rotation2());

    }

    public Rotation2 getRotation() {
        return rotation;
    }
}