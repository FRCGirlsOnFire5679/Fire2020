package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder extends AnalogInput {
    private double angleOffset;
    private boolean reversed = false;

    /**
     * @param channel     analog channel of the AbsoluteEncoder
     * @param angleOffset offset acquired by the getCalibration method
     */
    public AbsoluteEncoder(int channel, double angleOffset) {
        this(channel, angleOffset, false);
    }

    /**
     * @param channel     analog channel of the AbsoluteEncoder
     * @param angleOffset offset acquired by the getCalibration method
     * @param reversed    boolean indicating the AbsoluteEncoder's angle needs to be
     *                    reversed
     */
    public AbsoluteEncoder(int channel, double angleOffset, boolean reversed) {
        super(channel);
        this.angleOffset = Math.toRadians(angleOffset);
        this.reversed = reversed;
    }

    /**
     * Get the current angle of the AbsoluteEncoder in radians between 0 and 2pi.
     * 
     * @return current angle in radians
     */
    public double getAngle() {
        double angle = (getVoltage() - 0.2) * (2 * Math.PI) / 4.6;
        if (reversed)
            angle *= -1;
        return normalizeAngle(angle - angleOffset);
    }

    /**
     * Convert an angle to its equivalent between 0 and 2pi
     * 
     * @param angle angle in radians
     * @return normalized angle between 0 and 2pi
     */
    private double normalizeAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle < 0)
            angle += 2 * Math.PI;
        return angle;
    }

    /**
     * When the AbsoluteEncoder is pointing at the desired angle, call this method
     * to get the angleOffset such that the current angle will be zero.
     * 
     * @return angle offset to calibrate AbsoluteEncoder
     */
    public double getCalibration() {
        return normalizeAngle(getAngle() + angleOffset);
    }

    /**
     * Angle returned to a PID controller.
     */
    public double pidGet() {
        return getAngle();
    }

    public void setReverseDirection(boolean reversed) {
        this.reversed = reversed;
    }

}