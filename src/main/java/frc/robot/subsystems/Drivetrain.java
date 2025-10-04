package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages the robot drivetrain Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private final SparkMax lfMotor; // Left Front Drive Motor
    private final SparkMax lrMotor; // Left Rear Drive Motor
    private final SparkMax rfMotor; // Right Front Drive Motor
    private final SparkMax rrMotor; // Right Rear Drive Motor

    private final RelativeEncoder lEncoder; // Left Drive Encoder
    private final RelativeEncoder rEncoder; // Right Drive Encoder

    private double inputL;
    private double inputR;

    /**
     * Constructor
     * 
     * @param lfMotorID   Motor ID for the left front motor. This is the motor that
     *                    will be used for the left encoder.
     * @param lrMotorID   Motor ID for the left rear motor. This motor will be set
     *                    to follow the left front motor.
     * @param lfMotorID   Motor ID for the right front motor. This is the motor that
     *                    will be used for the right encoder.
     * @param lrMotorID   Motor ID for the right rear motor. This motor will be set
     *                    to follow the right front motor.
     * @param driveRatio  Drive motor gear ratio. All motors are assumed to have the
     *                    same gear ratio. This should be the number of turns of the
     *                    wheel per turn of the motor.
     * @param wheelRadius Drive wheel radius.
     */
    public Drivetrain(int lfMotorID, int lrMotorID, int rfMotorID, int rrMotorID, double driveRatio,
            Distance wheelRadius) {

        // Create Motors
        lfMotor = new SparkMax(lfMotorID, MotorType.kBrushless);
        lrMotor = new SparkMax(lrMotorID, MotorType.kBrushless);
        rfMotor = new SparkMax(rfMotorID, MotorType.kBrushless);
        rrMotor = new SparkMax(rrMotorID, MotorType.kBrushless);

        // Calculate the position conversion factor
        double distPerRev = wheelRadius.in(Inches) * Math.PI * driveRatio; // Distance traveled for one revolution of
                                                                           // the motor

        // Configure Left Front Motor
        SparkMaxConfig lfConfig = new SparkMaxConfig();

        lfConfig.encoder.positionConversionFactor(distPerRev); // Set the left encoder position conversion factor

        lfMotor.configure(lfConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Left Rear Motor
        SparkMaxConfig lrConfig = new SparkMaxConfig();

        lrConfig.follow(lfMotor); // Set left rear motor to follow left front motor

        lrMotor.configure(lrConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Right Front Motor
        SparkMaxConfig rfConfig = new SparkMaxConfig();

        rfConfig.encoder.positionConversionFactor(distPerRev); // Set the right encoder position conversion factor
        rfConfig.inverted(true);

        rfMotor.configure(rfConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Left Rear Motor
        SparkMaxConfig rrConfig = new SparkMaxConfig();

        rrConfig.follow(rfMotor); // Set right rear motor to follow right front motor

        rrMotor.configure(rrConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get drive encoders
        lEncoder = lfMotor.getEncoder();
        rEncoder = rfMotor.getEncoder();

        inputL = 0;
        inputR = 0;
    }

    /**
     * Sets the drive motor voltage
     * 
     * @param left  voltage to apply to the left drive motors
     * @param right voltage to apply to the right drive motors
     */
    public void setDrive(Voltage left, Voltage right) {
        inputL = left.magnitude();
        inputR = right.magnitude();
        lfMotor.setVoltage(left);
        rfMotor.setVoltage(right);
    }

    /**
     * Gets the average distance traveled by both sides of the drivetrain
     * 
     * @return average distance traveled by both sides of the drivetrain
     */
    public Distance getDistance() {
        double lDist = lEncoder.getPosition();
        double rDist = rEncoder.getPosition();
        double avgDist = (lDist + rDist) / 2;

        return Inches.of(avgDist);
    }

    /**
     * Resets the drive distance to zero
     */
    public void resetDistance() {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
    }

    /**
     * Drive command factory
     * @param left  left drive voltage supplier
     * @param right right drive voltage supplier
     * @return new drive command
     */
    public Command getDriveCmd(Supplier<Voltage> left, Supplier<Voltage> right) {
        return this.runEnd(
                () -> setDrive(left.get(), right.get()),
                () -> setDrive(Volts.zero(), Volts.zero()));
    }

    /**
     * Generates a command to drive forward for a certain distance
     * 
     * @param dist    distance to travel
     * @param voltage voltage to set to the motors
     * @return new command to drive the robot until it reaches a given distance
     */
    public Command getDriveDistCmd(Distance dist, Voltage voltage) {
        return Commands.deadline(Commands.waitUntil(() -> getDistance().gte(dist)), // Waits until the robot has reached
                                                                                    // a certain distance
                this.startEnd( // Sets the motors to a voltage at the start and turns them off at the end
                        () -> setDrive(voltage, voltage), () -> setDrive(Volts.zero(), Volts.zero())));
    }

    /**
     * Turns the robot for a given amount of time with a set voltage
     * 
     * @param time    time to turn
     * @param voltage voltage to set to the motors. Positive voltage will cause
     *                counter clockwise turning.
     * @return new command to turn the robot for a given amount of time
     */
    public Command getTurnTimeCmd(Time time, Voltage voltage) {
        return Commands.deadline(Commands.waitTime(time), // Waits for a certain amount of time
                this.startEnd( // Sets the drive motors to equal but opposite voltages at the starts and turns
                               // them off a the end
                        () -> setDrive(voltage, voltage.unaryMinus()), () -> setDrive(Volts.zero(), Volts.zero())));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Values", inputL);
        SmartDashboard.putNumber("Right Values", inputR);
    }
}
