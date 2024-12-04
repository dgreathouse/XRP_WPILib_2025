// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class g {
    public static class ARM{

    }
    public static class OI {
        public static final int driverControllerPort = 0;
        public static CommandPS5Controller driverController = new CommandPS5Controller(driverControllerPort);
        public static final double driverControllerDeadband = 0.02;
        public static final Trigger DRIVER_RESET_YAW = driverController.triangle();

    }
    public static class ROBOT{
        public static Drivetrain drive = new Drivetrain();
        public static XRPRangefinder rangeFinder = new XRPRangefinder();
        public static XRPReflectanceSensor m_lineSensor = new XRPReflectanceSensor();
        public static XRPGyro gyro = new XRPGyro();
        public static Arm arm = new Arm();
        public static double currentAngle = 0.0;
        public static XRPOnBoardIO boardIO = new XRPOnBoardIO();
    }
    public static class DRIVETRAIN{  private static final double GEAR_RATIO =
        (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
        public static final double WHEEL_DIAMETER_inch = 2.3622;
        public static final double MOTOR_CNTS_PER_SHAFT_REV = 12.0;
        public static final double WHEEL_COUNTS_PER_REV = MOTOR_CNTS_PER_SHAFT_REV * GEAR_RATIO;// 585;
        public static WheelSpeeds wheelSpeeds = new WheelSpeeds();
        public static final double TURN_KP = 0.008;
        public static final double TURN_KI = 0.01;
        public static final double TURN_KD = 0.0;
    }
}
