/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import java.util.Arrays;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRobotBattery;
import TrcFtcLib.ftclib.FtcServo;
import teamcode.drivebases.MecanumDrive;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.subsystems.Arm;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.subsystems.Claw;
import teamcode.subsystems.Intake;
import teamcode.vision.Vision;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private static final String moduleName = Robot.class.getSimpleName();
    private static final double STATUS_UPDATE_INTERVAL = 0.1;   // 100 milli-sec

    // Global objects
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    private static double nextStatusUpdateTime = 0.0;

    // Vision subsystems
    public Vision vision;

    // Sensors and indicators
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;

    // Subsystems
    public RobotDrive robotDrive;
    public Intake intake;
    public Arm arm; // Includes Arm Rotator, Arm Extender, and Wrist
    public Claw claw;
    public FtcServo pixelMover;
    public FtcServo launcher;
    public FtcServo lifter;

    // Auto Tasks
    public TaskAutoPlacePixel placePixelTask;

    // Performance tracker
    private final long[] totalElapsedTime = new long[6];
    private long loopCount;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        nextStatusUpdateTime = TrcTimer.getCurrentTime();

        speak("Init starting");

        // Initialize vision subsystems
        if (RobotParams.Preferences.tuneColorBlobVision ||
            RobotParams.Preferences.useAprilTagVision ||
            RobotParams.Preferences.useColorBlobVision)
        {
            vision = new Vision(this);
        }

        // If robotType is NoRobot, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
        {
            // Create and initialize sensors and indicators
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new BlinkinLEDs(RobotParams.HWNAME_BLINKIN);
            }

            // Create and initialize battery monitor
            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }

            // Create and initialize RobotDrive
            robotDrive = new MecanumDrive();

            // Create and initialize other subsystems
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake(RobotParams.HWNAME_INTAKE, this);
                }

                if (RobotParams.Preferences.useArm)
                {
                    arm = new Arm();
                }

                if (RobotParams.Preferences.useClaw)
                {
                    claw = new Claw(RobotParams.HWNAME_CLAW);
                }

                if (RobotParams.Preferences.usePixelMover)
                {
                    pixelMover = new FtcServo(RobotParams.HWNAME_PIXEL_MOVER);
                }

                if (RobotParams.Preferences.useLauncher)
                {
                    launcher = new FtcServo(RobotParams.HWNAME_LAUNCHER);
                }

                if (RobotParams.Preferences.useLifter)
                {
                    lifter = new FtcServo(RobotParams.HWNAME_LIFTER);
                }

                // Auto Tasks
                placePixelTask = new TaskAutoPlacePixel("PlacePixelTask", this);
            }
        }
        speak("Init complete");
    } // Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    } // toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);

                // Performance counter - could provide insight if autonomous wasn't performing as expected
                // Disabled if inCompetition
                robotDrive.gyro.setElapsedTimerEnabled(!RobotParams.Preferences.inCompetition);
            }

            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                // If an autonomous was ran, the ending position of the robot would be saved
                if (endOfAutoRobotPose != null)
                {
                    // Set robot position
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);

        // Initialize performance trackers
        Arrays.fill(totalElapsedTime, 0L);
        loopCount = 0;
    } // startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        if (loopCount > 0)
        {
            globalTracer.traceInfo(
                    moduleName,
                    "Update Status average elapsed times:\n" +
                            "DriveBase=%.6fs\n" +
                            " Elevator=%.6fs\n" +
                            "      Arm=%.6fs\n" +
                            "    Wrist=%.6fs\n" +
                            "   Intake=%.6fs\n" +
                            "PixelTray=%.6fs\n",
                    totalElapsedTime[0]/1000000000.0/loopCount,     //DriveBase
                    totalElapsedTime[1]/1000000000.0/loopCount,     //Elevator
                    totalElapsedTime[2]/1000000000.0/loopCount,     //Arm
                    totalElapsedTime[3]/1000000000.0/loopCount,     //Wrist
                    totalElapsedTime[4]/1000000000.0/loopCount,     //Intake
                    totalElapsedTime[4]/1000000000.0/loopCount);    //PixelTray
        }

        // Print all performance counters if there are any
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);

        // Disable vision
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.redBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RedBlobVision.");
                vision.setRedBlobVisionEnabled(false);
            }

            if (vision.blueBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling BlueBlobVision.");
                vision.setBlueBlobVisionEnabled(false);
            }

            vision.close();
       }

        // Disable Driving
        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent TeleOp run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }

            // Disable odometry
            robotDrive.driveBase.setOdometryEnabled(false);

            // Disable gyro task
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    } // stopMode


    /**
     * This method update all subsystem status on the dashboard.
     */
    public void updateStatus()
    {
        if (TrcTimer.getCurrentTime() > nextStatusUpdateTime)
        {
            int lineNum = 2; // Line num on dashboard to print at
            long startNanoTime; // Starting time of each updateStatus loop

            nextStatusUpdateTime += STATUS_UPDATE_INTERVAL;
            startNanoTime = TrcTimer.getNanoTime();

            // Drivebase Position
            if (robotDrive != null)
            {
                dashboard.displayPrintf(lineNum++, "DriveBase: Pose=%s", robotDrive.driveBase.getFieldPosition());
            }
            totalElapsedTime[0] += TrcTimer.getNanoTime() - startNanoTime;

            // ----- Subsystem status -----
            // Status of Intake
            if (intake != null)
            {
                startNanoTime = TrcTimer.getNanoTime();
                dashboard.displayPrintf(
                        lineNum++, "Intake: power=%.1f, sensor=%f, has2Pixels=%s",
                        intake.getIntakeMotor().getPower(), intake.getDistance(), intake.hasTwoPixels());
                totalElapsedTime[1] += TrcTimer.getNanoTime() - startNanoTime;
            }

            // Status of Arm and all of its subsystems
            if (arm != null)
            {
//                startNanoTime = TrcTimer.getNanoTime();
//                if (elevatorArm.elevator != null)
//                {
//                    dashboard.displayPrintf(
//                            lineNum++, "Elevator: power=%.3f, pos=%.1f, target=%.1f, lowerLimitSw=%s",
//                            elevatorArm.elevator.getPower(), elevatorArm.elevator.getPosition(),
//                            elevatorArm.elevator.getPidTarget(), elevatorArm.elevator.isLowerLimitSwitchActive());
//                }
//                totalElapsedTime[2] += TrcTimer.getNanoTime() - startNanoTime;
//
//                startNanoTime = TrcTimer.getNanoTime();
//                if (elevatorArm.arm != null)
//                {
//                    dashboard.displayPrintf(
//                            lineNum++, "Arm: power=%.3f, pos=%.1f/%f, target=%.1f",
//                            elevatorArm.arm.getPower(), elevatorArm.arm.getPosition(),
//                            elevatorArm.arm.getEncoderRawPosition(), elevatorArm.arm.getPidTarget());
//                }
//                totalElapsedTime[3] += TrcTimer.getNanoTime() - startNanoTime;
//
//                startNanoTime = TrcTimer.getNanoTime();
//                if (elevatorArm.wrist != null)
//                {
//                    if (elevatorArm.wristSensor != null)
//                    {
//                        dashboard.displayPrintf(
//                                lineNum++, "Wrist: pos=%.1f, distance=%.1f",
//                                elevatorArm.wrist.getPosition(), elevatorArm.wristGetDistance());
//                    }
//                    else
//                    {
//                        dashboard.displayPrintf(lineNum++, "Wrist: pos=%.1f", elevatorArm.wrist.getPosition());
//                    }
//                }
//                totalElapsedTime[4] += TrcTimer.getNanoTime() - startNanoTime;
            }

            // Status of Claw
            if (claw != null)
            {
                startNanoTime = TrcTimer.getNanoTime();
                dashboard.displayPrintf(
                        lineNum++, "Claw: lowerClawOpened=%s, upperClawOpened=%s",
                        claw.isLowerClawOpened(), claw.isUpperClawOpened());
                totalElapsedTime[5] += TrcTimer.getNanoTime() - startNanoTime;
            }

            // Increment loop count by 1
            loopCount++;
        }
    } // updateStatus


    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (arm != null)
        {
            arm.zeroCalibrate(owner);
        }
    } // zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    } // zeroCalibrate


    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        robotDrive.driveBase.setFieldPosition(
            adjustPoseByAlliance(
                autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                        RobotParams.STARTPOS_BLUE_AUDIENCE: RobotParams.STARTPOS_BLUE_BACKSTAGE,
                autoChoices.alliance, false));
    } // setRobotStartPosition

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
            double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = -newPose.y;
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.FULL_TILE_INCHES;
            newPose.y *= RobotParams.FULL_TILE_INCHES;
        }

        return newPose;
    } // adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, true);
    } // adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    } // adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, true);
    } // adjustPoseByAlliance


    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

} // class Robot
