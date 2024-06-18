/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcGamepad;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * Allow setting which robot (if multiple) to use.
     * NoRobot is useful for when only testing Vision.
     */
    public enum RobotType
    {
        CenterStageRobot,
        NoRobot
    } // RobotType

    /**
     * This class contains robot preferences. It controls enabling/disabling of various robot features.
     */
    public static class Preferences
    {
        // Global config
        public static RobotType robotType = RobotType.CenterStageRobot;
        public static boolean inCompetition = false;

        // Miscellaneous
        public static boolean useTraceLog = true;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useBlinkin = false;
        public static boolean useBatteryMonitor = false;
        @SuppressWarnings("ConstantConditions")
        public static boolean doStatusUpdate = !inCompetition;
        // Vision
        public static boolean useWebCam = true;
        public static boolean hasWebCam2 = false;
        public static boolean tuneColorBlobVision = false;
        public static boolean useAprilTagVision = false;
        public static boolean useColorBlobVision = false;
        @SuppressWarnings("ConstantConditions")
        public static boolean showVisionView = !inCompetition;
        public static boolean showVisionStat = true;
        // Drive Base
        @SuppressWarnings("ConstantConditions")
        public static boolean useExternalOdometry = robotType == RobotType.CenterStageRobot;
        // Subsystems
        @SuppressWarnings("ConstantConditions")
        public static boolean useSubsystems = robotType == RobotType.CenterStageRobot;
        public static boolean useIntake = false;
        public static boolean hasCollectedPixelsSensor = false;
        public static boolean useArm = false;
        public static boolean useArmRotator = false;
        public static boolean useArmExtender = false;
        public static boolean useClaw = false;
        public static boolean usePixelMover = false;
        public static boolean useLauncher = false;
        public static boolean useLifter = false;
    } // class Preferences

    public static final String ROBOT_NAME                       = "Robotxxxx"; // @TODO Set Robot Name
    public static final RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.UP;  // @TODO Ensure hub direction is correct
    public static final RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection =
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD; // @TODO Ensure hub direction is correct
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc99943";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/tracelogs";
    public static final String STEERING_CALIBRATION_DATA_FILE   = "SteerCalibration.txt";


    // ------------------------------------------------------------------------------------------ //
    // ----------------------------------- Robot Hardware Name ---------------------------------- //
    // ------------------------------------------------------------------------------------------ //
    // Drivebase
    public static final String HWNAME_LEFT_FRONT_WHEEL          = "frontL";
    public static final String HWNAME_RIGHT_FRONT_WHEEL         = "frontR";
    public static final String HWNAME_LEFT_BACK_WHEEL           = "backL";
    public static final String HWNAME_RIGHT_BACK_WHEEL          = "backR";

    // Subsystems
    // @TODO Check Validity
    public static final String HWNAME_INTAKE                    = "intake";
    public static final String HWNAME_ARM_ROTATOR               = "arm rotator";
    public static final String HWNAME_ARM_EXTENDER              = "arm extender";
    public static final String HWNAME_CLAW                      = "claw";
    public static final String HWNAME_PIXEL_MOVER               = "pixel mover";
    public static final String HWNAME_LAUNCHER                  = "launcher";
    public static final String HWNAME_LIFTER                    = "lifter";


    // Sensors
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_WEBCAM_FRONT              = "Webcam Front";
    public static final String HWNAME_WEBCAM_BACK               = "Webcam Back";

    // Miscellaneous
    public static final String HWNAME_BLINKIN                   = "blinkin";


    // ------------------------------------------------------------------------------------------ //
    // ------------------------------------ Field dimensions ------------------------------------ //
    // ------------------------------------------------------------------------------------------ //
    public static final double FULL_FIELD_INCHES                = 141.24;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = FULL_FIELD_INCHES/6.0;


    // ------------------------------------------------------------------------------------------ //
    // ------------------------------------ Robot dimensions ------------------------------------ //
    // ------------------------------------------------------------------------------------------ //
    // TODO: Need to measure the Robot Dimensions
    public static final double ROBOT_LENGTH                     = 17.0;
    public static final double ROBOT_WIDTH                      = 17.0;
    public static final double DRIVE_BASE_LENGTH                = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
    public static final double DRIVE_BASE_WIDTH                 = 16.0;

    // --------------------------------- Game related locations --------------------------------- //
    // ----- Autonomous -----
    // Starting Position (in inches)
    public static final double STARTPOS_AUDIENCE_X              = -1.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_BACKSTAGE_X             = 0.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_BLUE_Y                  = HALF_FIELD_INCHES - ROBOT_LENGTH / 2.0;
    public static final TrcPose2D STARTPOS_BLUE_AUDIENCE        = new TrcPose2D(
            STARTPOS_AUDIENCE_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_BACKSTAGE       = new TrcPose2D(
            STARTPOS_BACKSTAGE_X, STARTPOS_BLUE_Y, 180.0);
    // Parking Position (in tile units) @TODO Check to ensure effectiveness
    public static final double PARKPOS_X                        = 2.6;
    public static final double PARKPOS_BLUE_CORNER_Y            = 2.5;
    public static final double PARKPOS_BLUE_CENTER_Y            = 0.5;
    public static final TrcPose2D PARKPOS_BLUE_CORNER           = new TrcPose2D(
            PARKPOS_X, PARKPOS_BLUE_CORNER_Y, -90.0);
    public static final TrcPose2D PARKPOS_BLUE_CENTER           = new TrcPose2D(
            PARKPOS_X, PARKPOS_BLUE_CENTER_Y, -90.0);

    // Spike Mark locations to place the pixel (in tile units)
    public static final double SPIKE_MARK_ANGLE_OFFSET          = 90.0;
    public static final double AUDIENCE_SPIKES_X                = -1.5;
    public static final double BACKSTAGE_SPIKES_X               = 0.5;
    public static final double BLUE_SPIKES_Y                    = 1.5;
    public static final TrcPose2D[] BLUE_AUDIENCE_SPIKE_MARKS   = new TrcPose2D[] {
            new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y, 180.0 - SPIKE_MARK_ANGLE_OFFSET),
            new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y, 180.0),
            new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y, 180.0 + SPIKE_MARK_ANGLE_OFFSET)
    };
    public static final TrcPose2D[] BLUE_BACKSTAGE_SPIKE_MARKS  = new TrcPose2D[] {
            new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0 - SPIKE_MARK_ANGLE_OFFSET),
            new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0),
            new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0 + SPIKE_MARK_ANGLE_OFFSET)
    };

    // ----- April Tags -----
    public static final int[] BLUE_BACKDROP_APRILTAGS           = new int[]{1, 2, 3};
    public static final int[] RED_BACKDROP_APRILTAGS            = new int[]{4, 5, 6};
    // AprilTag locations to place the pixel in inches.
    // DO NOT CHANGE the AprilTag location numbers. They are from the AprilTag metadata.
    public static final double APRILTAG_BACKDROP_X              = 60.25;
    public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
    public static final double BACKDROP_APRILTAG_DELTA_Y        = 6.0;
    // All AprilTags are at the height of 4.0-inch except for AprilTag 7 and 10 which are at the height of 5.5-inch.
    public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
            new TrcPose2D(APRILTAG_BACKDROP_X, 41.41, 90.0),        // TagId 1
            new TrcPose2D(APRILTAG_BACKDROP_X, 35.41, 90.0),        // TagId 2
            new TrcPose2D(APRILTAG_BACKDROP_X, 29.41, 90.0),        // TagId 3
            new TrcPose2D(APRILTAG_BACKDROP_X, -29.41, 90.0),       // TagId 4
            new TrcPose2D(APRILTAG_BACKDROP_X, -35.41, 90.0),       // TagId 5
            new TrcPose2D(APRILTAG_BACKDROP_X, -41.41, 90.0),       // TagId 6
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -40.63, -90.0), // TagId 7
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -35.13, -90.0), // TagId 8
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 35.13, -90.0),  // TagId 9
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 40.63, -90.0)   // TagId 10
    };

    // ------------------------------------------------------------------------------------------ //
    // ------------------------------------ Vision subsystem ------------------------------------ //
    // ------------------------------------------------------------------------------------------ //
    // Camera image size
    public static final int CAM_IMAGE_WIDTH                     = 640;
    public static final int CAM_IMAGE_HEIGHT                    = 480;
    public static final OpenCvCameraRotation CAM_ORIENTATION    = OpenCvCameraRotation.UPRIGHT;

    // TODO: Need to measure the camera position offsets.
    // Front Camera location on robot
    public static final double FRONTCAM_X_OFFSET                = 0.0;
    public static final double FRONTCAM_Y_OFFSET                = -(ROBOT_LENGTH/2.0 - 2.5);
    public static final double FRONTCAM_Z_OFFSET                = 9.75;
    public static final TrcPose2D FRONTCAM_POSE                 = new TrcPose2D(
            FRONTCAM_X_OFFSET, FRONTCAM_Y_OFFSET, 0.0);
    // Back Camera location on robot
    public static final double BACKCAM_X_OFFSET                 = 0.0;
    public static final double BACKCAM_Y_OFFSET                 = (ROBOT_LENGTH/2.0 - 2.125);
    public static final double BACKCAM_Z_OFFSET                 = 6.25;
    public static final TrcPose2D BACKCAM_POSE                  = new TrcPose2D(
            BACKCAM_X_OFFSET, BACKCAM_Y_OFFSET, 180.0);

    // Measurement unit: pixels @TODO Tune Values
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = CAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = CAM_IMAGE_HEIGHT - 1;
    // Measurement unit: inches @TODO Check Accuracy
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH/2.0 + FRONTCAM_Y_OFFSET;

    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);


    // ------------------------------------------------------------------------------------------ //
    // ------------------------------------- Motor odometry ------------------------------------- //
    // ------------------------------------------------------------------------------------------ //
    // https://www.andymark.com/products/neverest-classic-40-gearmotor?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6OkNhdGFsb2c6OkNhdGVnb3J5LzViYjYxOGI0YmM2ZjZkNmRlMWU2OWZkZg
    public static final double NEVEREST_40_ENCODER_PPR      = 280.0;
    public static final double NEVEREST_40_RPM              = 160.0;
    public static final double NEVEREST_40_MAX_VELOCITY_PPS = NEVEREST_40_ENCODER_PPR*NEVEREST_40_RPM/60.0; // 746.67 pps
    // https://www.andymark.com/products/neverest-classic-60-gearmotor?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6OkNhdGFsb2c6OkNhdGVnb3J5LzViYjYxOGI0YmM2ZjZkNmRlMWU2OWZkZg
    public static final double NEVEREST_60_ENCODER_PPR      = 420.0;
    public static final double NEVEREST_60_RPM              = 105.0;
    public static final double NEVEREST_60_MAX_VELOCITY_PPS = NEVEREST_60_ENCODER_PPR*NEVEREST_60_RPM/60.0; // 735.00 pps

    // ------------------------------------------------------------------------------------------ //
    // ----------------------------------- DriveBase subsystem ---------------------------------- //
    // ------------------------------------------------------------------------------------------ //
    public static DriveOrientation DEF_DRIVE_ORIENTATION        = DriveOrientation.FIELD;

    public static final boolean DRIVE_WHEEL_BRAKE_MODE_ON       = true;
    public static final double TURN_POWER_LIMIT                 = 0.5;
    public static final double DRIVE_POWER_SCALE_SLOW           = 0.5;
    public static final double DRIVE_POWER_SCALE_NORMAL         = 1.0;
    public static final double TURN_POWER_SCALE_SLOW            = 0.5;
    public static final double TURN_POWER_SCALE_NORMAL          = 1.0;

    // Odometry Wheel @TODO Get Measurements
    public static final double ODWHEEL_DIAMETER                 = 35.0 * TrcUtil.INCHES_PER_MM;
    public static final double ODWHEEL_CPR                      = 4096.0;
    public static final double ODWHEEL_INCHES_PER_COUNT         = Math.PI*ODWHEEL_DIAMETER/ODWHEEL_CPR;
    // Scale = 0.00105687652708656383937269814237 inches/count
    public static final double Y_LEFT_ODWHEEL_X_OFFSET          = -144.0 * TrcUtil.INCHES_PER_MM;
    public static final double Y_LEFT_ODWHEEL_Y_OFFSET          = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double Y_RIGHT_ODWHEEL_X_OFFSET         = 144.0 * TrcUtil.INCHES_PER_MM;
    public static final double Y_RIGHT_ODWHEEL_Y_OFFSET         = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double X_ODWHEEL_X_OFFSET               = 0.0;
    public static final double X_ODWHEEL_Y_OFFSET               = -168.0 * TrcUtil.INCHES_PER_MM;
    public static final FtcGamepad.DriveMode ROBOT_DRIVE_MODE   = FtcGamepad.DriveMode.ARCADE_MODE;


    // ------------------------------------------------------------------------------------------ //
    // ------------------------------ Velocity controlled constants ----------------------------- //
    // ------------------------------------------------------------------------------------------ //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = NEVEREST_40_MAX_VELOCITY_PPS;

    // @TODO Set Values
    public static final TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.001, 0.0);
    public static final double X_POS_TOLERANCE                  = 1.0;
    public static final double X_POS_INCHES_PER_COUNT           = 0.01924724265461924299065420560748;
    public static final Double X_RAMP_RATE                      = null; // 10.0;

    public static final TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0);
    public static final double Y_POS_TOLERANCE                  = 1.0;
    public static final double Y_POS_INCHES_PER_COUNT           = 0.02166184604662450653409090909091;
    public static final Double Y_RAMP_RATE                      = null; // 10.0;

    public static final TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.02, 0.08, 0.003, 0.0, 30.0);
    public static final double TURN_TOLERANCE                   = 1.0;
    public static final Double TURN_RAMP_RATE                   = null; // 10.0;


    // ------------------------------------------------------------------------------------------ //
    // --------------------------------- Pure Pursuit parameters -------------------------------- //
    // ------------------------------------------------------------------------------------------ //
    // @TODO Set Values
    // No-Load max velocity (i.e. theoretical maximum)
    // goBilda 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY               = 23.0;     // To be measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 500.0;    // To be measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 1.0;
    public static final double PPD_POS_ERR_RATE_THRESHOLD       = 1.0;
    public static final double PPD_TURN_TOLERANCE               = 1.0;
    public static final double PPD_TURN_ERR_RATE_THRESHOLD      = 1.0;


    // ------------------------------------------------------------------------------------------ //
    // ---------------------------------- Subsystems Parameters --------------------------------- //
    // ------------------------------------------------------------------------------------------ //
    // ----- Intake -----
    // @TODO Add Intake Params

    // ----- Arm Rotator -----
    // @TODO Add Arm Rotator Params

    // ----- Arm Extender -----
    // @TODO Add Arm Extender Params

    // ----- Claw -----
    // @TODO Claw Params

    // ----- Pixel Mover -----
    // @TODO Pixel Mover Params

    // ----- Airplane Launcher -----
    // @TODO Airplane Launcher Params

    // ----- Lifter -----
    // @TODO Lifter Params

} // class RobotParams
