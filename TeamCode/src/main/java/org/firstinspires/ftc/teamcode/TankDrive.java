package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.gosftc.lib.rr.actions.TankFollowTrajectoryAction;
import com.gosftc.lib.rr.actions.TankTurnAction;
import com.gosftc.lib.rr.drive.TankDrivetrain;
import com.gosftc.lib.rr.localizer.Localizer;
import com.gosftc.lib.rr.localizer.TankDriveLocalizer;
import com.gosftc.lib.rr.params.PathProfileParams;
import com.gosftc.lib.rr.params.TankControlParams;
import com.gosftc.lib.rr.params.TurnProfileCommands;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;

@Config
public final class TankDrive {
    public static Params PARAMS = new Params();

    private static final TankKinematics KINEMATICS = new TankKinematics(Params.IN_PER_TICK * Params.TRACK_WIDTH_TICKS);

    private static final TurnConstraints DEFAULT_TURN_CONSTRAINTS = new TurnConstraints(
            PARAMS.turnProfileCommands.maxAngVel, -PARAMS.turnProfileCommands.maxAngAccel, PARAMS.turnProfileCommands.maxAngAccel);
    private static final VelConstraint DEFAULT_VEL_CONSTRAINT =
            new MinVelConstraint(Arrays.asList(
                    KINEMATICS.new WheelVelConstraint(PARAMS.pathProfileParams.maxWheelVel),
                    new AngularVelConstraint(PARAMS.turnProfileCommands.maxAngVel)
            ));
    private static final AccelConstraint DEFAULT_ACCEL_CONSTRAINT =
            new ProfileAccelConstraint(PARAMS.pathProfileParams.minProfileAccel, PARAMS.pathProfileParams.maxProfileAccel);

    public final LazyImu lazyImu;

    private final TankDrivetrain m_drive;


    @SuppressWarnings("PMD.FieldNamingConventions")
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        private static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        private static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        private static final double IN_PER_TICK = 0;
        private static final double TRACK_WIDTH_TICKS = 0;

        // feedforward parameters (in tick units)
        private static final double KS = 0;
        private static final double KV = 0;
        private static final double KA = 0;

        // path profile parameters (in inches)
        private static final double MAX_WHEEL_VEL = 50;
        private static final double MIN_PROFILE_ACCEL = -30;
        private static final double MAX_PROFILE_ACCEL = 50;

        // turn profile parameters (in radians)
        private static final double MAX_ANG_VEL = Math.PI; // shared with path
        private static final double MAX_ANG_ACCEL = Math.PI;

        // path controller gains
        private static final double RAMSETE_ZETA = 0.7; // in the range (0, 1)
        private static final double RAMSETE_B_BAR = 2.0; // positive

        // turn controller gains
        private static final double TURN_GAIN = 0.0;
        private static final double TURN_VEL_GAIN = 0.0;

        public PathProfileParams pathProfileParams = new PathProfileParams(MAX_WHEEL_VEL, MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);
        public TurnProfileCommands turnProfileCommands = new TurnProfileCommands(MAX_ANG_VEL, MAX_ANG_ACCEL);
        public TankControlParams controllerParams = new TankControlParams(
                KS, KV, KA,
                TURN_GAIN, TURN_VEL_GAIN,
                RAMSETE_ZETA, RAMSETE_B_BAR);

        public double getInchesPerTick() {
            return IN_PER_TICK;
        }
    }

    public TankDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        List<DcMotorEx> leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        List<DcMotorEx>rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse motor directions if needed
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Localizer localizer = new TankDriveLocalizer(leftMotors, rightMotors, KINEMATICS, Params.IN_PER_TICK);

        m_drive = new TankDrivetrain(leftMotors, rightMotors, voltageSensor, localizer, pose);

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    public Localizer getLocalizer() {
        return m_drive.getLocalizer();
    }

    public TankDrivetrain getDrive() {
        return m_drive;
    }

    public void setDrivePowers(PoseVelocity2d poseVelocity2d) {
        m_drive.setDrivePowers(poseVelocity2d);
    }

    public void updatePoseEstimate() {
        m_drive.updatePoseEstimate();
    }

    public Pose2d getPose() {
        return m_drive.getPose();
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                (timeTurn -> new TankTurnAction(m_drive, KINEMATICS, PARAMS.controllerParams, Params.IN_PER_TICK, timeTurn)),
                (timeTrajectory -> new TankFollowTrajectoryAction(m_drive, KINEMATICS, PARAMS.controllerParams, Params.IN_PER_TICK, timeTrajectory)),
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                DEFAULT_TURN_CONSTRAINTS,
                DEFAULT_VEL_CONSTRAINT, DEFAULT_ACCEL_CONSTRAINT
        );
    }
}
