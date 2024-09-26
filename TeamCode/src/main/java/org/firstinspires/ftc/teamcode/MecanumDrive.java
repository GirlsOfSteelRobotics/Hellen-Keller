package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.gosftc.lib.rr.actions.MecanumFollowTrajectoryAction;
import com.gosftc.lib.rr.actions.MecanumTurnAction;
import com.gosftc.lib.rr.drive.MecanumDrivetrain;
import com.gosftc.lib.rr.localizer.Localizer;
import com.gosftc.lib.rr.localizer.MecanumDriveLocalizer;
import com.gosftc.lib.rr.params.MecanumControlParams;
import com.gosftc.lib.rr.params.PathProfileParams;
import com.gosftc.lib.rr.params.TurnProfileCommands;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.Math;
import java.util.Arrays;

@Config
public final class MecanumDrive {
    public static Params PARAMS = new Params();

    private static final MecanumKinematics KINEMATICS = new MecanumKinematics(
            Params.IN_PER_TICK * Params.TRACK_WIDTH_TICKS, Params.IN_PER_TICK / Params.LATERAL_IN_PER_TICK);

    private static final TurnConstraints DEFAULT_TURN_CONSTRAINTS = new TurnConstraints(
            PARAMS.turnProfileCommands.maxAngVel, -PARAMS.turnProfileCommands.maxAngAccel, PARAMS.turnProfileCommands.maxAngAccel);
    private static final VelConstraint DEFAULT_VEL_CONSTRAINT =
            new MinVelConstraint(Arrays.asList(
                    KINEMATICS.new WheelVelConstraint(PARAMS.pathProfileParams.maxWheelVel),
                    new AngularVelConstraint(PARAMS.turnProfileCommands.maxAngVel)
            ));
    private static final AccelConstraint DEFAULT_ACCEL_CONSTRAINT =
            new ProfileAccelConstraint(PARAMS.pathProfileParams.minProfileAccel, PARAMS.pathProfileParams.maxProfileAccel);

    public final LazyImu m_lazyImu;

    private final MecanumDrivetrain m_drive;

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
        private static final double IN_PER_TICK = 1;
        private static final double LATERAL_IN_PER_TICK = IN_PER_TICK;
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
        private static final double AXIAL_GAIN = 0.0;
        private static final double LATERAL_GAIN = 0.0;
        private static final double HEADING_GAIN = 0.0; // shared with turn

        private static final double AXIAL_VEL_GAIN = 0.0;
        private static final double LATERAL_VEL_GAIN = 0.0;
        private static final double HEADING_VEL_GAIN = 0.0; // shared with turn

        public PathProfileParams pathProfileParams = new PathProfileParams(MAX_WHEEL_VEL, MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);
        public TurnProfileCommands turnProfileCommands = new TurnProfileCommands(MAX_ANG_VEL, MAX_ANG_ACCEL);
        public MecanumControlParams controllerParams = new MecanumControlParams(
                KS, KV, KA,
                AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN);

        public double getInchesPerTick() {
            return IN_PER_TICK;
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        m_lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Localizer localizer = new MecanumDriveLocalizer(leftFront, leftBack, rightBack, rightFront, m_lazyImu, KINEMATICS, Params.IN_PER_TICK);

        m_drive = new MecanumDrivetrain(leftFront, leftBack, rightBack, rightFront, pose, voltageSensor, localizer);
        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public Localizer getLocalizer() {
        return m_drive.getLocalizer();
    }

    public MecanumDrivetrain getDrive() {
        return m_drive;
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        m_drive.setDrivePowers(powers);
    }

    public void updatePoseEstimate() {
        m_drive.updatePoseEstimate();
    }

    public Pose2d getPose() {
        return m_drive.getPose();
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                (timeTurn -> new MecanumTurnAction(m_drive, KINEMATICS, PARAMS.controllerParams, PARAMS.IN_PER_TICK, timeTurn)),
                (timeTrajectory -> new MecanumFollowTrajectoryAction(m_drive, KINEMATICS, PARAMS.controllerParams, PARAMS.IN_PER_TICK, timeTrajectory)),
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
