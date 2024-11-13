package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.PinpointEncoder;
import com.gosftc.lib.rr.localizer.Localizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.rr_tuning.mecanum.MecanumLocalizationTest;
import org.firstinspires.ftc.teamcode.opmodes.rr_tuning.mecanum.MecanumManualFeedbackTuner;
import org.firstinspires.ftc.teamcode.opmodes.rr_tuning.mecanum.MecanumSplineTest;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    @SuppressWarnings("PMD.ExcessiveMethodLength")
    public static void register(OpModeManager manager) {
        if (DISABLED) {
            return;
        }

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            dvf = hardwareMap -> {
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                GoBildaPinpointDriverRR localizer = md.getLocalizer();
                List<Encoder> leftEncs = new ArrayList<>();
                List<Encoder> rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>();
                List<Encoder> perpEncs = new ArrayList<>();

                return new DriveView(
                    DriveType.MECANUM,
                        MecanumDrive.PARAMS.getInchesPerTick(),
                        MecanumDrive.PARAMS.pathProfileParams.maxWheelVel,
                        MecanumDrive.PARAMS.pathProfileParams.minProfileAccel,
                        MecanumDrive.PARAMS.pathProfileParams.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        md.getDrive().getLeftMotors(),
                        md.getDrive().getRightMotors(),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        md.m_lazyImu,
                        md.getDrive().voltageSensor,
                        () -> MecanumDrive.PARAMS.controllerParams.createFeedForward(MecanumDrive.PARAMS.getInchesPerTick())
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        List<Class<?>> configClazzes = Arrays.asList(
                AngularRampLogger.class,
                ForwardRampLogger.class,
                LateralRampLogger.class,
                ManualFeedforwardTuner.class,
                MecanumMotorDirectionDebugger.class
        );

        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            manager.register(metaForClass(MecanumManualFeedbackTuner.class), MecanumManualFeedbackTuner.class);
            manager.register(metaForClass(MecanumSplineTest.class), MecanumSplineTest.class);
            manager.register(metaForClass(MecanumLocalizationTest.class), MecanumLocalizationTest.class);

            configClazzes.add(MecanumManualFeedbackTuner.class);
        }

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : configClazzes) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
