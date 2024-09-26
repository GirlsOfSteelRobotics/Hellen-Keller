package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.gosftc.lib.rr.messages.ThreeDeadWheelInputsMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        // y position of the first parallel encoder (in tick units)
        public final double par0YTicks;

        // y position of the second parallel encoder (in tick units)
        public final double par1YTicks;

        // x position of the perpendicular encoder (in tick units)
        public final double perpXTicks;

        public double inPerTick;

        public Params(double par0YTicks, double par1YTicks, double perpXTicks, double inPerTick) {
            this.par0YTicks = par0YTicks;
            this.par1YTicks = par1YTicks;
            this.perpXTicks = perpXTicks;
            this.inPerTick = inPerTick;
        }
    }

    private final Params params;

    private final Encoder par0, par1, perp;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    public ThreeDeadWheelLocalizer(OverflowEncoder par0, OverflowEncoder par1, OverflowEncoder perp, Params params) {
        this.params = params;

        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", params);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (params.par0YTicks * par1PosDelta - params.par1YTicks * par0PosDelta) / (params.par0YTicks - params.par1YTicks),
                                (params.par0YTicks * par1PosVel.velocity - params.par1YTicks * par0PosVel.velocity) / (params.par0YTicks - params.par1YTicks),
                        }).times(params.inPerTick),
                        new DualNum<Time>(new double[] {
                                (params.perpXTicks / (params.par0YTicks - params.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (params.perpXTicks / (params.par0YTicks - params.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(params.inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (params.par0YTicks - params.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (params.par0YTicks - params.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }

    @Override
    public void validateParams() throws RuntimeException {
        if (params.perpXTicks == 0 && params.par0YTicks == 0 && params.par1YTicks == 1) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
    }
    @Override
    public List<Encoder> getLeftEncoders() {
        return new ArrayList<>();
    }

    @Override
    public List<Encoder> getRightEncoders() {
        return new ArrayList<>();
    }

    @Override
    public List<Encoder> getParallelEncoders() {
        return Arrays.asList(par0, par1);
    }

    @Override
    public List<Encoder> getPerpendicularEncoders() {
        return Collections.singletonList(perp);
    }
}
