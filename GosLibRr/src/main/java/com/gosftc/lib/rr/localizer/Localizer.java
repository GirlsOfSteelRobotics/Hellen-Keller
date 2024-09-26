package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;

import java.util.List;

public interface Localizer {
    Twist2dDual<Time> update();

    /**
     * Validates that the parameters have been updated. Will throw if not
     *
     * @throws RuntimeException if the parameters are invalid
     */
    void validateParams() throws RuntimeException;

    List<Encoder> getLeftEncoders();
    List<Encoder> getRightEncoders();

    List<Encoder> getParallelEncoders();
    List<Encoder> getPerpendicularEncoders();
}
