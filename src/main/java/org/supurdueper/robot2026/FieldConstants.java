// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.supurdueper.robot2026;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses have a blue alliance
 * origin.
 */
public class FieldConstants {

    public static final FieldType fieldType = FieldType.ANDYMARK;

    public static final double fieldLength =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
    public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of
    // starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.NO_BARGE;

    @Getter
    public enum AprilTagLayoutType {
        OFFICIAL("2025-official"),
        NO_BARGE("2025-no-barge"),
        BLUE_REEF("2025-blue-reef"),
        RED_REEF("2025-red-reef"),
        FIELD_BORDER("2025-field-border");

        AprilTagLayoutType(String name) {
            try {
                layout = new AprilTagFieldLayout(Path.of(
                        Filesystem.getDeployDirectory().getPath(),
                        "apriltags",
                        fieldType.getJsonFolder(),
                        name + ".json"));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            if (layout == null) {
                layoutString = "";
            } else {
                try {
                    layoutString = new ObjectMapper().writeValueAsString(layout);
                } catch (JsonProcessingException e) {
                    throw new RuntimeException(
                            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
                }
            }
        }

        private final AprilTagFieldLayout layout;
        private final String layoutString;
    }

    @RequiredArgsConstructor
    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        @Getter
        private final String jsonFolder;
    }
}
