#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/ai_vision.hpp"
#include "pros_pose/pros_pose.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-19, -18}, pros::MotorGearset::blue); // left motor group - LB: 19 (reversed), LF: 18 (reversed)
pros::MotorGroup rightMotors({8, 9}, pros::MotorGearset::blue); // right motor group - RF: 8, RB: 9
pros::MotorGroup liftMotors({-4, 14}, pros::MotorGearset::red); //lift motor grroup - RL: 4 (reversed), LL: 14
pros::MotorGroup rollerMotors({2, -1}); 						//roller motor group - LC: 2, RC: 1 (reversed)

// Inertial Sensor on port 20
pros::Imu imu(20);
pros::AIVision vision(5); // Vision sensor on port 5
pros::Distance dist(17); // Distance sensor on port 17
// Experimental values derived from your measured image edges.
// Not a full camera calibration.
ProsPose tagPoseEstimator(
    110.0,    // Trial fx
    114.0,    // Trial fy
    160.0,    // Keep current cx
    120.0,    // Keep current cy
    0.028575  // Measured tag side: 1.125 inches (real tag is 1.125 inches, but printed is .875 inches)
);

pros::AIVision::Color redColor = {   //color definition for red
    .id = 2,
    .red = 255,
    .green = 0,
    .blue = 0,
    .hue_range = 15,
    .saturation_range = 0.5
};

pros::AIVision::Color blueColor = {   //color definition for blue
    .id = 3,
    .red = 0,
    .green = 0,
    .blue = 255,
    .hue_range = 15,
    .saturation_range = 0.5
};

//Motors
pros::Motor ClawMotor(3); // Claw motor - port 3
pros::Motor ToggleMotor(6); // Toggle motor - port 6

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 16,
pros::Rotation horizontalEnc(16);
// vertical tracking wheel encoder. Rotation sensor, port 15,
pros::Rotation verticalEnc(15);
// horizontal tracking wheel. 2" diameter, 0" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);
// vertical tracking wheel. 2" diameter, 2.25" offset, right of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.25);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15.25, // 15.25 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              300, // drivetrain rpm is 300
                              5 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(12,  // proportional gain (kP)
                                            0,   // integral gain (kI)
                                            28,  // derivative gain (kD)
                                            0,   // anti windup
                                            0,   // small error range, in inches
                                            0,   // small error range timeout, in ms
                                            0,   // large error range, in inches
                                            0,   // large error range timeout, in ms
                                            0    // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(7,   // proportional gain (kP)
                                             0,   // integral gain (kI)
                                             25,  // derivative gain (kD)
                                             0,   // anti windup
                                             0,   // small error range, in degrees
                                             0,   // small error range timeout, in ms
                                             0,   // large error range, in degrees
                                             0,   // large error range timeout, in ms
                                             0    // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Holds an X and Y value in our local map.
struct FieldXY {
    double x;
    double y;
};


constexpr double DISTANCE_SENSOR_OFFSET = 2.0; //distance from the front of the robot to the tracking center


// ID 2 was shown in your earlier Brain photo.
// Change this if the red base uses a different ID.
constexpr int RED_BASE_TAG_ID = 2;
constexpr double RED_BASE_X = 23.11; //pos of red base in inches, x is right, y is forward
constexpr double RED_BASE_Y = 46.66; //pos of red base in inches, x is right, y is forward
constexpr double RED_BASE_APPROACH_DISTANCE = 3.0; //distance from the red base to stop at in inches

constexpr double RED_BLUE_STACK_X = 46.66;
constexpr double RED_BLUE_STACK_Y = 70.20;
constexpr double DROP_OFF_BASE_X = 23.11;
constexpr double DROP_OFF_BASE_Y = 93.75;

constexpr double APPR_HEADING = 90.0; //approach heading for red base in degress

constexpr double MIN_TRUSTED_TAG_DISTANCE = 3.0; //minimum distance in inches to trust the tag reading
constexpr double MAX_TRUSTED_TAG_DISTANCE = 15.0; //maximum distance in inches to trust the tag reading

constexpr double MAX_ERROR = 2.0; //maximum error in inches to trust the tag reading

constexpr double TAG_CONSIST_TOL = 0.3;
constexpr int REQ_CONSEC_READINGS = 3; //number of consecutive readings required to trust the tag reading
int CONSEC_TAG_READINGS = 0; 
double tagCandidateX = 0.0;
double tagCandidateY = 0.0; 

// pros_pose returns translation in meters.
constexpr double METERS_TO_INCHES = 39.37007874015748;
constexpr double DEG_TO_RAD = 0.017453292519943295;

// Camera position relative to the robot's center.
constexpr double CAMERA_FORWARD_IN = 6.5;
constexpr double CAMERA_RIGHT_IN = 7.625;
constexpr double AUTON_START_FIELD_X = 0.0;
constexpr double AUTON_START_FIELD_Y = 70.20;

// ---- V5 distance-sensor final docking --------------------------------------------------
// The AI/vision camera is mounted off-center and loses its AprilTag during the final
// claw-align turn, so the robot can't perfectly line the claw up with the base/stack.
// A V5 Distance sensor mounted front-center, low to the ground, can still see the object
// the whole way in. dockByDistance() drives forward slowly and stops at a fixed sensor
// gap, giving a repeatable final stop regardless of how the odometry has drifted.
constexpr double MM_PER_INCH = 25.4;            // sensor reports mm; targets use inches
constexpr double RED_BASE_DOCK_DIST_IN = 3.0;   // final sensor->object gap at the red base
constexpr double STACK_DOCK_DIST_IN = 3.0;      // final sensor->object gap at the red/blue stack
constexpr double DROP_OFF_DOCK_DIST_IN = 3.0;   // final sensor->object gap at the drop-off base
constexpr double DOCK_TOLERANCE_IN = 0.15;      // stop once within this of the target gap
constexpr int DOCK_CRAWL_SPEED = 12;            // 0-127: slow + smooth for a precise stop
constexpr int DOCK_FACE_SPEED = 20;             // 0-127: gentle final facing turn
constexpr double DOCK_FACE_TURN_DEG = 8.0;      // skip the facing turn if already within this
constexpr double DOCK_MAX_TRAVEL_IN = 14.0;     // hard safety cap on total forward crawl
constexpr int DOCK_MAX_FACE_MS = 1000;          // max time for the facing turn
constexpr int DOCK_MAX_CRAWL_MS = 2000;         // max time for the forward crawl
// --------------------------------------------------------------------------------------------

FieldXY fieldToAutonCoordinates(double fieldX, double fieldY) {
    return {
        fieldX - AUTON_START_FIELD_X,
        fieldY - AUTON_START_FIELD_Y
    };
}

// Signed angular difference (target - current), wrapped into the (-180, 180] range, in degrees.
static double wrapDeg(double target, double current) {
    double d = std::fmod(target - current, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

bool AprilTagLocalization = true; //set a boolean to determine whether to use AprilTag localization or not
bool AprilTagCorrectionAccepted = false; //set a boolean to determine whether to accept the AprilTag correction or not
bool AprilTagReading = false;

double AprilTagRobotX = 0.0; //set a double to hold the x position derived from the AprilTag
double AprilTagRobotY = 0.0; //set a double to hold the y position derived from the AprilTag

double CamForward = 0.0; //set a double to hold the forward distance from the camera to the AprilTag
double CamRight = 0.0; //set a double to hold the right distance from the camera to the AprilTag

double aprilTagForwardIn;

// Find the robot-to-tag offset, expressed in map directions.
bool tagOffsetInField(
    const ProsPoseResult& result,
    double headingDeg,
    FieldXY& offset
) {
    // Reject invalid or unusable measurements.
    if (!result.is_valid ||
        !std::isfinite(result.x) ||
        !std::isfinite(result.y) ||
        !std::isfinite(result.z) ||
        !std::isfinite(headingDeg) ||
        result.z <= 0.0) {
        return false;
    }

    // Change camera-relative meters into robot-relative inches.
    const double rightIn =
        result.x * METERS_TO_INCHES + CAMERA_RIGHT_IN;

    const double forwardIn =
        result.z * METERS_TO_INCHES + CAMERA_FORWARD_IN;

    // Account for the direction the robot is facing.
    const double headingRad = headingDeg * DEG_TO_RAD;
    const double c = std::cos(headingRad);
    const double s = std::sin(headingRad);

    offset.x = rightIn * c + forwardIn * s;
    offset.y = -rightIn * s + forwardIn * c;

    return true;
}

void resetTagConsist() {
    CONSEC_TAG_READINGS = 0;
}

void updatePosefromVision(double visionX, double visionY) {
    AprilTagCorrectionAccepted = false;     // Reset the correction accepted flag
    const auto pose = chassis.getPose();    // Get the current pose of the robot
    if (!AprilTagLocalization) { // If AprilTag localization is disabled, return early
        resetTagConsist();
        return;
    }
    if (!std::isfinite(visionX) || 
        !std::isfinite(visionY) ||
        !std::isfinite(aprilTagForwardIn)) { // If the vision coordinates are not finite, return early
        resetTagConsist();
        return;
    }
    if (aprilTagForwardIn < MIN_TRUSTED_TAG_DISTANCE || 
        aprilTagForwardIn > MAX_TRUSTED_TAG_DISTANCE) { // If the forward distance is outside the trusted range, return early
        resetTagConsist();
        return;
    }
    const double errorX = std::fabs(visionX - pose.x); // Calculate the error in the X direction
    const double errorY = std::fabs(visionY - pose.y); // Calculate the error in the Y direction
    if (errorX > MAX_ERROR || errorY > MAX_ERROR) { // If the error is greater than the maximum allowed, return early
        resetTagConsist();
        return;
    }
    if (CONSEC_TAG_READINGS == 0) {
        tagCandidateX = visionX;
        tagCandidateY = visionY;
        CONSEC_TAG_READINGS = 1;
        return; 
    }
    const double candidateErrorX = std::fabs(visionX- tagCandidateX);
    const double candidateErrorY = std::fabs(visionY - tagCandidateY);
    if (candidateErrorX > TAG_CONSIST_TOL || candidateErrorY > TAG_CONSIST_TOL) {
        tagCandidateX = visionX;
        tagCandidateY = visionY;
        CONSEC_TAG_READINGS = 1;
        return; 
    }
    CONSEC_TAG_READINGS++;
    if (CONSEC_TAG_READINGS < REQ_CONSEC_READINGS) {
        return;
    }
    chassis.setPose(visionX, visionY, pose.theta); // Update the robot's pose with the vision coordinates
    AprilTagCorrectionAccepted = true; // Set the correction accepted flag to true
    resetTagConsist();
}

void proccessAprilTagLocalization(FieldXY& redTagPosition, bool& redTagMapped) {
    AprilTagCorrectionAccepted = false; //set a boolean to determine whether to accept the AprilTag correction or not
    AprilTagReading = false; //set a boolean to determine whether to use AprilTag reading or not

    //Read objects once per pass
        auto objects = vision.get_all_objects(); // get all objects from the vision sensor
        bool foundTarget = false; // create a variable to hold whether the target has been found
    //clear the LCD
        for (int line = 0; line < 8; line++) { // loop through the first 4 lines of the LCD
            pros::lcd::clear_line(line); // clear the LCD's lines one at a time
        }
        for (const auto& object : objects) { // loop through all the objects found by the vision sensor
            if (pros::AIVision::is_type(object, pros::AivisionDetectType::tag)== false) { // check if the object is a tag
                continue; // if the object is not a tag, skip to the next object
            }
            if (object.id != RED_BASE_TAG_ID) { // check if the tag is the correct tag
                continue; // if the tag is not correct, skip to the next object
            }
            
            foundTarget = true; // if the tag is found, set foundTarget to true
            const auto& tag = object.object.tag; // create a variable to hold the tag object
            
            double corners[4][2] = {   // create a variable to hold the corners of the tag
                {static_cast<double>(tag.x3), static_cast<double>(tag.y3)},
                {static_cast<double>(tag.x2), static_cast<double>(tag.y2)},
                {static_cast<double>(tag.x1), static_cast<double>(tag.y1)},
                {static_cast<double>(tag.x0), static_cast<double>(tag.y0)},
            };
            pros::lcd::print(0, "C0 %d,%d", tag.x0, tag.y0);
            pros::lcd::print(1, "C1 %d,%d", tag.x1, tag.y1);
            pros::lcd::print(2, "C2 %d,%d", tag.x2, tag.y2);
            pros::lcd::print(3, "C3 %d,%d", tag.x3, tag.y3);

            ProsPoseResult result = tagPoseEstimator.estimate(corners); // estimate the pose of the tag
            const auto pose = chassis.getPose(); // get the robot's current pose
            FieldXY offset = {}; // create a variable to hold the offset of the tag

            //pros::lcd::print(0, "Selected tag: %d", (int)object.id);
            pros::lcd::print(5, "Odom X: %.1f Odom Y: %.1f", pose.x, pose.y);
            pros::lcd::print(6, "Heading: %.1f", pose.theta);

            bool usableTranslation = tagOffsetInField(result, pose.theta, offset); // get the offset of the tag

            if (usableTranslation == true) {   // if the offset is usable, set the redTagPosition to the offset
                AprilTagReading= true;
                bool mappedThisFrame = false; // create a variable to hold whether the tag has been mapped this frame
                if (!redTagMapped) { // if the tag has not been mapped yet, set the redTagPosition to the offset
                    redTagPosition.x = pose.x + offset.x; // set the redTagPosition x to the robot's x position plus the offset x
                    redTagPosition.y = pose.y + offset.y; // set the redTagPosition y to the robot's y position plus the offset y
                    redTagMapped = true; // set redTagMapped to true
                    mappedThisFrame = true; // set mappedThisFrame to true
                }

                FieldXY visionRobotPos{}; // create a variable to hold the robot's position relative to the tag
                visionRobotPos.x = redTagPosition.x - offset.x; // set the visionRobotPos x to the redTagPosition x minus the offset x
                visionRobotPos.y = redTagPosition.y - offset.y; // set the visionRobotPos y to the redTagPosition y minus the offset y
                aprilTagForwardIn = result.z * METERS_TO_INCHES; // set the aprilTagCameraForwardIn to the result z times the conversion factor

                updatePosefromVision(visionRobotPos.x, visionRobotPos.y); // update the robot's pose with the vision coordinates
                printf("usable=%d accepted=%d count=%d\n",
                AprilTagReading,
                AprilTagCorrectionAccepted,
                CONSEC_TAG_READINGS);

                if (mappedThisFrame== true) { 
                    //pros::lcd::print(1, "First Reading Mapped");
                }
                else { 
                    //pros::lcd::print(1, "VISION TEST ONLY");
                }
                printf(
                "actual=?, camR=%.2f, camF=%.2f, yaw=%.1f, odom=(%.2f, %.2f, %.1f), vision=(%.2f, %.2f)\n",
                result.x * METERS_TO_INCHES,
                result.z * METERS_TO_INCHES,
                result.yaw,
                pose.x,
                pose.y,
                pose.theta,
                visionRobotPos.x,
                visionRobotPos.y
                );
                pros::lcd::print(4, "Cam R:%.1f F:%.1f", result.x * METERS_TO_INCHES, result.z * METERS_TO_INCHES);
                //pros::lcd::print(3, "Tag X:%.1f Y:%.1f", redTagPosition.x, redTagPosition.y);
                //pros::lcd::print(4, "Vision X:%.1f Y:%.1f", visionRobotPos.x, visionRobotPos.y);
               }
            else { // if the offset is not usable, print that the tag is not usable
                pros::lcd::print(1, "Tag Not Usable");
            }
            if (result.is_valid&&std::isfinite(result.yaw)) { // if the results are valid, print that the yaw is unused
                pros::lcd::print(7, "Yaw Unused: %.1f", result.yaw);
            }
            else { // if the results are not valid, print that yaw is invalid
                pros::lcd::print(7, "Yaw INVALID");
            }
        }
        if (!foundTarget) { // if the tag has not been mapped, print that the tag is not found
            pros::lcd::print(0, "Tag %d Not Found", RED_BASE_TAG_ID);
            if (!redTagMapped) { // if the tag has not been mapped, print that the tag is not mapped
                pros::lcd::print(1, "Waiting for reference");
            }
            else { // if the tag has been mapped, print that the tag is mapped
                pros::lcd::print(1, "Reference Mapped");
            }
        }
        if (!AprilTagReading) {
            resetTagConsist();
        }
}

void stopDrive() {
    leftMotors.move(0);
    rightMotors.move(0);
}

void useTagMode() {
    vision.disable_detection_types(
        pros::AivisionModeType::colors
    );

    vision.enable_detection_types(
        pros::AivisionModeType::tags
    );
}

void useStackColorMode() {
    AprilTagCorrectionAccepted = false;
    AprilTagReading = false;
    resetTagConsist();
    vision.disable_detection_types(
        pros::AivisionModeType::tags
    );

    vision.enable_detection_types(
        pros::AivisionModeType::colors
    );
}

struct StackColorTarget {
    double centerX;
    int height;
    int id;
};

bool findStackColorTarget(StackColorTarget& target) {
    auto detections = vision.get_all_objects();
    bool foundTarget = false;
    double largestArea = 0.0;

    for (const auto& detection : detections) {
        if (!pros::AIVision::is_type(
                detection, pros::AivisionDetectType::color)) {
            continue;
        }
        if (detection.id != redColor.id && detection.id != blueColor.id) {
            continue;
        }

        const auto& box = detection.object.color;
        const double area = static_cast<double>(box.width) * box.height;
        if (area > largestArea) {
            largestArea = area;
            target = {
                box.xoffset + box.width / 2.0,
                static_cast<int>(box.height),
                detection.id
            };
            foundTarget = true;
        }
    }

    return foundTarget;
}

/**
 * @brief Final, repeatable stop using the front-center V5 distance sensor.
 *
 * The vision camera is off-center and can lose the AprilTag during the final
 * claw-align turn, so odometry alone cannot line the claw up exactly. This
 * routine faces the object's odometry point and then crawls forward slowly,
 * stopping the instant the measured gap matches `gapIn` -- giving a consistent
 * stop every run regardless of odometry drift.
 *
 * @param label       name used for logging
 * @param targetX/Y   the object's auton-local coordinates (used to face it)
 * @param gapIn       desired final sensor->object gap, in inches
 * @param toleranceIn stopping tolerance around `gapIn`, in inches
 * @param maxMs       upper bound on the whole maneuver (caps the auton budget)
 * @return true if the sensor reported the gap was reached; false on timeout,
 *         no reading, or the safety travel cap (the robot is left stopped).
 */
bool dockByDistance(const char* label,
                    double targetX, double targetY,
                    double gapIn,
                    double toleranceIn,
                    uint32_t maxMs) {
    chassis.cancelAllMotions();
    stopDrive();

    const uint32_t startMs = pros::millis();
    auto timeLeft = [&]() -> uint32_t {
        const uint32_t now = pros::millis();
        const uint32_t deadline = startMs + maxMs;
        return now < deadline ? deadline - now : 0;
    };
    if (timeLeft() == 0) {
        printf("%s dock: no time left in auton; skipping.\n", label);
        return false;
    }

    // 1) Face the object so the front sensor can see it. Skip the turn if we are
    //    already lined up -- the distance sensor handles the fine forward stop.
    const auto facePose = chassis.getPose();
    const double targetHeadingDeg =
        std::atan2(targetX - facePose.x, targetY - facePose.y) / DEG_TO_RAD;
    if (std::fabs(wrapDeg(targetHeadingDeg, facePose.theta)) > DOCK_FACE_TURN_DEG) {
        const int faceMs = static_cast<int>(
            std::min<uint32_t>(static_cast<uint32_t>(DOCK_MAX_FACE_MS), timeLeft()));
        if (faceMs > 0) {
            chassis.turnToPoint(
                targetX, targetY, faceMs,
                {
                    .maxSpeed = DOCK_FACE_SPEED,
                    .minSpeed = DOCK_FACE_SPEED,
                    .earlyExitRange = 2.0
                },
                false
            );
        }
    }
    stopDrive();
    if (timeLeft() == 0) {
        return false;
    }

    // 2) Crawl forward slowly, stopping the instant the gap matches the target.
    const double targetMm = gapIn * MM_PER_INCH;
    const double tolMm = toleranceIn * MM_PER_INCH;
    const auto crawlStart = chassis.getPose();
    const uint32_t crawlStartMs = pros::millis();

    bool reached = false;
    int32_t lastReadingMm = 9999;
    while (timeLeft() > 0 && (pros::millis() - crawlStartMs) < DOCK_MAX_CRAWL_MS) {
        const int32_t readingMm = dist.get();
        // 9999 == no object detected; treat any non-sane reading as "far away".
        const bool sane = readingMm > 1 && readingMm < 9000;
        lastReadingMm = readingMm;
        const double gapMm = sane ? static_cast<double>(readingMm) : 10000.0;

        if (gapMm <= targetMm + tolMm) {
            reached = true;   // stopped at the desired, repeatable gap
            break;
        }

        // Safety: never crawl farther than a hard cap.
        const auto nowPose = chassis.getPose();
        const double traveledIn =
            std::hypot(nowPose.x - crawlStart.x, nowPose.y - crawlStart.y);
        if (traveledIn >= DOCK_MAX_TRAVEL_IN) {
            break;
        }

        leftMotors.move(DOCK_CRAWL_SPEED);
        rightMotors.move(DOCK_CRAWL_SPEED);
        pros::delay(std::min<uint32_t>(10, timeLeft()));
    }

    stopDrive();
    printf("%s dock: reached=%d reading=%d mm (target %.2f in)\n",
           label, reached, static_cast<int>(lastReadingMm), gapIn);
    return reached;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    vision.reset();
    const int32_t redColorResult = vision.set_color(redColor);
    const int32_t blueColorResult = vision.set_color(blueColor);
    if (redColorResult != PROS_SUCCESS || blueColorResult != PROS_SUCCESS) {
        printf(
            "Failed to configure stack colors: red=%d blue=%d\n",
            redColorResult,
            blueColorResult
        );
    }
    pros::delay(100); // Existing delay; not a verified readiness guarantee.

    // Keep this only if your physical tag is from this family.
    vision.set_tag_family(pros::AivisionTagFamily::tag_21H7);

    // Use tag detection rather than color detection for this test.
    vision.disable_detection_types(pros::AivisionModeType::colors);
    vision.enable_detection_types(pros::AivisionModeType::tags);
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

// The autonomous routine is driven by the RedApproachState machine declared inside
// autonomous() below. The previous global AutonPhase enum and currentPhase were
// unused, so they have been removed.

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // ---------- STARTUP ----------
    chassis.cancelAllMotions();
    stopDrive();
    // The autonomous coordinate origin is the marked start at field (0, 70.20).
    chassis.setPose(0, 0, 0);
    const uint32_t startTime = pros::millis();

    chassis.moveToPoint(0, 9.5, 1000, {}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 750, {}, false);
    liftMotors.move(64);
    pros::delay(1000);
    liftMotors.move_absolute(liftMotors.get_position(), 30);

    AprilTagLocalization = true;
    AprilTagReading = false;
    AprilTagCorrectionAccepted = false;
    resetTagConsist();
    useTagMode();

    // Local reference: keep these for the entire autonomous run.
    FieldXY redTagPosition = {};
    bool redTagMapped = false;

    // These states only cover the red-base approach.
    enum class RedApproachState {
        ACQUIRE_TAG,
        FALLBACK_FORWARD,
        FALLBACK_MOVING,
        PLAN_APPROACH,
        MOVE_TO_TARGET,
        MOVING,
        ALIGN_CLAW_WITH_RED_BASE,
        FINAL_RED_BASE_DOCK,     // distance-sensor final stop at the red base
        OPEN_RED_BASE,
        BACK_AWAY_FROM_RED_BASE,
        CONFIGURE_STACK_VISION,
        MOVE_TO_STACK,
        STACK_DOCK,             // distance-sensor final stop at the red/blue stack
        FIND_AND_PICK_UP_STACK,
        MOVE_TO_DROP_OFF,
        DROP_OFF_DOCK,          // distance-sensor final stop at the drop-off base
        RELEASE_STACK,
        DONE
    };

    RedApproachState state = RedApproachState::ACQUIRE_TAG;

    constexpr uint32_t ACQUIRE_TIMEOUT_MS = 4000;
    constexpr int MOTION_TIMEOUT_MS = 2500;
    constexpr int APPROACH_MOTION_TIMEOUT_MS = 4000;
    constexpr uint32_t TOTAL_TIMEOUT_MS = 15000;

    // The claw contact point is 7.5 inches forward of the tracking center.
    constexpr double STANDOFF_IN = 7.5;

    // Initial test only: cap the commanded target displacement.
    constexpr double MAX_APPROACH_TRAVEL_IN = 3.0;

    constexpr double TARGET_TOLERANCE_IN = 0.75;

    double targetX = 0.0;
    double targetY = 0.0;
    bool redBaseAtStandoff = false;
    double fallbackTargetX = 0.0;
    double fallbackTargetY = 0.0;
    bool stackCollected = false;
    bool stackDelivered = false;
    uint32_t acquisitionStartTime = pros::millis();
    //Calculating relative coordinates:
    const FieldXY stackTarget =
        fieldToAutonCoordinates(RED_BLUE_STACK_X, RED_BLUE_STACK_Y);
    const FieldXY dropTarget =
        fieldToAutonCoordinates(DROP_OFF_BASE_X, DROP_OFF_BASE_Y);

    auto remainingAutonMs = [&]() -> uint32_t {
        const uint32_t elapsed = pros::millis() - startTime;
        return elapsed < TOTAL_TIMEOUT_MS ? TOTAL_TIMEOUT_MS - elapsed : 0;
    };

    auto moveToStandoff = [&](const char* label,
                              const FieldXY& target,
                              double standoff) {
        const uint32_t remaining = remainingAutonMs();
        if (remaining == 0) {
            return false;
        }

        const auto pose = chassis.getPose();
        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance)) {
            printf("Cannot plan %s approach: invalid pose.\n", label);
            return false;
        }
        if (distance <= standoff) {
            // Already at (or closer than) the standoff point: there is nothing to
            // drive, so this counts as a success and the next step proceeds.
            printf("%s already within standoff; skipping move.\n", label);
            return true;
        }

        const double approachX = target.x - dx / distance * standoff;
        const double approachY = target.y - dy / distance * standoff;
        const int motionTimeout = static_cast<int>(
            std::min<uint32_t>(8000, remaining)
        );

        printf(
            "%s coordinate: field=(%.2f, %.2f), local=(%.2f, %.2f), "
            "waypoint=(%.2f, %.2f)\n",
            label,
            target.x + AUTON_START_FIELD_X,
            target.y + AUTON_START_FIELD_Y,
            target.x,
            target.y,
            approachX,
            approachY
        );

        chassis.cancelAllMotions();
        stopDrive();
        chassis.moveToPoint(
            approachX,
            approachY,
            motionTimeout,
            {.maxSpeed = 40, .minSpeed = 20, .earlyExitRange = 1.0},
            false
        );

        const auto finalPose = chassis.getPose();
        const double waypointError = std::hypot(
            approachX - finalPose.x,
            approachY - finalPose.y
        );
        printf("%s waypoint odometry error: %.2f in\n", label, waypointError);
        return std::isfinite(waypointError) && waypointError <= 6.0;
    };

    // ---------- TIMED AUTONOMOUS LOOP ----------
    while (pros::millis() - startTime < TOTAL_TIMEOUT_MS) {
        switch (state) {

            case RedApproachState::ACQUIRE_TAG: {
                // Stay stopped while collecting consistent observations.
                proccessAprilTagLocalization(
                    redTagPosition,
                    redTagMapped
                );

                if (redTagMapped && AprilTagCorrectionAccepted) {
                    printf("Tag 2 acquired. Planning approach.\n");
                    state = RedApproachState::PLAN_APPROACH;
                }
                else if (pros::millis() - acquisitionStartTime >=
                         ACQUIRE_TIMEOUT_MS) {
                    printf("Tag acquisition timed out; moving forward 1 inch.\n");
                    state = RedApproachState::FALLBACK_FORWARD;
                }

                break;
            }

            case RedApproachState::FALLBACK_FORWARD: {
                AprilTagLocalization = false;
                resetTagConsist();
                const uint32_t remaining = remainingAutonMs();
                if (remaining == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                const auto pose = chassis.getPose();
                const double headingRad = pose.theta * DEG_TO_RAD;
                fallbackTargetX = pose.x + std::sin(headingRad);
                fallbackTargetY = pose.y + std::cos(headingRad);

                chassis.moveToPoint(
                    fallbackTargetX,
                    fallbackTargetY,
                    static_cast<int>(std::min<uint32_t>(
                        MOTION_TIMEOUT_MS,
                        remaining
                    )),
                    {.maxSpeed = 40, .minSpeed = 20, .earlyExitRange = 0.1},
                    false
                );
                state = RedApproachState::FALLBACK_MOVING;
                break;
            }

            case RedApproachState::FALLBACK_MOVING: {
                if (!chassis.isInMotion()) {
                    const auto pose = chassis.getPose();
                    const double fallbackError = std::hypot(
                        fallbackTargetX - pose.x,
                        fallbackTargetY - pose.y
                    );

                    if (!std::isfinite(fallbackError) ||
                        fallbackError > TARGET_TOLERANCE_IN) {
                        printf(
                            "Fallback move incomplete; retrying tag acquisition. "
                            "Odom error: %.2f in\n",
                            fallbackError
                        );
                    }

                    AprilTagLocalization = true;
                    AprilTagReading = false;
                    AprilTagCorrectionAccepted = false;
                    resetTagConsist();
                    useTagMode();
                    acquisitionStartTime = pros::millis();
                    state = RedApproachState::ACQUIRE_TAG;
                }
                break;
            }

            case RedApproachState::PLAN_APPROACH: {
                // Do not apply the stationary consistency filter
                // while the robot is moving.
                AprilTagLocalization = false;
                resetTagConsist();

                const auto pose = chassis.getPose();

                const double dx = redTagPosition.x - pose.x;
                const double dy = redTagPosition.y - pose.y;
                const double distanceToTag = std::hypot(dx, dy);

                if (!std::isfinite(distanceToTag) ||
                    distanceToTag <= 0.0) {
                    printf("Invalid tag distance; reacquiring AprilTag.\n");
                    redTagMapped = false;
                    redTagPosition = {};
                    AprilTagLocalization = true;
                    AprilTagReading = false;
                    AprilTagCorrectionAccepted = false;
                    resetTagConsist();
                    acquisitionStartTime = pros::millis();
                    state = RedApproachState::ACQUIRE_TAG;
                    break;
                }

                if (distanceToTag <= STANDOFF_IN) {
                    printf(
                        "At claw standoff: center-to-tag distance %.2f in.\n",
                        distanceToTag
                    );
                    redBaseAtStandoff = true;
                    state = RedApproachState::ALIGN_CLAW_WITH_RED_BASE;
                    break;
                }

                // const double travel = std::fmin(
                //     MAX_APPROACH_TRAVEL_IN,
                //     distanceToTag - STANDOFF_IN
                // );
                const double travel = MAX_APPROACH_TRAVEL_IN;

                targetX =
                    pose.x + (dx / distanceToTag) * travel;

                targetY =
                    pose.y + (dy / distanceToTag) * travel;

                printf(
                    "APPROACH: target=(%.2f, %.2f), "
                    "commanded displacement=%.2f in\n",
                    targetX,
                    targetY,
                    travel
                );
                printf(
                    "Moving from (%.2f, %.2f) to (%.2f, %.2f)\n",
                    pose.x,
                    pose.y,
                    targetX,
                    targetY
                );

                redBaseAtStandoff = false;
                state = RedApproachState::ALIGN_CLAW_WITH_RED_BASE;
                break;
            }

            case RedApproachState::MOVE_TO_TARGET: {
                chassis.cancelAllMotions();
                stopDrive();
                const uint32_t remaining = remainingAutonMs();
                if (remaining == 0) {
                    state = RedApproachState::DONE;
                    break;
                }
                chassis.moveToPoint(
                    targetX,
                    targetY,
                    static_cast<int>(std::min<uint32_t>(
                        APPROACH_MOTION_TIMEOUT_MS,
                        remaining
                    )),
                    {.maxSpeed = 90, .minSpeed = 64, .earlyExitRange = 0.7},
                    false
                );
                state = RedApproachState::MOVING;
                break;
            }

            case RedApproachState::MOVING: {
                // LemLib controls the motors during this state.
                if (!chassis.isInMotion()) {
                    const auto pose = chassis.getPose();

                    const double remainingError = std::hypot(
                        targetX - pose.x,
                        targetY - pose.y
                    );
                    const double remainingTagDistance = std::hypot(
                        redTagPosition.x - pose.x,
                        redTagPosition.y - pose.y
                    );

                    if (!std::isfinite(remainingError) ||
                        !std::isfinite(remainingTagDistance)) {
                        printf("Invalid odometry after motion; reacquiring AprilTag.\n");
                        redTagMapped = false;
                        redTagPosition = {};
                        AprilTagLocalization = true;
                        AprilTagReading = false;
                        AprilTagCorrectionAccepted = false;
                        resetTagConsist();
                        acquisitionStartTime = pros::millis();
                        state = RedApproachState::ACQUIRE_TAG;
                    }
                    else if (remainingError <= TARGET_TOLERANCE_IN) {
                        if (remainingTagDistance <= STANDOFF_IN) {
                            printf(
                                "At claw standoff. Tag distance: %.2f in\n",
                                remainingTagDistance
                            );
                            state = RedApproachState::PLAN_APPROACH;
                        }
                        else {
                            printf(
                                "Approach step reached; remaining tag distance: "
                                "%.2f in. Replanning.\n",
                                remainingTagDistance
                            );
                            state = RedApproachState::PLAN_APPROACH;
                        }
                    }
                    else {
                        printf(
                            "Approach step incomplete; replanning. "
                            "Odom error: %.2f in\n",
                            remainingError
                        );

                        state = RedApproachState::PLAN_APPROACH;
                    }
                }

                break;
            }

            case RedApproachState::ALIGN_CLAW_WITH_RED_BASE: {
                AprilTagLocalization = false;
                resetTagConsist();
                const uint32_t remaining = remainingAutonMs();
                if (remaining == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                chassis.turnToPoint(
                    redTagPosition.x,
                    redTagPosition.y,
                    static_cast<int>(std::min<uint32_t>(1500, remaining)),
                    {.maxSpeed = 50, .minSpeed = 20, .earlyExitRange = 2.0},
                    false
                );
                state = remainingAutonMs() > 0
                    ? (redBaseAtStandoff
                        ? RedApproachState::FINAL_RED_BASE_DOCK
                        : RedApproachState::MOVE_TO_TARGET)
                    : RedApproachState::DONE;
                break;
            }

            case RedApproachState::FINAL_RED_BASE_DOCK: {
                // Final, repeatable stop at the red base using the front-center
                // distance sensor: face the base, then crawl to its target gap.
                dockByDistance(
                    "red base",
                    redTagPosition.x, redTagPosition.y,
                    RED_BASE_DOCK_DIST_IN,
                    DOCK_TOLERANCE_IN,
                    remainingAutonMs()
                );
                state = RedApproachState::OPEN_RED_BASE;
                break;
            }

            case RedApproachState::OPEN_RED_BASE: {
                const uint32_t actionTime =
                    std::min<uint32_t>(500, remainingAutonMs());
                if (actionTime == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                ClawMotor.move(-64);
                pros::delay(actionTime);
                ClawMotor.move(0);
                state = remainingAutonMs() > 0
                    ? RedApproachState::BACK_AWAY_FROM_RED_BASE
                    : RedApproachState::DONE;
                break;
            }

            case RedApproachState::BACK_AWAY_FROM_RED_BASE: {
                const uint32_t remaining = remainingAutonMs();
                if (remaining == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                const auto pose = chassis.getPose();
                const double headingRad = pose.theta * DEG_TO_RAD;
                const double reverseTargetX =
                    pose.x - 2.0 * std::sin(headingRad);
                const double reverseTargetY =
                    pose.y - 2.0 * std::cos(headingRad);
                chassis.moveToPoint(
                    reverseTargetX,
                    reverseTargetY,
                    static_cast<int>(std::min<uint32_t>(
                        MOTION_TIMEOUT_MS,
                        remaining
                    )),
                    {.maxSpeed = 40},
                    false
                );
                state = remainingAutonMs() > 0
                    ? RedApproachState::CONFIGURE_STACK_VISION
                    : RedApproachState::DONE;
                break;
            }

            case RedApproachState::CONFIGURE_STACK_VISION:
                AprilTagLocalization = false;
                AprilTagReading = false;
                AprilTagCorrectionAccepted = false;
                resetTagConsist();
                useStackColorMode();
                state = RedApproachState::MOVE_TO_STACK;
                break;

            case RedApproachState::MOVE_TO_STACK:
                // Drive toward the stack; STACK_DOCK then makes the precise final
                // stop with the distance sensor, so an imperfect odometry approach
                // is not fatal.
                moveToStandoff("Red/blue stack", stackTarget, 12.0);
                state = RedApproachState::STACK_DOCK;
                break;

            case RedApproachState::STACK_DOCK: {
                // Final, repeatable stop at the red/blue stack via the distance sensor.
                dockByDistance(
                    "red/blue stack",
                    stackTarget.x, stackTarget.y,
                    STACK_DOCK_DIST_IN,
                    DOCK_TOLERANCE_IN,
                    remainingAutonMs()
                );
                state = RedApproachState::FIND_AND_PICK_UP_STACK;
                break;
            }

            case RedApproachState::FIND_AND_PICK_UP_STACK: {
                constexpr double STACK_CROSSCHECK_RADIUS_IN = 20.0;
                constexpr int STACK_GRAB_HEIGHT_PX = 90;
                if (remainingAutonMs() == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                StackColorTarget target{};
                if (!findStackColorTarget(target)) {
                    stopDrive();
                    pros::delay(std::min<uint32_t>(20, remainingAutonMs()));
                    break;
                }

                const auto pose = chassis.getPose();
                const double coordinateError = std::hypot(
                    stackTarget.x - pose.x,
                    stackTarget.y - pose.y
                );
                if (!std::isfinite(coordinateError) ||
                    coordinateError > STACK_CROSSCHECK_RADIUS_IN) {
                    stopDrive();
                    printf(
                        "Vision target rejected: stack coordinate error "
                        "%.2f in exceeds %.2f in.\n",
                        coordinateError,
                        STACK_CROSSCHECK_RADIUS_IN
                    );
                    state = RedApproachState::DONE;
                    break;
                }

                if (target.height >= STACK_GRAB_HEIGHT_PX) {
                    stopDrive();
                    const uint32_t rollerTime =
                        std::min<uint32_t>(700, remainingAutonMs());
                    if (rollerTime == 0) {
                        state = RedApproachState::DONE;
                        break;
                    }
                    rollerMotors.move(64);
                    pros::delay(rollerTime);
                    rollerMotors.move(0);

                    const uint32_t clawTime =
                        std::min<uint32_t>(500, remainingAutonMs());
                    if (clawTime == 0) {
                        state = RedApproachState::DONE;
                        break;
                    }
                    ClawMotor.move(34);
                    pros::delay(clawTime);
                    ClawMotor.move(0);

                    stackCollected =
                        rollerTime == 700 && clawTime == 500;
                    printf(
                        "Red/blue stack %s; color id=%d, height=%d px, "
                        "coordinate error=%.2f in.\n",
                        stackCollected ? "acquired" : "pickup timed out",
                        target.id,
                        target.height,
                        coordinateError
                    );
                    state = stackCollected && remainingAutonMs() > 0
                        ? RedApproachState::MOVE_TO_DROP_OFF
                        : RedApproachState::DONE;
                    break;
                }

                const double horizontalError = target.centerX - 160.0;
                const int turn = std::clamp(
                    static_cast<int>(horizontalError * 0.25),
                    -20,
                    20
                );
                constexpr int forward = 18;
                leftMotors.move(forward + turn);
                rightMotors.move(forward - turn);
                pros::delay(std::min<uint32_t>(20, remainingAutonMs()));
                break;
            }

            case RedApproachState::MOVE_TO_DROP_OFF:
                // Drive toward the drop-off; DROP_OFF_DOCK then makes the precise
                // final stop with the distance sensor, so a bad approach is not fatal.
                moveToStandoff("Drop-off base", dropTarget, 7.0);
                state = RedApproachState::DROP_OFF_DOCK;
                break;

            case RedApproachState::DROP_OFF_DOCK: {
                // Final, repeatable stop at the drop-off base via the distance sensor.
                dockByDistance(
                    "drop-off base",
                    dropTarget.x, dropTarget.y,
                    DROP_OFF_DOCK_DIST_IN,
                    DOCK_TOLERANCE_IN,
                    remainingAutonMs()
                );
                state = RedApproachState::RELEASE_STACK;
                break;
            }

            case RedApproachState::RELEASE_STACK: {
                const uint32_t releaseTime =
                    std::min<uint32_t>(500, remainingAutonMs());
                if (releaseTime == 0) {
                    state = RedApproachState::DONE;
                    break;
                }

                ClawMotor.move(-64);
                pros::delay(releaseTime);
                ClawMotor.move(0);
                stackDelivered = releaseTime == 500;
                if (stackDelivered) {
                    printf("Stack released at the drop-off base.\n");
                }
                state = RedApproachState::DONE;
                break;
            }

            case RedApproachState::DONE:
                break;
        }

        if (state == RedApproachState::DONE) {
            break;
        }

        pros::delay(std::min<uint32_t>(100, remainingAutonMs()));
    }

    chassis.cancelAllMotions();
    stopDrive();
    rollerMotors.move(0);
    ToggleMotor.move(0);
    ClawMotor.move(0);
    AprilTagLocalization = false;
    AprilTagReading = false;
    AprilTagCorrectionAccepted = false;
    resetTagConsist();

    if (pros::millis() - startTime >= TOTAL_TIMEOUT_MS) {
        printf("STOP: Overall autonomous timeout.\n");
    }

    if (stackCollected && stackDelivered) {
        printf("Autonomous route completed and delivered the stack.\n");
    }
    printf("Autonomous sequence ended.\n");
}

/**
 * Runs in driver control.
 */
void opcontrol() {
    while (true) {
        const int dir = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        leftMotors.move(dir + turn);
        rightMotors.move(dir - turn);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftMotors.move(64);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            liftMotors.move(-64);
        } else {
            liftMotors.move_absolute(liftMotors.get_position(), 25);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            rollerMotors.move(-64);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            ToggleMotor.move(127);
        } else {
            rollerMotors.move(0);
            ToggleMotor.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            ClawMotor.move(34);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            ClawMotor.move(-34);
        } else {
            ClawMotor.move(0);
        }

        pros::delay(20);
    }
}