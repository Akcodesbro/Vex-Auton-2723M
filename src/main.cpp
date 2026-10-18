

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
pros::Distance dist(15); // Distance sensor on port 15
// Experimental values derived from your measured image edges.
// Not a full camera calibration.
ProsPose tagPoseEstimator(
    110.0,    // Trial fx
    114.0,    // Trial fy
    160.0,    // Keep current cx
    120.0,    // Keep current cy
    0.028575  // Measured tag side: 1.125 inches (real tag is 1.125 inches, but printed is .875 inches)
);

pros::AIVision::Color yellowColor = { //yellow color description for the vision sensor
    .id = 1,
    .red = 255,
    .green = 220,
    .blue = 0,
    .hue_range = 15,
    .saturation_range = 0.5
};

//Motors
pros::Motor ClawMotor(3); // Claw motor - port 3

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 16,
pros::Rotation horizontalEnc(16);
// vertical tracking wheel encoder. Rotation sensor, port 17,
pros::Rotation verticalEnc(17);
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

constexpr double YELLOW_STACK_X = 0.0; //pos of yellow stack in inches, x is right, y is forward
constexpr double YELLOW_STACK_Y = 46.66; //pos of yellow stack in inches, x is right, y is forward

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
constexpr double CAMERA_FORWARD_IN = 1.378;
constexpr double CAMERA_RIGHT_IN = 7.625;

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

void useYellowMode() {
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

void processYellowDetection() {
    auto detections = vision.get_all_objects();
    bool foundYellow = false;
    for (const auto& detection : detections) {
        if (!pros::AIVision::is_type(
                detection, pros::AivisionDetectType::color)) {
            continue;
        }
        if (detection.id != yellowColor.id) {
            continue;
        }
        foundYellow = true;
        const auto& box = detection.object.color;
        const double centerX = box.xoffset + box.width / 2.0;
        const double centerY = box.yoffset + box.height / 2.0;
        const double horizontalError = centerX - 160.0;
        printf(
            "YELLOW: center=(%.1f, %.1f), width=%d, height=%d, error=%.1f\n",
            centerX,
            centerY,
            static_cast<int>(box.width),
            static_cast<int>(box.height),
            horizontalError
        );
    }
    if (!foundYellow) {
        printf("Yellow Not Found!");
    }
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
    vision.set_color(yellowColor);
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

enum class AutonPhase { //create an enum class to hold the different phases of the autonomous routine
    DRIVE_TO_RED_BASE,
    FINAL_RED_BASE_APPROACH,
    FIND_YELLOW_STACK,
    HANDLE_YELLOW_STACK,
    RETURN_TO_RED_BASE,
    FINAL_RETUREN_APPROACH,
    DONE
};
AutonPhase currentPhase = AutonPhase::DRIVE_TO_RED_BASE; //set the current phase to the first phase of the autonomous routine

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // ---------- STARTUP ----------
    chassis.cancelAllMotions();
    stopDrive();
    chassis.setPose(0, 0, 0);
    const uint32_t startTime = pros::millis();

    chassis.moveToPoint(0, 4, 1000, {}, false);
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
        MOVING,
        DONE
    };

    RedApproachState state = RedApproachState::ACQUIRE_TAG;

    constexpr uint32_t TOTAL_TIMEOUT_MS = 15000;
    constexpr uint32_t ACQUIRE_TIMEOUT_MS = 4000;
    constexpr int MOTION_TIMEOUT_MS = 2500;
    constexpr int APPROACH_MOTION_TIMEOUT_MS = 4000;

    // Tracking-center-to-tag clearance.
    // Measure an appropriate value for your robot before running.
    constexpr double STANDOFF_IN = 7;

    // Initial test only: cap the commanded target displacement.
    constexpr double MAX_APPROACH_TRAVEL_IN = 3.0;

    constexpr double TARGET_TOLERANCE_IN = 0.75;

    double targetX = 0.0;
    double targetY = 0.0;
    double fallbackTargetX = 0.0;
    double fallbackTargetY = 0.0;
    uint32_t acquisitionStartTime = pros::millis();

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

                const auto pose = chassis.getPose();
                const double headingRad = pose.theta * DEG_TO_RAD;
                fallbackTargetX = pose.x + std::sin(headingRad);
                fallbackTargetY = pose.y + std::cos(headingRad);

                chassis.moveToPoint(
                    fallbackTargetX,
                    fallbackTargetY,
                    MOTION_TIMEOUT_MS,
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

                if (distanceToTag <=
                    STANDOFF_IN + TARGET_TOLERANCE_IN) {
                    printf(
                        "Approach complete: center-to-tag distance "
                        "%.2f in is within %.2f in clearance tolerance.\n",
                        distanceToTag,
                        STANDOFF_IN + TARGET_TOLERANCE_IN
                    );

                    state = RedApproachState::DONE;
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
                printf("Moving from %.2f,%.2f to %.2f,%.2f,pose.x,pose.y, targetX,targetY");

                chassis.cancelAllMotions();
                stopDrive();
                // Start once. Do not restart this every loop.
                chassis.moveToPoint(targetX, targetY, 1000, {.maxSpeed = 40, .minSpeed = 30, .earlyExitRange = 0.7}, false);
                // chassis.moveToPoint(
                //     targetX,
                //     targetY,
                //     APPROACH_MOTION_TIMEOUT_MS,
                //     {.maxSpeed = 40, .minSpeed = 20, .earlyExitRange = 0.5},
                //     false
                // );

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
                        if (remainingTagDistance <=
                            STANDOFF_IN + TARGET_TOLERANCE_IN) {
                            printf(
                                "Approach complete. Tag clearance: %.2f in\n",
                                remainingTagDistance
                            );
                            state = RedApproachState::DONE;
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

            case RedApproachState::DONE:
                break;
        }

        if (state == RedApproachState::DONE) {
            break;
        }

        pros::delay(100);
    }

    // ---------- ALWAYS STOP BEFORE EXITING ----------
    chassis.cancelAllMotions();
    stopDrive();

    if (state == RedApproachState::DONE) {
        ClawMotor.move(-64);
        pros::delay(500);
        ClawMotor.move(0);

        const auto pose = chassis.getPose();
        const double headingRad = pose.theta * DEG_TO_RAD;
        const double reverseTargetX = pose.x - 2.0 * std::sin(headingRad);
        const double reverseTargetY = pose.y - 2.0 * std::cos(headingRad);
        chassis.moveToPoint(
            reverseTargetX,
            reverseTargetY,
            MOTION_TIMEOUT_MS,
            {.maxSpeed = 40},
            false
        );
    }

    AprilTagLocalization = false;
    AprilTagReading = false;
    AprilTagCorrectionAccepted = false;
    resetTagConsist();

    if (state != RedApproachState::DONE) {
        printf("STOP: Overall autonomous timeout.\n");
    }

    printf("Red-base approach routine ended.\n");
}