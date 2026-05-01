#include <webots/Robot.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iomanip>

using namespace webots;

// Simple struct to hold info about potential box detections
struct BoxCandidate {
    int x, y;               // Center position
    int width, height;      // Size in pixels
    int area;               // Total area
    float aspectRatio;      // Width/height ratio -  identify boxes vs random shapes
    int matchPixels;        // How many pixels matched  target color
    float matchRatio;       // Percentage of the blob that is the right color
};

class NAOController {
private:
    Robot *robot;
    int timeStep;
    
    // Camera  - using both top and bottom for better view
    Camera *topCamera;
    Camera *bottomCamera;
    
    // Sonar sensors  for distance measurement - helps us know when to stop
    DistanceSensor *sonarRight;
    DistanceSensor *sonarLeft;
    
    // Right arm motors - this is what does the punching
    Motor *rShoulderPitch;   // Moves  arm forward/back
    Motor *rShoulderRoll;    // Raises arm  sideways
    Motor *rElbowRoll;       // Bends the elbow
    
    // Left arm motors - mostly just stays down during this task
    Motor *lShoulderPitch;
    Motor *lShoulderRoll;
    Motor *lElbowRoll;
    
    // Walking motion - using  pre-made motion so the robot doesn't fall over
    Motion *walkForwards;
    
    // The robot's brain - switches between looking, walking, and punching
    enum RobotState { SEARCHING, WALKING_TOWARDS, ARM_PHASE };
    RobotState currentState;
    
    // Tracking detection and movement
    double detectionStartTime;   // When we  first saw the box
    int walkSteps;               // Counts  how many steps taken 
    int frameCounter;            // For  controlling how often we print debug info
    
    // Detection settings - tuned  through trial and error
    const int MIN_BOX_SIZE = 20;          // Smaller than this is probably noise
    const int MAX_BOX_SIZE = 500;         // Bigger than this means w too close

    const float MIN_ASPECT_RATIO = 0.3;   // Accepts both tall and wide boxes
    const float MAX_ASPECT_RATIO = 3.5;   
    const float MIN_MATCH_RATIO = 0.1;    // Only needs 10% color match - pretty forgiving
    
    // Color range for our specific box - got these values from the debug  output
    // The box was showing RGB values around (182,133,83) during testing
    int boxRMin = 160, boxRMax = 210;
    int boxGMin = 120, boxGMax = 160;
    int boxBMin = 70, boxBMax = 110;
    
public:
    NAOController() {
        robot = new Robot();
        timeStep = (int)robot->getBasicTimeStep();
        
        // Set up cameras
        topCamera = robot->getCamera("CameraTop");
        bottomCamera = robot->getCamera("CameraBottom");
        
        if (topCamera) {
            topCamera->enable(timeStep);
            std::cout << "✓ Top camera enabled" << std::endl;
        }
        if (bottomCamera) {
            bottomCamera->enable(timeStep);
            std::cout << "✓ Bottom camera enabled" << std::endl;
        }
        
        // Set up sonar sensors so we  can detect how far the box is
        sonarRight = robot->getDistanceSensor("Sonar/Right");
        sonarLeft = robot->getDistanceSensor("Sonar/Left");
        
        if (sonarRight) sonarRight->enable(timeStep);
        if (sonarLeft) sonarLeft->enable(timeStep);
        
        // Get control over the arm joints
        rShoulderPitch = robot->getMotor("RShoulderPitch");
        rShoulderRoll = robot->getMotor("RShoulderRoll");
        rElbowRoll = robot->getMotor("RElbowRoll");
        lShoulderPitch = robot->getMotor("LShoulderPitch");
        lShoulderRoll = robot->getMotor("LShoulderRoll");
        lElbowRoll = robot->getMotor("LElbowRoll");
        
        // Load the walking animation
        walkForwards = new Motion("../../motions/Forwards.motion");
        std::cout << "✓ Motion files loaded" << std::endl;
        
        // Start with arms down and relaxed
        setupInitialArmPositions();
        
        // Begin  in search mode
        currentState = SEARCHING;
        detectionStartTime = 0;
        walkSteps = 0;
        frameCounter = 0;
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "NAO Robot - Cardboard Box Detection" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "🔧 Looking for color range (R:160-210, G:120-160, B:70-110)" << std::endl;
        std::cout << "   This should match your box which was around RGB(182,133,83)\n" << std::endl;
    }
    
    // Puts both arms down at the sides 
    void setupInitialArmPositions() {
        rShoulderPitch->setVelocity(0.8);
        rShoulderRoll->setVelocity(0.5);
        rElbowRoll->setVelocity(0.5);
        lShoulderPitch->setVelocity(0.5);
        lShoulderRoll->setVelocity(0.5);
        lElbowRoll->setVelocity(0.5);
         
        lShoulderPitch->setPosition(1.5);
        lShoulderRoll->setPosition(0.0);
        lElbowRoll->setPosition(0.0);
        rShoulderPitch->setPosition(1.5);
        rShoulderRoll->setPosition(0.0);
        rElbowRoll->setPosition(0.0);
        
        double startTime = robot->getTime();
        while (robot->getTime() < startTime + 2.0) {
            robot->step(timeStep);
        }
    }
    
    // Checks  if a pixel is the color of our box
    bool isMatchColor(int r, int g, int b) {
        return (r > boxRMin && r < boxRMax &&
                g > boxGMin && g < boxGMax &&
                b > boxBMin && b < boxBMax);
    }

    
    // Uses a flood fill algorithm to grow regions from seed pixels
    std::vector<BoxCandidate> findBlobs(const unsigned char* image, int width, int height) {
        std::vector<BoxCandidate> blobs;
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        
        // Scan every few  pixels for performance reasons
        for (int y = 0; y < height; y += 2) {
            for (int x = 0; x < width; x += 2) {
                int r = Camera::imageGetRed(image, width, x, y);
                int g = Camera::imageGetGreen(image, width, x, y);
                int b = Camera::imageGetBlue(image, width, x, y);
                
                if (isMatchColor(r, g, b) && !visited[y][x]) {
                    // Found a seed pixel - now  expand it to find the whole blob
                    int minX = x, maxX = x, minY = y, maxY = y;
                    int matchCount = 0;
                    int totalPixels = 0;
                    
                    std::vector<std::pair<int,int>> queue;
                    queue.push_back({x, y});
                    visited[y][x] = true;
                    
                    // Breadth-first search to find  connected pixels
                    while (!queue.empty()) {
                        auto [cx, cy] = queue.back();
                        queue.pop_back();
                        
                        // Update bounding box
                        minX = std::min(minX, cx);
                        maxX = std::max(maxX, cx);
                        minY = std::min(minY, cy);
                        maxY = std::max(maxY, cy);
                        
                        // Check neighboing pixels
                        for (int dy = -2; dy <= 2; dy += 2) {
                            for (int dx = -2; dx <= 2; dx += 2) {
                                int nx = cx + dx;
                                int ny = cy + dy;
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx]) {
                                    int nr = Camera::imageGetRed(image, width, nx, ny);
                                    int ng = Camera::imageGetGreen(image, width, nx, ny);
                                    int nb = Camera::imageGetBlue(image, width, nx, ny);
                                    if (isMatchColor(nr, ng, nb)) {
                                        visited[ny][nx] = true;
                                        queue.push_back({nx, ny});
                                    }
                                }
                            }
                        }
                    }
                    
                    // Now that   we have the bounding box, count how many pixels actually match
                    for (int by = minY; by <= maxY && by < height; by++) {
                        for (int bx = minX; bx <= maxX && bx < width; bx++) {
                            int br = Camera::imageGetRed(image, width, bx, by);
                            int bg = Camera::imageGetGreen(image, width, bx, by);
                            int bb = Camera::imageGetBlue(image, width, bx, by);
                            if (isMatchColor(br, bg, bb)) matchCount++;
                            totalPixels++;
                        }
                    }
                    
                    int boxWidth = maxX - minX;
                    int boxHeight = maxY - minY;
                    
                    // Filter by  size and shape - we want something box-like
                    if (boxWidth > MIN_BOX_SIZE && boxHeight > MIN_BOX_SIZE &&
                        boxWidth < MAX_BOX_SIZE && boxHeight < MAX_BOX_SIZE) {
                        
                        float aspectRatio = (float)boxWidth / boxHeight;
                        float matchRatio = (float)matchCount / totalPixels;
                        
                        if (aspectRatio >= MIN_ASPECT_RATIO && aspectRatio <= MAX_ASPECT_RATIO &&
                            matchRatio >= MIN_MATCH_RATIO) {
                            
                            BoxCandidate candidate;
                             candidate.x = (minX + maxX) / 2;
                            candidate.y = (minY + maxY) / 2;
                            candidate.width = boxWidth;
                            candidate.height = boxHeight;
                            candidate.area = boxWidth * boxHeight;
                        candidate.aspectRatio = aspectRatio;
                            candidate.matchPixels = matchCount;
                            candidate.matchRatio = matchRatio;
                            blobs.push_back(candidate);
                            
                            // Let the user  know something was found
                            std::cout << "   📦 Found a candidate: " << boxWidth << "x" << boxHeight 
                                      << " (" << (int)(matchRatio*100) << "% match)" << std::endl;
                        }
                    }
                }
            }
        }
        return blobs;
    }
    
    // Looks for boxes using both cameras
    bool detectCardboardBox() {
        std::vector<BoxCandidate> allCandidates;
        
        // Check top camera first
        if (topCamera) {
            const unsigned char *topImage = topCamera->getImage();
            if (topImage) {
                int width = topCamera->getWidth();
                int height = topCamera->getHeight();
                auto blobs = findBlobs(topImage, width, height);
                allCandidates.insert(allCandidates.end(), blobs.begin(), blobs.end());
            }
        }

        
        // Also check bottom camera
        if (bottomCamera) {
            const unsigned char *bottomImage = bottomCamera->getImage();
            if (bottomImage) {
                int width = bottomCamera->getWidth();
                int height = bottomCamera->getHeight();
                auto blobs = findBlobs(bottomImage, width, height);
                allCandidates.insert(allCandidates.end(), blobs.begin(), blobs.end());
            }
        }
        
        // Pick the biggest candidat
        std::sort(allCandidates.begin(), allCandidates.end(),
                  [](const BoxCandidate& a, const BoxCandidate& b) {
                      return a.area > b.area;
                  });
        
        if (!allCandidates.empty()) {
            BoxCandidate& best = allCandidates[0];
            std::cout << "\n📍 BOX SPOTTED! Size: " << best.width << "x" << best.height 
                      << " (" << (int)(best.matchRatio * 100) << "% color match)" << std::endl;
            return true;
        }
        
        return false;
    }
     
    // Reads the sonar sensors to see how far away the box is
    double getDistance() {
        if (sonarRight) {
            double dist = sonarRight->getValue();
            if (dist > 0 && dist < 5.0) return dist;
        }
        if (sonarLeft) {
            double dist = sonarLeft->getValue();
            if (dist > 0 && dist < 5.0) return dist;
        }
        return 5.0;  // Default far distance if sensors are weird
    }
    
    // Moves the  robot forward using the pre-recorded walking motion
    void walkForward() {
        if (!walkForwards) return;
        
        walkForwards->play();
        double startTime = robot->getTime();
        
        // Walk for  about 0.8 seconds, looping the motion if needed
        while (robot->getTime() < startTime + 0.8) {
            robot->step(timeStep);
            if (walkForwards->isOver()) {
                walkForwards->play();  // Keep walking if motion ended
            }
        }
        
        walkForwards->stop();
        walkSteps++;
        
        if (walkSteps % 5 == 0) {
            std::cout << "  🚶 Walking... (step " << walkSteps << ")" << std::endl;
        }
    }
    
    // Stops the  walking motion
    void stopWalking() {
        if (walkForwards) {
            walkForwards->stop();
        }
        robot->step(timeStep * 2);
    }
    
    // Decides if we're close enough to punch
    bool isVeryNear() {
        double dist = getDistance();
        return (dist > 0 && dist < 0.32);  
    }
    
    // The main event  - four phases of arm movement ending with a punch
    void performArmSequence() {
        std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║              🎯 TARGET REACHED! Time to punch!                  ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════════════════╝" << std::endl;
        
        // Set comortable  speeds for the arm movements
        rShoulderPitch->setVelocity(0.8);
        rShoulderRoll->setVelocity(0.5);
        rElbowRoll->setVelocity(0.5);
        
        // Step 1: Raise  the arm out to the side like a chicken wing
        std::cout << "  [1/4] Raising arm sideways..." << std::endl;
        rShoulderRoll->setPosition(-1.2);
        rShoulderPitch->setPosition(1.5);
        rElbowRoll->setPosition(0.0);
        
        double startTime = robot->getTime();
        while (robot->getTime() < startTime + 2.0) robot->step(timeStep);
        
        // Step 2: Bend the elbow -  getting ready to punch
        std::cout << "  [2/4] Chambering the punch..." << std::endl;
        rElbowRoll->setPosition(1.2);
        startTime = robot->getTime();
        while (robot->getTime() < startTime + 1.5) robot->step(timeStep);
        
        // Step 3: Rotate the shoulder  so the fist points forward
        std::cout << "  [3/4] Aiming forward..." << std::endl;
        rShoulderPitch->setPosition(-0.1);
        startTime = robot->getTime();
        while (robot->getTime() < startTime + 1.5) robot->step(timeStep);
        
        // Step 4: The punch
        std::cout << "  [4/4] THROWING THE PUNCH! 👊" << std::endl;
        rShoulderPitch->setVelocity(90000);  // Go as fast as possible
        rShoulderRoll->setVelocity(90000);
        rElbowRoll->setVelocity(90000);
        
        // Extend the arm forward
        rElbowRoll->setPosition(0.0);
        rShoulderRoll->setPosition(0.1);
        rShoulderPitch->setPosition(-0.1);
        
        double impactTime = robot->getTime();
        while (robot->getTime() < impactTime + 0.5) robot->step(timeStep);
        
        std::cout << "\n  💥 PUNCH LANDED! 👊 " << std::endl;
        
        // Reset to normal speeds for next time
        rShoulderPitch->setVelocity(0.8);
        rElbowRoll->setVelocity(0.5);
        
        std::cout << "\n✅ Punch sequence complete!" << std::endl;
    }
    
    // Main program loop - runs until the mission is complete
    void run() {
        std::cout << "🔍 Searching for the cardboard box..." << std::endl;
        std::cout << "   Color range: R:" << boxRMin << "-" << boxRMax 
                  << " G:" << boxGMin << "-" << boxGMax 
                  << " B:" << boxBMin << "-" << boxBMax << std::endl;
        std::cout << "   (Based on your box's actual color ~RGB 182,133,83)\n" << std::endl;
        
        while (robot->step(timeStep) != -1) {
            frameCounter++;
            
            switch (currentState) {
                case SEARCHING: {
                    bool boxDetected = detectCardboardBox();
                    
                    if (boxDetected) {
                        if (detectionStartTime == 0) {
                            detectionStartTime = robot->getTime();
                            std::cout << "\n⏱️  Need 2 seconds of confirmation before moving..." << std::endl;
                        }
                        
                        double elapsed = robot->getTime() - detectionStartTime;
                        
                        if (frameCounter % 20 == 0) {
                            int progress = (int)((elapsed / 2.0) * 20);
                            std::cout << "\r⏱️  Confirming: [";
                            for (int i = 0; i < 20; i++) {
                                std::cout << (i < progress ? "█" : "░");
                            }
                            std::cout << "] " << (int)(elapsed * 50) << "%   " << std::flush;
                        }
                        
                        if (elapsed  >= 2.0) {
                            std::cout << "\n✅ Box confirmed! Walking towards it now..." << std::endl;
                            currentState = WALKING_TOWARDS;
                            detectionStartTime = 0;
                        }
                    } else {
                        if (detectionStartTime != 0) {
                            std::cout << "\n⚠️ Lost sight of the box! Looking again..." << std::endl;
                            detectionStartTime = 0;
                        }
                        
                        if (frameCounter % 150 == 0) {
                            std::cout << "🔍 Still searching for the box..." << std::endl;
                        }
                    }
                    break;
                }
                
                case WALKING_TOWARDS: {
                    double currentDist = getDistance();
                    std::cout << "\r📏 Distance to box: " << std::fixed << std::setprecision(2) << currentDist << "m     " << std::flush;
                    
                    if (isVeryNear()) {
                        std::cout << "\n✅ We're close enough! Distance: " << currentDist << "m" << std::endl;
                        stopWalking();
                        currentState = ARM_PHASE;
                    } else {
                        walkForward();
                        
                        // Safety - don't walk forever
                        if (walkSteps > 50) {
                            std::cout << "\n⚠️ Took too many steps, stopping anyway." << std::endl;
                            stopWalking();
                            currentState = ARM_PHASE;
                        }
                    }
                    break;
                }
                
                case ARM_PHASE:
                    performArmSequence();
                    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
                    std::cout << "║                    🎉 MISSION COMPLETE! 🎉                       ║" << std::endl;
                    std::cout << "╚════════════════════════════════════════════════════════════════╝" << std::endl;
                 return;
            }
        }
    }
    
    ~NAOController() {
        delete walkForwards;
     delete robot;
    }
};

int main(int argc, char **argv) {
    NAOController controller;
    controller.run();
    return 0;
}
