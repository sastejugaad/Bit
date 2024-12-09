#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "MPU9250.h"

class Compass {
private:
    Adafruit_SSD1306 *display;
    float initialYaw;  // Stores the initial yaw to determine "front"

public:
    Compass(Adafruit_SSD1306 *disp) : display(disp), initialYaw(0) {}

    // Initialize the compass (call this in setup)
    void init(float yaw) {
        initialYaw = yaw;  // Set the current yaw as the "front" direction
    }

    // Draw the compass
    void drawCompass(float yaw) {
        // Map the yaw relative to the initialYaw
        float relativeYaw = yaw - initialYaw;

        // Convert yaw to angle in the display's coordinate system
        int arrowX = 64; // Center of the circle (x-coordinate)
        int arrowY = 32; // Center of the circle (y-coordinate)
        int radius = 20; // Circle radius
        int arrowLength = 15; // Length of the arrow

        // Calculate arrow tip position
        int arrowTipX = arrowX + arrowLength * sin(relativeYaw);
        int arrowTipY = arrowY - arrowLength * cos(relativeYaw);

        // Clear the display for redrawing
        display->clearDisplay();

        // Draw the compass circle
        display->drawCircle(arrowX, arrowY, radius, SSD1306_WHITE);

        // Draw the arrow pointing towards the relative direction
        display->drawLine(arrowX, arrowY, arrowTipX, arrowTipY, SSD1306_WHITE);

        // Draw a small circle at the center
        display->fillCircle(arrowX, arrowY, 2, SSD1306_WHITE);

        // Display the updated compass
        display->display();
    }

    // Draw a 3D cube that rotates based on IMU orientation
    void drawCube(float pitch, float yaw, float roll) {
        // Clear the display
        display->clearDisplay();

        // Cube parameters
        int size = 30;  // Size of the cube
        int centerX = 64; // Center of the screen
        int centerY = 32;

        // Define 3D cube vertices
        float vertices[8][3] = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, // Back face
            {-1, -1,  1}, {1, -1,  1}, {1, 1,  1}, {-1, 1,  1}  // Front face
        };

        // Rotate cube based on pitch, yaw, and roll
        float rotated[8][2];
        for (int i = 0; i < 8; i++) {
            // Rotate around X-axis (pitch)
            float x1 = vertices[i][0];
            float y1 = vertices[i][1] * cos(pitch) - vertices[i][2] * sin(pitch);
            float z1 = vertices[i][1] * sin(pitch) + vertices[i][2] * cos(pitch);

            // Rotate around Y-axis (yaw)
            float x2 = x1 * cos(yaw) + z1 * sin(yaw);
            float y2 = y1;
            float z2 = -x1 * sin(yaw) + z1 * cos(yaw);

            // Rotate around Z-axis (roll)
            float x3 = x2 * cos(roll) - y2 * sin(roll);
            float y3 = x2 * sin(roll) + y2 * cos(roll);
            float z3 = z2;

            // Project 3D points to 2D
            rotated[i][0] = centerX + (x3 * size) / (z3 + 3);
            rotated[i][1] = centerY + (y3 * size) / (z3 + 3);
        }

        // Draw edges of the cube
        int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Back face
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Front face
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Connecting edges
        };

        for (int i = 0; i < 12; i++) {
            int start = edges[i][0];
            int end = edges[i][1];
            display->drawLine(rotated[start][0], rotated[start][1], rotated[end][0], rotated[end][1], SSD1306_WHITE);
        }

        // Display the updated cube
        display->display();
    }
};

#endif
