/*
    File: gyroplate.ino
    Author: Samuel Svensson
    Date: 2023-12-13
    Discription: A program that controlls a plateue to keep it level.
*/


//include libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "U8glib.h"

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// servo objects
Servo myservox;
Servo myservoz;

// oledscreen object
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

// global variables/arrays
// ten latest x, y and z values of g to create average values
float xOld[10];
float yOld[10];
float zOld[10];
float x_y_z[3]; // array for average g-values in 3 axis'

float originPoints [4][3] = {  // four 3D points for planes origin position
{-1.5, 0, 1},
{1.5, 0, 1},
{1.5, 0, -1},
{-1.5, 0,-1},
};

// variables for target degrees
int xDeg = 0;
int zDeg = 0;
bool horizontal = false;

// origin rotation angles for screen plane
int xOriginRotation = 20;  // rotation around the X axis
int yOriginRotation = 30;  // rotation around the Y axis
int zOriginRotation = 0;  // rotation around the Z axis

// declare variable for rotation angles
int xAngle;
int yAngle;
int zAngle;

// z-offset and size modifier for screen plane
int zOffset = -4;    // offset on Z axis
int size = 60.0;    // plane size (multiplier)

// Assign pins
int xPotentiometerPin = A0;
int zPotentiometerPin = A1;
int horizontalButtonPin = 3;


void setup(void) {
    Serial.begin(115200);

    // Setup oled-screen 
    u8g.setColorIndex(1); // set screen-color to white
    u8g.setFont(u8g_font_helvR10);

    // Set pinmode etc
    pinMode(xPotentiometerPin, INPUT);
    pinMode(zPotentiometerPin, INPUT);
    pinMode(horizontalButtonPin, INPUT);
    myservox.attach(9);
    myservoz.attach(10);

    while (!Serial) delay(10);     // will pause Zero until serial console opens

    Serial.println("LIS3DH test!");

    if (! lis.begin(0x18)) {
        Serial.println("Couldnt start");
        while (1) yield();
    }
    Serial.println("LIS3DH found!");

    // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
    Serial.print("Range = "); Serial.print(2 << lis.getRange());
    Serial.println("G");

    // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
    Serial.print("Data rate set to: ");
    switch (lis.getDataRate()) {
        case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
        case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
        case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
        case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
        case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
        case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
        case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

        case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
        case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
        case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
    }
}

void loop() {

    // get input
    accUpdate();
    updateInput();
    
    // update angles depending on horizontal mode
    if (horizontal) {
        xAngle = degrees(atan(x_y_z[2] / x_y_z[1]));
        zAngle = degrees( atan(x_y_z[0] / x_y_z[1]));
    }
    else {
        xAngle = degrees(atan(x_y_z[2] / x_y_z[1])) + xDeg;
        zAngle = degrees(atan(x_y_z[0] / x_y_z[1])) + zDeg;
    }

    // update screen and servos
    updateOled(originPoints, xAngle+xOriginRotation, yAngle+yOriginRotation, zAngle+zOriginRotation, size, zOffset, horizontal);
    updateServox(xAngle);
    updateServoz(zAngle);

    // Info to Serial monitor
    Serial.println("Accelerometer values:");
    Serial.print("x: " + String(x_y_z[0]));
    Serial.print(" y: " + String(x_y_z[1]));
    Serial.println(" z: " + String(x_y_z[2]));
    Serial.println();
    Serial.println("Potentiometer values");
    Serial.print("xDeg: " + String(xDeg));
    Serial.println(" zDeg: " + String(zDeg));
    Serial.println();
    Serial.println("Angle values:");
    Serial.print("x: " + String(xAngle));
    Serial.print(" y: " + String(yAngle));
    Serial.println(" z: " + String(zAngle));
    Serial.println();
    Serial.println("Horizontal:" + String(horizontal));
    Serial.println("----------");
    //delay(200);
}


/*
Gets "g" value for 3 axis's and stores in array "x_y_z"
Returns: void
*/
void accUpdate() {
    sensors_event_t event;
    lis.getEvent(&event);
    float x = -event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z;
    for (int i=0; i<9; i++) {
        xOld[i] = xOld[i+1];
    }
    xOld[9] = -event.acceleration.x;
    
    for (int i=0; i<9; i++) {
        yOld[i] = yOld[i+1];
    }
    yOld[9] = -event.acceleration.y;

    for (int i=0; i<9; i++) {
        zOld[i] = zOld[i+1];
    }
    zOld[9] = -event.acceleration.z;

    x_y_z [0] = -averageOfArray10(xOld);
    x_y_z [1] = averageOfArray10(zOld);
    x_y_z [2] = averageOfArray10(yOld);
}


/*
Gets input from componets
Returns: void
*/
void updateInput() {
    xDeg = map(analogRead(xPotentiometerPin), 0, 1027, -90, 90);
    zDeg = -map(analogRead(zPotentiometerPin), 0, 1027, -90, 90);

    if (digitalRead(horizontalButtonPin) == 1) {
        horizontal = !horizontal;
        delay(200);
    }
}


/*
Writes position to servo on y-axis
Parameters:
    - int pos
Returns: void
*/
void updateServox(int pos) {

    myservox.write(-pos+90);

}


/*
Writes position to servo on z-axis
Parameters:
    - int pos
Returns: void
*/
void updateServoz(int pos) {

  myservoz.write(pos+90);

}


/*
Updates oled-screen with info about the planes rotation
Parameters:
    - float originpoints[][3]
    - int xAngle
    - int yAngle
    - int zAngle
    - int size
    - int zOffset
Returns: void
*/
void updateOled(float originPoints[][3], int xAngle, int yAngle, int zAngle, int size, int zOffset, bool horizontal) {
    int x = 0;
    int y = 1;
    int z = 2;

    // planes four 3D points, rotated around XYZ - axis'
    float zRotated3dPoints [4][3];
    float zxRotated3dPoints [4][3];
    float zxyRotated3dPoints [4][3];

    int points[4][2]; // four 2D points for the rotated plane

    // calculate the points cordinates after three rotations
    for (int i=0; i<4; i++) {
        // rotate 3d points around the z axis (rotating X and Y positions)
        zRotated3dPoints [i][x] = originPoints [i][x] * cos(radians(zAngle)) - originPoints [i][y] * sin(radians(zAngle));
        zRotated3dPoints [i][y] = originPoints [i][x] * sin(radians(zAngle)) + originPoints [i][y] * cos(radians(zAngle));
        zRotated3dPoints [i][z] = originPoints [i][z]; 

        // rotate 3d points around the x axis (rotating Y and Z positions)
        zxRotated3dPoints [i][x] = zRotated3dPoints [i][x];
        zxRotated3dPoints [i][y] = zRotated3dPoints [i][y] * cos(radians(xAngle)) - zRotated3dPoints [i][z] * sin(radians(xAngle));
        zxRotated3dPoints [i][z] = zRotated3dPoints [i][y] * sin(radians(xAngle)) + zRotated3dPoints [i][z] * cos(radians(xAngle));

        // rotate 3d points around the y axis (rotating X and Z positions)
        zxyRotated3dPoints [i][x] = zxRotated3dPoints [i][x] * cos(radians(yAngle)) + zxRotated3dPoints [i][z] * sin(radians(yAngle));
        zxyRotated3dPoints [i][y] = zxRotated3dPoints [i][y];
        zxyRotated3dPoints [i][z] = -zxRotated3dPoints [i][x] * sin(radians(yAngle)) + zxRotated3dPoints [i][z] * cos(radians(yAngle)) + zOffset; 
        

        // project 3d points into 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
        points[i][x] = round(64 + zxyRotated3dPoints [i][x] / zxyRotated3dPoints [i][z] * size);
        points[i][y] = round(32 + zxyRotated3dPoints [i][y] / zxyRotated3dPoints [i][z] * size);
    }

    u8g.firstPage();
    do {
        // connect the lines between the individual points
        u8g.drawLine(points[ 0 ][ x ], points[ 0 ][ y ], points[ 1 ][ x ], points[ 1 ][ y ]);  // connect points 0-1
        u8g.drawLine(points[ 1 ][ x ], points[ 1 ][ y ], points[ 2 ][ x ], points[ 2 ][ y ]);  // connect points 1-2
        u8g.drawLine(points[ 2 ][ x ], points[ 2 ][ y ], points[ 3 ][ x ], points[ 3 ][ y ]);  // connect points 2-3
        u8g.drawLine(points[ 3 ][ x ], points[ 3 ][ y ], points[ 0 ][ x ], points[ 0 ][ y ]);  // connect points 3-0

        u8g.drawLine(points[ 0 ][ x ], points[ 0 ][ y ], points[ 2 ][ x ], points[ 2 ][ y ]);   // connect points 0-2
        u8g.drawLine(points[ 1 ][ x ], points[ 1 ][ y ], points[ 3 ][ x ], points[ 3 ][ y ]);   // connect points 1-3


        // draw half circle gauges -90 - 90 deg
        int gaugeSize = 10;

        // left side    x
        int leftGaugeX = 20;
        int leftGaugeY = 48;
        u8g.drawCircle(leftGaugeX, leftGaugeY, gaugeSize, U8G_DRAW_UPPER_LEFT | U8G_DRAW_UPPER_RIGHT);
        u8g.drawLine(leftGaugeX, leftGaugeY, leftGaugeX + cos(radians(xAngle-20+90))*gaugeSize, leftGaugeY - sin(radians(xAngle-20+90))*gaugeSize);
        u8g.drawStr(leftGaugeX -16, leftGaugeY -14, String(xDeg).c_str());

        // right side   z
        int rightGaugeX = 108;
        int rightGaugeY = 48;
        u8g.drawCircle(rightGaugeX, rightGaugeY, gaugeSize, U8G_DRAW_UPPER_LEFT | U8G_DRAW_UPPER_RIGHT);
        u8g.drawLine(rightGaugeX, rightGaugeY, rightGaugeX + cos(radians(-zAngle+90))*gaugeSize, rightGaugeY - sin(radians(-zAngle+90))*gaugeSize);
        u8g.drawStr(rightGaugeX, rightGaugeY -14, String(zDeg).c_str());


        // horizontalmode - on/off
        if (horizontal) {
            u8g.drawStr(10, 63, "H-mode: ON");
        }
        else {
            u8g.drawStr(10, 63, "H-mode: OFF");
        }
    } while ( u8g.nextPage() );   // u8g library specific, has to be there

}

/*
Calculates average value of an array with size 10
Parameters:
    - float a[10]
Returns: int
*/
int averageOfArray10(float a[10]) {
    int sum = 0;
    for (int i=0; i<10; i++) {
        sum += a[i];
    }
    return sum/10;
}
