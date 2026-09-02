#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <TinyGPSPlus.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>

// ============================================================
// DISPLAY
// ============================================================

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_GC9A01 _panel;
    lgfx::Bus_SPI _bus;

public:
    LGFX()
    {
        auto busCfg = _bus.config();

        busCfg.spi_host = SPI2_HOST;
        busCfg.spi_mode = 0;
        busCfg.freq_write = 10000000;
        busCfg.freq_read = 10000000;
        busCfg.spi_3wire = false;
        busCfg.use_lock = true;
        busCfg.dma_channel = SPI_DMA_CH_AUTO;

        busCfg.pin_sclk = 12;
        busCfg.pin_mosi = 11;
        busCfg.pin_miso = -1;
        busCfg.pin_dc = 9;

        _bus.config(busCfg);
        _panel.setBus(&_bus);

        auto panelCfg = _panel.config();

        panelCfg.pin_cs = 10;
        panelCfg.pin_rst = 8;
        panelCfg.pin_busy = -1;

        panelCfg.memory_width = 240;
        panelCfg.memory_height = 240;
        panelCfg.panel_width = 240;
        panelCfg.panel_height = 240;

        panelCfg.offset_x = 0;
        panelCfg.offset_y = 0;
        panelCfg.offset_rotation = 0;

        panelCfg.readable = false;
        panelCfg.invert = true;
        panelCfg.rgb_order = false;
        panelCfg.dlen_16bit = false;
        panelCfg.bus_shared = false;

        _panel.config(panelCfg);
        setPanel(&_panel);
    }
};

LGFX tft;

// ============================================================
// SPRITES
// ============================================================

LGFX_Sprite clockSprite(&tft);
LGFX_Sprite compassSprite(&tft);

// ============================================================
// PINS
// ============================================================

#define TFT_BL 7

#define BUTTON_1 1
#define BUTTON_2 20

// ============================================================
// CAPACITIVE TOUCH
// ============================================================
// GPIO 2 = additional Button 1
// GPIO 3 = additional Button 2
// Physical buttons remain fully functional.
// ============================================================

#define TOUCH_BUTTON_1 13                 
#define TOUCH_BUTTON_2 14

// Touch readings are compared with the startup baseline.
// Your measured readings were approximately:
// GPIO 2 untouched = 20500
// GPIO 3 untouched = 25500
//
// A 5000-count deviation is used as the touch trigger.
// This detects both upward and downward changes and avoids
// relying on the direction of the touch signal.
const uint16_t TOUCH_DEVIATION_THRESHOLD = 5000;
const int TOUCH_CALIBRATION_SAMPLES = 100;

uint16_t touchBaseline1 = 0;
uint16_t touchBaseline2 = 0;

bool touchButton1State = false;
bool touchButton2State = false;

bool touchButton1Handled = false;
bool touchButton2Handled = false;

// BME280
#define BME_SDA 5
#define BME_SCL 4

// ============================================================
// GPS
// ============================================================

#define GPS_RX_PIN 18
#define GPS_TX_PIN 17

HardwareSerial GPS(1);
TinyGPSPlus gps;

// ============================================================
// BME280
// ============================================================

Adafruit_BME280 bme;

bool bmeAvailable = false;

// Fixed calibration offset applied to raw temperature readings.
// BME280 tends to read a few degrees high due to self-heating
// (especially in an enclosure near the ESP32/display).
// Adjust this value by comparing against a trusted reference
// thermometer once the device has stabilized at ambient temp.
const float TEMP_CALIBRATION_OFFSET = -2.;

float temperatureC = 0.0;
float pressureHpa = 0.0;
float humidity = 0.0;

float previousTemperature = 0.0;
float previousPressure = 0.0;
float previousHumidity = 0.0;

bool havePreviousWeather = false;

unsigned long lastWeatherReading = 0;

const unsigned long WEATHER_UPDATE_INTERVAL = 10000;

const float TEMP_TREND_THRESHOLD = 0.2;
const float PRESSURE_TREND_THRESHOLD = 1.0;
const float HUMIDITY_TREND_THRESHOLD = 2.0;
float seaLevelPressureHpa = 1020.0f;
bool seaLevelPressureCalibrated = false;

const float GPS_CALIBRATION_HDOP = 1.5f;
const float ALTITUDE_TREND_THRESHOLD_METERS = 2.0;

int temperatureTrend = 0;
int pressureTrend = 0;
int humidityTrend = 0;
int altitudeTrend = 0;

float previousGPSAltitude = 0.0f;
bool havePreviousGPSAltitude = false;
unsigned long lastGPSAltitudeTrendUpdate = 0;

const unsigned long GPS_ALTITUDE_TREND_INTERVAL = 10000;

int calculateTrend(
    float current,
    float previous,
    float threshold
);

float pressureAltitudeMeters()
{
    if (pressureHpa <= 0.0f)
        return 0.0f;

    return 44330.0f *
        (1.0f - pow(
            pressureHpa /
            seaLevelPressureHpa,
            0.1903f
        ));
}

void calibrateSeaLevelPressure()
{
    if (
        seaLevelPressureCalibrated ||
        !bmeAvailable ||
        pressureHpa <= 0.0f ||
        !gps.location.isValid() ||
        !gps.altitude.isValid() ||
        !gps.hdop.isValid() ||
        gps.hdop.hdop() >
        GPS_CALIBRATION_HDOP)
    {
        return;
    }

    float altitudeMeters =
        gps.altitude.meters();

    float pressureRatio =
        1.0f -
        (altitudeMeters / 44330.0f);

    if (pressureRatio <= 0.0f)
        return;

    seaLevelPressureHpa =
        pressureHpa /
        pow(pressureRatio, 5.255f);

    seaLevelPressureCalibrated =
        true;

    Serial.print(
        "Sea-level pressure calibrated: "
    );

    Serial.println(
        seaLevelPressureHpa,
        1
    );
}

void updateAltitudeTrend()
{
    if (!gps.altitude.isValid())
        return;

    unsigned long now =
        millis();

    float currentAltitude =
        gps.altitude.meters();

    if (!havePreviousGPSAltitude)
    {
        previousGPSAltitude =
            currentAltitude;

        havePreviousGPSAltitude =
            true;

        lastGPSAltitudeTrendUpdate =
            now;

        altitudeTrend = 0;

        return;
    }

    if (
        now -
        lastGPSAltitudeTrendUpdate <
        GPS_ALTITUDE_TREND_INTERVAL)
    {
        return;
    }

    altitudeTrend =
        calculateTrend(
            currentAltitude,
            previousGPSAltitude,
            ALTITUDE_TREND_THRESHOLD_METERS
        );

    previousGPSAltitude =
        currentAltitude;

    lastGPSAltitudeTrendUpdate =
        now;
}

// ============================================================
// SCREEN
// ============================================================

const int CX = 120;
const int CY = 120;

void drawTrendArrow(
    int x,
    int y,
    int trend
);

// ============================================================
// THEME
// ============================================================

// false = dark
// true  = light

bool lightTheme = false;

const unsigned long THEME_HOLD_TIME = 3000;

bool themeTriggered = false;

// ------------------------------------------------------------
// THEME COLOURS
// ------------------------------------------------------------

uint16_t themeBackground()
{
    return lightTheme
        ? TFT_WHITE
        : TFT_BLACK;
}

uint16_t themeText()
{
    return lightTheme
        ? TFT_BLACK
        : TFT_WHITE;
}

uint16_t themeSecondaryText()
{
    return lightTheme
        ? TFT_DARKGREY
        : TFT_LIGHTGREY;
}

uint16_t themeLine()
{
    return lightTheme
        ? TFT_LIGHTGREY
        : TFT_DARKGREY;
}

void showSplashScreen()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        themeText()
    );

    tft.drawString(
        "BIKE",
        CX,
        CY - 18
    );

    tft.drawString(
        "COMPUTER",
        CX,
        CY + 18
    );

    tft.setFont(
        &fonts::Font2
    );
    
    tft.drawString(
        "By Stevcrow74",
        CX,
        CY + 35
    );

    tft.drawCircle(
        CX - 58,
        CY,
        18,
        themeLine()
    );

    tft.drawCircle(
        CX + 58,
        CY,
        18,
        themeLine()
    );

    delay(5000);
}

// ============================================================
// PAGES
// ============================================================

// Button 1:
// 0 MAIN
// 1 TRIP
// 2 GPS
// 3 COMPASS
// 4 CLOCK
// 8 HISTORY
//
// Button 2:
// 4 CLOCK
// 5 WEATHER
// 6 BAROMETER
// 7 SYSTEM

int currentPage = 0;
int previousPage = -1;

bool pageNeedsRedraw = true;

// ============================================================
// BUTTON 1
// ============================================================

bool lastButton1State = HIGH;
unsigned long lastButton1Change = 0;

unsigned long button1PressStart = 0;

bool button1Held = false;
bool tripResetTriggered = false;

const unsigned long BUTTON_DEBOUNCE = 180;
const unsigned long TRIP_RESET_HOLD_TIME = 3000;

// ============================================================
// BUTTON 2
// ============================================================

bool lastButton2State = HIGH;
unsigned long lastButton2Change = 0;

unsigned long button2PressStart = 0;

bool button2Held = false;

// ============================================================
// PAGE TIMEOUT
// ============================================================

unsigned long pageEnteredTime = 0;

const unsigned long INFO_PAGE_TIMEOUT = 10000;

// ============================================================
// GPS DATA
// ============================================================

float speedKmh = 0.0;

double distanceKm = 0.0;
double odometerKm = 0.0;

double lastLat = 0.0;
double lastLon = 0.0;

bool havePreviousPosition = false;

int satellites = 0;

double hdop = 99.9;

bool gpsFix = false;

// ============================================================
// HISTORY GRAPH
// ============================================================

const int HISTORY_SAMPLE_COUNT = 60;
const unsigned long HISTORY_SAMPLE_INTERVAL = 10000;

float speedHistory[HISTORY_SAMPLE_COUNT] = {};
float altitudeHistory[HISTORY_SAMPLE_COUNT] = {};
bool altitudeHistoryValid[HISTORY_SAMPLE_COUNT] = {};

int historySampleCount = 0;
int historyWriteIndex = 0;

unsigned long lastHistorySample = 0;
unsigned long lastHistoryPageUpdate = 0;

void recordHistorySample()
{
    unsigned long now = millis();

    if (
        now -
        lastHistorySample <
        HISTORY_SAMPLE_INTERVAL)
    {
        return;
    }

    lastHistorySample = now;

    speedHistory[historyWriteIndex] =
        speedKmh;

    altitudeHistoryValid[historyWriteIndex] =
        gps.altitude.isValid();

    if (altitudeHistoryValid[historyWriteIndex])
    {
        altitudeHistory[historyWriteIndex] =
            gps.altitude.meters();
    }

    historyWriteIndex =
        (historyWriteIndex + 1) %
        HISTORY_SAMPLE_COUNT;

    if (historySampleCount < HISTORY_SAMPLE_COUNT)
        historySampleCount++;
}

// ============================================================
// TRIP DATA
// ============================================================

float maximumSpeed = 0.0;
float averageSpeed = 0.0;

double speedSum = 0.0;
unsigned long speedSamples = 0;

Preferences tripStorage;

const unsigned long TRIP_SAVE_INTERVAL = 30000;

unsigned long lastTripSave = 0;

void loadTripData()
{
    tripStorage.begin(
        "trip",
        true
    );

    distanceKm =
        tripStorage.getDouble(
            "distance",
            0.0
        );

    odometerKm =
        tripStorage.getDouble(
            "odometer",
            0.0
        );

    maximumSpeed =
        tripStorage.getFloat(
            "maxSpeed",
            0.0f
        );

    speedSum =
        tripStorage.getDouble(
            "speedSum",
            0.0
        );

    speedSamples =
        tripStorage.getULong(
            "samples",
            0
        );

    tripStorage.end();

    if (speedSamples > 0)
    {
        averageSpeed =
            speedSum /
            speedSamples;
    }
}

void saveTripData()
{
    tripStorage.begin(
        "trip",
        false
    );

    tripStorage.putDouble(
        "distance",
        distanceKm
    );

    tripStorage.putDouble(
        "odometer",
        odometerKm
    );

    tripStorage.putFloat(
        "maxSpeed",
        maximumSpeed
    );

    tripStorage.putDouble(
        "speedSum",
        speedSum
    );

    tripStorage.putULong(
        "samples",
        speedSamples
    );

    tripStorage.end();
}

void resetTripData()
{
    distanceKm = 0.0;
    maximumSpeed = 0.0f;
    averageSpeed = 0.0f;
    speedSum = 0.0;
    speedSamples = 0;

    saveTripData();
}

// ============================================================
// GAUGE
// ============================================================

const float GAUGE_START = 225.0;
const float GAUGE_END = 495.0;

const int GAUGE_INNER = 94;
const int GAUGE_OUTER = 113;

// ============================================================
// SPEED BAR
// ============================================================

const int SPEED_BAR_POSITION = 115;
const int SPEED_BAR_RADIUS = 15;

float previousBarSpeed = 0.0;
bool speedBarStarted = false;

// ============================================================
// GAUGE MODE
// ============================================================

bool highSpeedMode = false;

// ============================================================
// CLOCK
// ============================================================

String lastTime = "";

int previousClockSecond = -1;

unsigned long lastClockUpdate = 0;

// ============================================================
// COMPASS
// ============================================================

float currentHeading = 0.0;

unsigned long lastCompassUpdate = 0;

// ============================================================
// BAROMETER
// ============================================================

const int BARO_CX = 120;
const int BARO_CY = 120;

const float BARO_START_ANGLE = -135.0;
const float BARO_END_ANGLE = 135.0;

const float BARO_MIN = 980.0;
const float BARO_MAX = 1040.0;

float previousBarometerPressure = -999.0;

bool barometerStarted = false;

// ============================================================
// SYSTEM PAGE
// ============================================================

unsigned long lastSystemUpdate = 0;

void updateSystemPage();
void drawCurrentPage();

// ============================================================
// GAUGE ANGLE
// ============================================================

float gaugeAngle(float speed)
{
    float maxSpeed =
        highSpeedMode ? 120.0f : 40.0f;

    speed =
        constrain(
            speed,
            0.0f,
            maxSpeed
        );

    return GAUGE_START +
           ((speed / maxSpeed) *
            (GAUGE_END - GAUGE_START));
}

// ============================================================
// DRAW GAUGE TICK
// ============================================================

void drawTick(
    float angle,
    int inner,
    int outer,
    uint16_t colour)
{
    float rad = radians(angle);

    int x1 =
        CX + cos(rad) * inner;

    int y1 =
        CY + sin(rad) * inner;

    int x2 =
        CX + cos(rad) * outer;

    int y2 =
        CY + sin(rad) * outer;

    tft.drawLine(
        x1,
        y1,
        x2,
        y2,
        colour
    );
}

// ============================================================
// DRAW GAUGE
// ============================================================

void drawGauge()
{
    tft.drawCircle(
        CX,
        CY,
        116,
        themeLine()
    );

    tft.drawCircle(
        CX,
        CY,
        114,
        themeLine()
    );

    int maxSpeed =
        highSpeedMode ? 120 : 40;

    int tickStep =
        highSpeedMode ? 5 : 2;

    for (
        int speed = 0;
        speed <= maxSpeed;
        speed += tickStep)
    {
        float angle =
            gaugeAngle(speed);

        int inner =
            GAUGE_INNER;

        if (highSpeedMode)
        {
            if (speed % 20 == 0)
                inner = 88;
        }
        else
        {
            if (speed % 10 == 0)
                inner = 88;
        }

        uint16_t colour;

        if (highSpeedMode)
        {
            if (speed >= 100)
                colour = TFT_RED;
            else if (speed >= 70)
                colour = TFT_YELLOW;
            else
                colour = TFT_GREEN;
        }
        else
        {
            if (speed >= 35)
                colour = TFT_RED;
            else if (speed >= 25)
                colour = TFT_YELLOW;
            else
                colour = TFT_GREEN;
        }

        drawTick(
            angle,
            inner,
            GAUGE_OUTER,
            colour
        );
    }

    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::Font2);
    tft.setTextColor(themeText());

    int numberStep =
        highSpeedMode ? 20 : 10;

    for (
        int speed = 0;
        speed <= maxSpeed;
        speed += numberStep)
    {
        float angle =
            gaugeAngle(speed);

        float rad =
            radians(angle);

        const int radius = 77;

        int x =
            CX +
            cos(rad) * radius;

        int y =
            CY +
            sin(rad) * radius;

        tft.drawNumber(
            speed,
            x,
            y
        );
    }
}

// ============================================================
// SPEED BAR COLOUR
// ============================================================

uint16_t speedBarColour(float speed)
{
    if (highSpeedMode)
    {
        if (speed >= 100)
            return TFT_RED;

        if (speed >= 70)
            return TFT_YELLOW;

        return TFT_GREEN;
    }

    if (speed >= 35)
        return TFT_RED;

    if (speed >= 25)
        return TFT_YELLOW;

    return TFT_GREEN;
}

// ============================================================
// DRAW SPEED BAR POINT
// ============================================================

void drawSpeedBarPoint(
    float angle,
    uint16_t colour)
{
    float rad =
        radians(angle);

    int x =
        CX +
        cos(rad) *
        SPEED_BAR_POSITION;

    int y =
        CY +
        sin(rad) *
        SPEED_BAR_POSITION;

    tft.fillCircle(
        x,
        y,
        SPEED_BAR_RADIUS,
        colour
    );
}

// ============================================================
// ERASE SPEED BAR POINT
// ============================================================

void eraseSpeedBarPoint(float angle)
{
    float rad =
        radians(angle);

    int x =
        CX +
        cos(rad) *
        SPEED_BAR_POSITION;

    int y =
        CY +
        sin(rad) *
        SPEED_BAR_POSITION;

    tft.fillCircle(
        x,
        y,
        SPEED_BAR_RADIUS + 1,
        themeBackground()
    );
}

// ============================================================
// ERASE SPEED BAR SECTION
// ============================================================

void eraseSpeedBarSection(
    float fromSpeed,
    float toSpeed)
{
    if (toSpeed <= fromSpeed)
        return;

    float startAngle =
        gaugeAngle(fromSpeed);

    float endAngle =
        gaugeAngle(toSpeed);

    for (
        float angle = startAngle;
        angle <= endAngle;
        angle += 0.6)
    {
        eraseSpeedBarPoint(angle);
    }

    tft.drawCircle(
        CX,
        CY,
        116,
        themeLine()
    );

    tft.drawCircle(
        CX,
        CY,
        114,
        themeLine()
    );
}

// ============================================================
// DRAW SPEED BAR SECTION
// ============================================================

void drawSpeedBarSection(
    float fromSpeed,
    float toSpeed)
{
    if (toSpeed <= fromSpeed)
        return;

    float startAngle =
        gaugeAngle(fromSpeed);

    float endAngle =
        gaugeAngle(toSpeed);

    float maxSpeed =
        highSpeedMode
        ? 120.0
        : 40.0;

    for (
        float angle = startAngle;
        angle <= endAngle;
        angle += 0.6)
    {
        float barSpeed =
            ((angle - GAUGE_START) /
             (GAUGE_END - GAUGE_START)) *
            maxSpeed;

        drawSpeedBarPoint(
            angle,
            speedBarColour(barSpeed)
        );
    }
}

// ============================================================
// DRAW SPEED BAR
// ============================================================

void drawSpeedBar()
{
    float maxBarSpeed =
        highSpeedMode
        ? 120.0
        : 40.0;

    float newSpeed =
        constrain(
            speedKmh,
            0.0f,
            maxBarSpeed
        );

    if (!speedBarStarted)
    {
        speedBarStarted = true;
        previousBarSpeed = 0.0;

        if (newSpeed > 0.1)
        {
            drawSpeedBarSection(
                0.0,
                newSpeed
            );
        }

        previousBarSpeed =
            newSpeed;

        return;
    }

    if (
        newSpeed >
        previousBarSpeed + 0.05)
    {
        drawSpeedBarSection(
            previousBarSpeed,
            newSpeed
        );
    }
    else if (
        newSpeed <
        previousBarSpeed - 0.05)
    {
        eraseSpeedBarSection(
            newSpeed,
            previousBarSpeed
        );

        if (newSpeed > 0.1)
        {
            drawSpeedBarSection(
                0,
                newSpeed
            );
        }
    }

    previousBarSpeed =
        newSpeed;
}

// ============================================================
// SATELLITE SYMBOL
// ============================================================

void drawSatelliteSymbol(
    uint16_t colour)
{
    const int x = 42;
    const int y = 108;

    tft.fillRoundRect(
        x - 3,
        y - 3,
        6,
        6,
        1,
        colour
    );

    tft.fillRect(
        x - 9,
        y - 3,
        5,
        6,
        colour
    );

    tft.fillRect(
        x + 4,
        y - 3,
        5,
        6,
        colour
    );

    tft.drawLine(
        x,
        y - 4,
        x,
        y - 8,
        colour
    );

    tft.drawArc(
        x,
        y - 8,
        6,
        6,
        220,
        320,
        colour
    );
}

// ============================================================
// GPS COLOUR
// ============================================================

uint16_t getGPSColour()
{
    if (!gpsFix)
        return TFT_RED;

    if (
        satellites < 4 ||
        hdop > 2.5)
    {
        return TFT_YELLOW;
    }

    return TFT_GREEN;
}

// ============================================================
// SATELLITE DISPLAY
// ============================================================

void updateSatelliteSymbol()
{
    uint16_t colour =
        getGPSColour();

    tft.fillRect(
        30,
        91,
        24,
        18,
        themeBackground()
    );

    drawSatelliteSymbol(
        colour
    );

    tft.fillRect(
        18,
        108,
        48,
        13,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        colour
    );

    char satText[12];

    if (gpsFix)
    {
        snprintf(
            satText,
            sizeof(satText),
            "%d",
            satellites
        );
    }
    else
    {
        snprintf(
            satText,
            sizeof(satText),
            "--"
        );
    }

    tft.drawString(
        satText,
        42,
        115
    );
}

// ============================================================
// MAIN STATIC SCREEN
// ============================================================

void drawStaticScreen()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.drawCircle(
        CX,
        CY,
        70,
        lightTheme ? 0xC618 : 0x18C3
    );

    tft.drawCircle(
        CX,
        CY,
        72,
        lightTheme ? 0xE71C : 0x10A2
    );

    drawGauge();

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "KM/H",
        CX,
        133
    );

    updateSatelliteSymbol();
}

// ============================================================
// SPEED
// ============================================================

void updateSpeed()
{
    tft.fillRect(
        58,
        82,
        124,
        43,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font7
    );

    tft.setTextColor(
        themeText()
    );

    char speedText[10];

    snprintf(
        speedText,
        sizeof(speedText),
        "%02.1f",
        speedKmh
    );

    tft.drawString(
        speedText,
        CX,
        103
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "KM/H",
        CX,
        133
    );
}

// ============================================================
// MAIN TEMPERATURE
// ============================================================

void updateMainTemperature()
{
    tft.fillRect(
        72,
        142,
        96,
        32,
        themeBackground()
    );

    tft.fillRect(
        171,
        147,
        19,
        19,
        themeBackground()
    );

    if (!bmeAvailable)
        return;

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        themeText()
    );

    char tempText[16];

    snprintf(
        tempText,
        sizeof(tempText),
        "%.1f",
        temperatureC
    );

    tft.drawString(
        tempText,
        CX - 5,
        156
    );

    int textWidth =
        tft.textWidth(
            tempText
        );

    int degreeX =
        (CX - 5) +
        (textWidth / 2) +
        5;

    tft.drawCircle(
        degreeX,
        146,
        2,
        themeText()
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.drawString(
        "C",
        degreeX + 9,
        157
    );

    drawTrendArrow(
        180,
        156,
        altitudeTrend
    );
}

// ============================================================
// DISTANCE
// ============================================================

void updateMainDistance()
{
    tft.fillRect(
        20,
        124,
        80,
        39,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "DIST",
        42,
        130
    );

    tft.setTextColor(
        themeText()
    );

    char distanceText[20];

    snprintf(
        distanceText,
        sizeof(distanceText),
        "%.2f km",
        distanceKm
    );

    tft.drawString(
        distanceText,
        42,
        148
    );

    updateMainTemperature();
}

// ============================================================
// HISTORY PAGE
// ============================================================

void drawHistoryGraph(
    int top,
    int height,
    const float* values,
    const bool* valid,
    float minimum,
    float maximum,
    uint16_t colour
)
{
    const int left = 22;
    const int right = 222;
    const int bottom = top + height;

    tft.drawRect(
        left,
        top,
        right - left,
        height,
        themeLine()
    );

    if (historySampleCount < 2)
        return;

    int firstIndex =
        (historyWriteIndex - historySampleCount +
         HISTORY_SAMPLE_COUNT) %
        HISTORY_SAMPLE_COUNT;

    bool havePreviousPoint = false;
    int previousX = 0;
    int previousY = 0;

    for (int sample = 0; sample < historySampleCount; sample++)
    {
        int index =
            (firstIndex + sample) %
            HISTORY_SAMPLE_COUNT;

        if (valid != nullptr && !valid[index])
        {
            havePreviousPoint = false;
            continue;
        }

        float fraction =
            (values[index] - minimum) /
            (maximum - minimum);

        fraction = constrain(
            fraction,
            0.0f,
            1.0f
        );

        int x = right - 1 -
            ((right - left - 1) *
             (historySampleCount - 1 - sample)) /
            (HISTORY_SAMPLE_COUNT - 1);

        int y = bottom - 1 -
            (int)((height - 2) * fraction);

        if (havePreviousPoint)
        {
            tft.drawLine(
                previousX,
                previousY,
                x,
                y,
                colour
            );
        }

        previousX = x;
        previousY = y;
        havePreviousPoint = true;
    }
}

void drawHistoryPage()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "HISTORY",
        CX,
        16
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "SPEED KM/H",
        CX,
        36
    );

    float maximumSpeedHistory = 10.0f;

    for (int sample = 0; sample < historySampleCount; sample++)
    {
        int index =
            (historyWriteIndex - historySampleCount + sample +
             HISTORY_SAMPLE_COUNT) %
            HISTORY_SAMPLE_COUNT;

        if (speedHistory[index] > maximumSpeedHistory)
            maximumSpeedHistory =
                speedHistory[index];
    }

    drawHistoryGraph(
        46,
        65,
        speedHistory,
        nullptr,
        0.0f,
        maximumSpeedHistory,
        TFT_GREEN
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "GPS ALTITUDE M",
        CX,
        132
    );

    float minimumAltitude = 0.0f;
    float maximumAltitude = 1.0f;
    bool haveAltitude = false;

    for (int sample = 0; sample < historySampleCount; sample++)
    {
        int index =
            (historyWriteIndex - historySampleCount + sample +
             HISTORY_SAMPLE_COUNT) %
            HISTORY_SAMPLE_COUNT;

        if (!altitudeHistoryValid[index])
            continue;

        float altitude =
            altitudeHistory[index];

        if (!haveAltitude)
        {
            minimumAltitude = altitude;
            maximumAltitude = altitude;
            haveAltitude = true;
        }
        else
        {
            minimumAltitude = min(
                minimumAltitude,
                altitude
            );

            maximumAltitude = max(
                maximumAltitude,
                altitude
            );
        }
    }

    if (maximumAltitude - minimumAltitude < 10.0f)
    {
        float midpoint =
            (maximumAltitude + minimumAltitude) /
            2.0f;

        minimumAltitude = midpoint - 5.0f;
        maximumAltitude = midpoint + 5.0f;
    }

    drawHistoryGraph(
        142,
        65,
        altitudeHistory,
        altitudeHistoryValid,
        minimumAltitude,
        maximumAltitude,
        TFT_GREEN
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "10 MINUTES",
        CX,
        224
    );
}

// ============================================================
// DAY OF WEEK
// ============================================================

int dayOfWeek(
    int year,
    int month,
    int day)
{
    if (month < 3)
    {
        month += 12;
        year--;
    }

    int k = year % 100;
    int j = year / 100;

    int h =
        (
            day +
            (13 * (month + 1)) / 5 +
            k +
            k / 4 +
            j / 4 +
            5 * j
        ) % 7;

    return (h + 6) % 7;
}

// ============================================================
// IRISH SUMMER TIME
// ============================================================

bool isIrishSummerTime(
    int year,
    int month,
    int day,
    int hour)
{
    if (
        month < 3 ||
        month > 10)
        return false;

    if (
        month > 3 &&
        month < 10)
        return true;

    int marchLastSunday = 31;

    while (
        dayOfWeek(
            year,
            3,
            marchLastSunday
        ) != 0)
    {
        marchLastSunday--;
    }

    int octoberLastSunday = 31;

    while (
        dayOfWeek(
            year,
            10,
            octoberLastSunday
        ) != 0)
    {
        octoberLastSunday--;
    }

    if (month == 3)
    {
        if (day > marchLastSunday)
            return true;

        if (day < marchLastSunday)
            return false;

        return hour >= 1;
    }

    if (month == 10)
    {
        if (day < octoberLastSunday)
            return true;

        if (day > octoberLastSunday)
            return false;

        return hour < 1;
    }

    return false;
}

// ============================================================
// GPS IRISH TIME
// ============================================================

String getIrishTime()
{
    if (
        !gps.date.isValid() ||
        !gps.time.isValid())
    {
        return "--:--:--";
    }

    int year =
        gps.date.year();

    int month =
        gps.date.month();

    int day =
        gps.date.day();

    int hour =
        gps.time.hour();

    int minute =
        gps.time.minute();

    int second =
        gps.time.second();

    if (
        isIrishSummerTime(
            year,
            month,
            day,
            hour))
    {
        hour++;
    }

    if (hour >= 24)
    {
        hour = 0;
        day++;

        int daysInMonth = 31;

        if (
            month == 4 ||
            month == 6 ||
            month == 9 ||
            month == 11)
        {
            daysInMonth = 30;
        }
        else if (month == 2)
        {
            bool leap =
                (
                    year % 4 == 0 &&
                    (
                        year % 100 != 0 ||
                        year % 400 == 0
                    )
                );

            daysInMonth =
                leap ? 29 : 28;
        }

        if (day > daysInMonth)
        {
            day = 1;
            month++;

            if (month > 12)
            {
                month = 1;
                year++;
            }
        }
    }

    char buffer[12];

    snprintf(
        buffer,
        sizeof(buffer),
        "%02d:%02d:%02d",
        hour,
        minute,
        second
    );

    return String(buffer);
}

// ============================================================
// MAIN CLOCK
// ============================================================

void updateClock()
{
    String currentTime =
        getIrishTime();

    if (
        currentTime ==
        lastTime)
        return;

    lastTime =
        currentTime;

    tft.fillRect(
        82,
        52,
        76,
        24,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeText()
    );

    char timeText[6];

    snprintf(
        timeText,
        sizeof(timeText),
        "%c%c:%c%c",
        currentTime[0],
        currentTime[1],
        currentTime[3],
        currentTime[4]
    );

    tft.drawString(
        timeText,
        CX,
        64
    );
}

// ============================================================
// READ GPS
// ============================================================

void readGPS()
{
    while (GPS.available())
    {
        gps.encode(
            GPS.read()
        );
    }

    if (gps.speed.isValid())
        speedKmh =
            gps.speed.kmph();
    else
        speedKmh = 0.0;

    gpsFix =
        gps.location.isValid();

    if (gps.satellites.isValid())
        satellites =
            gps.satellites.value();

    if (gps.hdop.isValid())
        hdop =
            gps.hdop.hdop();

    calibrateSeaLevelPressure();

    updateAltitudeTrend();

    if (gps.location.isUpdated())
    {
        double lat =
            gps.location.lat();

        double lon =
            gps.location.lng();

        if (havePreviousPosition)
        {
            double segmentDistance =
                TinyGPSPlus::distanceBetween(
                    lastLat,
                    lastLon,
                    lat,
                    lon
                );

            if (
                segmentDistance >= 0.5 &&
                segmentDistance <= 100.0)
            {
                distanceKm +=
                    segmentDistance / 1000.0;

                odometerKm +=
                    segmentDistance / 1000.0;
            }
        }

        lastLat = lat;
        lastLon = lon;

        havePreviousPosition =
            true;
    }

    if (gps.speed.isValid())
    {
        if (
            speedKmh >
            maximumSpeed)
        {
            maximumSpeed =
                speedKmh;
        }
    }

    if (gps.course.isValid())
    {
        currentHeading =
            gps.course.deg();
    }
}

// ============================================================
// WEATHER TREND
// ============================================================

int calculateTrend(
    float current,
    float previous,
    float threshold)
{
    float difference =
        current - previous;

    if (
        difference >
        threshold)
        return 1;

    if (
        difference <
        -threshold)
        return -1;

    return 0;
}

// ============================================================
// READ BME280
// ============================================================

void readWeather()
{
    if (!bmeAvailable)
        return;

    unsigned long now =
        millis();

    if (
        now -
        lastWeatherReading <
        WEATHER_UPDATE_INTERVAL)
    {
        return;
    }

    lastWeatherReading =
        now;

    float newTemperature =
        bme.readTemperature() +
        TEMP_CALIBRATION_OFFSET;

    float newPressure =
        bme.readPressure() /
        100.0F;

    float newHumidity =
        bme.readHumidity();

    if (
        isnan(newTemperature) ||
        isnan(newPressure) ||
        isnan(newHumidity))
    {
        return;
    }

    if (!havePreviousWeather)
    {
        temperatureC =
            newTemperature;

        pressureHpa =
            newPressure;

        humidity =
            newHumidity;

        previousTemperature =
            temperatureC;

        previousPressure =
            pressureHpa;

        previousHumidity =
            humidity;

        havePreviousWeather =
            true;

        temperatureTrend = 0;
        pressureTrend = 0;
        humidityTrend = 0;

        return;
    }

    previousTemperature =
        temperatureC;

    previousPressure =
        pressureHpa;

    previousHumidity =
        humidity;

    temperatureC =
        newTemperature;

    pressureHpa =
        newPressure;

    humidity =
        newHumidity;

    temperatureTrend =
        calculateTrend(
            temperatureC,
            previousTemperature,
            TEMP_TREND_THRESHOLD
        );

    pressureTrend =
        calculateTrend(
            pressureHpa,
            previousPressure,
            PRESSURE_TREND_THRESHOLD
        );

    humidityTrend =
        calculateTrend(
            humidity,
            previousHumidity,
            HUMIDITY_TREND_THRESHOLD
        );

}

// ============================================================
// WEATHER ARROW
// ============================================================

void drawTrendArrow(
    int x,
    int y,
    int trend)
{
    uint16_t colour;

    if (trend > 0)
        colour = TFT_GREEN;
    else if (trend < 0)
        colour = TFT_YELLOW;
    else
        colour = themeSecondaryText();

    if (trend > 0)
    {
        tft.drawLine(
            x,
            y + 7,
            x,
            y - 7,
            colour
        );

        tft.drawLine(
            x,
            y - 7,
            x - 5,
            y - 2,
            colour
        );

        tft.drawLine(
            x,
            y - 7,
            x + 5,
            y - 2,
            colour
        );
    }
    else if (trend < 0)
    {
        tft.drawLine(
            x,
            y - 7,
            x,
            y + 7,
            colour
        );

        tft.drawLine(
            x,
            y + 7,
            x - 5,
            y + 2,
            colour
        );

        tft.drawLine(
            x,
            y + 7,
            x + 5,
            y + 2,
            colour
        );
    }
    else
    {
        tft.drawLine(
            x - 7,
            y,
            x + 7,
            y,
            colour
        );

        tft.drawLine(
            x + 7,
            y,
            x + 2,
            y - 4,
            colour
        );

        tft.drawLine(
            x + 7,
            y,
            x + 2,
            y + 4,
            colour
        );
    }
}

// ============================================================
// WEATHER PAGE VALUES
// ============================================================

void updateWeatherPage()
{
    if (!bmeAvailable)
    {
        tft.setTextDatum(
            middle_center
        );

        tft.setFont(
            &fonts::Font2
        );

        tft.setTextColor(
            TFT_RED
        );

        tft.drawString(
            "BME280 NOT FOUND",
            CX,
            115
        );

        return;
    }

    char text[32];

    tft.fillRect(
        80,
        38,
        132,
        24,
        themeBackground()
    );

    tft.fillRect(
        80,
        78,
        132,
        24,
        themeBackground()
    );

    tft.fillRect(
        80,
        118,
        132,
        24,
        themeBackground()
    );

    tft.fillRect(
        80,
        158,
        132,
        24,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        themeText()
    );

    snprintf(
        text,
        sizeof(text),
        "%.1f C",
        temperatureC
    );

    tft.drawString(
        text,
        137,
        50
    );

    drawTrendArrow(
        205,
        50,
        temperatureTrend
    );

    tft.setFont(
        &fonts::Font2
    );

    snprintf(
        text,
        sizeof(text),
        "%.0f hPa",
        pressureHpa
    );

    tft.drawString(
        text,
        137,
        90
    );

    drawTrendArrow(
        205,
        90,
        pressureTrend
    );

    snprintf(
        text,
        sizeof(text),
        "%.0f %%",
        humidity
    );

    tft.drawString(
        text,
        137,
        130
    );

    drawTrendArrow(
        205,
        130,
        humidityTrend
    );

    snprintf(
        text,
        sizeof(text),
        "%.0f m",
        pressureAltitudeMeters()
    );

    tft.drawString(
        text,
        137,
        170
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeLine()
    );

    tft.drawString(
        "BME280",
        CX,
        215
    );
}

// ============================================================
// WEATHER PAGE
// ============================================================

void drawWeatherPage()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "WEATHER",
        CX,
        25
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "TEMP",
        45,
        50
    );

    tft.drawString(
        "PRESSURE",
        55,
        90
    );

    tft.drawString(
        "HUMIDITY",
        55,
        130
    );

    tft.drawString(
        "ALT",
        45,
        170
    );

    updateWeatherPage();
}

// ============================================================
// AVERAGE SPEED
// ============================================================

void updateAverageSpeed()
{
    if (
        gps.speed.isValid() &&
        gpsFix)
    {
        speedSum +=
            speedKmh;

        speedSamples++;

        if (speedSamples > 0)
        {
            averageSpeed =
                speedSum /
                speedSamples;
        }
    }
}

// ============================================================
// SPEED PAGE
// ============================================================

void drawSpeedPage()
{
    drawStaticScreen();

    updateSpeed();

    updateMainDistance();

    speedBarStarted = false;
    previousBarSpeed = 0;

    drawSpeedBar();
}

// ============================================================
// TRIP PAGE STATIC
// ============================================================

void drawTripPageStatic()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "TRIP",
        CX,
        24
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "DISTANCE",
        CX,
        52
    );

    tft.drawString(
        "CURRENT",
        CX,
        102
    );

    tft.drawString(
        "MAX",
        70,
        153
    );

    tft.drawString(
        "ODO",
        CX,
        153
    );

    tft.drawString(
        "AVG",
        170,
        153
    );

    tft.drawString(
        "SAT",
        CX,
        205
    );
}

// ============================================================
// TRIP PAGE VALUES
// ============================================================

void updateTripPage()
{
    char text[30];

    tft.fillRect(
        45,
        60,
        150,
        25,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        themeText()
    );

    snprintf(
        text,
        sizeof(text),
        "%.2f km",
        distanceKm
    );

    tft.drawString(
        text,
        CX,
        72
    );

    tft.fillRect(
        45,
        110,
        150,
        25,
        themeBackground()
    );

    snprintf(
        text,
        sizeof(text),
        "%.1f km/h",
        speedKmh
    );

    tft.drawString(
        text,
        CX,
        122
    );

    tft.fillRect(
        40,
        162,
        60,
        25,
        themeBackground()
    );

    tft.setFont(
        &fonts::Font2
    );

    snprintf(
        text,
        sizeof(text),
        "%.1f",
        maximumSpeed
    );

    tft.drawString(
        text,
        70,
        176
    );

    tft.fillRect(
        95,
        162,
        50,
        25,
        themeBackground()
    );

    snprintf(
        text,
        sizeof(text),
        "%.0f",
        odometerKm
    );

    tft.drawString(
        text,
        CX,
        176
    );

    tft.fillRect(
        140,
        162,
        60,
        25,
        themeBackground()
    );

    snprintf(
        text,
        sizeof(text),
        "%.1f",
        averageSpeed
    );

    tft.drawString(
        text,
        170,
        176
    );

    tft.fillRect(
        95,
        210,
        50,
        25,
        themeBackground()
    );

    tft.setTextColor(
        getGPSColour()
    );

    snprintf(
        text,
        sizeof(text),
        "%d",
        satellites
    );

    tft.drawString(
        text,
        CX,
        222
    );
}

// ============================================================
// GPS PAGE STATIC
// ============================================================

void drawGPSPageStatic()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "GPS",
        CX,
        20
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "STATUS",
        65,
        49
    );

    tft.drawString(
        "SAT",
        175,
        49
    );

    tft.drawString(
        "LAT",
        52,
        88
    );

    tft.drawString(
        "LON",
        52,
        116
    );

    tft.drawString(
        "ALT",
        52,
        144
    );

    tft.drawString(
        "HDOP",
        52,
        172
    );

    tft.drawString(
        "TIME",
        52,
        200
    );
}

// ============================================================
// GPS PAGE VALUES
// ============================================================

void updateGPSPage()
{
    char text[40];

    tft.fillRect(
        25,
        56,
        80,
        20,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        getGPSColour()
    );

    if (gpsFix)
        tft.drawString(
            "FIX",
            65,
            66
        );
    else
        tft.drawString(
            "NO FIX",
            65,
            66
        );

    tft.fillRect(
        145,
        56,
        60,
        20,
        themeBackground()
    );

    tft.setTextColor(
        getGPSColour()
    );

    snprintf(
        text,
        sizeof(text),
        "%d",
        satellites
    );

    tft.drawString(
        text,
        175,
        66
    );

    tft.fillRect(
        72,
        78,
        140,
        22,
        themeBackground()
    );

    tft.setTextColor(
        themeText()
    );

    if (gps.location.isValid())
    {
        snprintf(
            text,
            sizeof(text),
            "%.5f",
            gps.location.lat()
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            "---"
        );
    }

    tft.drawString(
        text,
        150,
        88
    );

    tft.fillRect(
        72,
        106,
        140,
        22,
        themeBackground()
    );

    if (gps.location.isValid())
    {
        snprintf(
            text,
            sizeof(text),
            "%.5f",
            gps.location.lng()
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            "---"
        );
    }

    tft.drawString(
        text,
        150,
        116
    );

    tft.fillRect(
        72,
        134,
        140,
        22,
        themeBackground()
    );

    if (gps.altitude.isValid())
    {
        snprintf(
            text,
            sizeof(text),
            "%.1f m",
            gps.altitude.meters()
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            "---"
        );
    }

    tft.drawString(
        text,
        150,
        144
    );

    tft.fillRect(
        72,
        162,
        140,
        22,
        themeBackground()
    );

    if (gps.hdop.isValid())
    {
        snprintf(
            text,
            sizeof(text),
            "%.1f",
            hdop
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            "---"
        );
    }

    tft.drawString(
        text,
        150,
        172
    );

    tft.fillRect(
        72,
        190,
        140,
        22,
        themeBackground()
    );

    tft.setTextColor(
        themeText()
    );

    tft.drawString(
        getIrishTime(),
        150,
        200
    );

    tft.fillRect(
        70,
        212,
        100,
        24,
        themeBackground()
    );

    snprintf(
        text,
        sizeof(text),
        "%.1f km/h",
        speedKmh
    );

    tft.drawString(
        text,
        CX,
        224
    );
}

// ============================================================
// CLOCK FACE
// ============================================================

void createClockFace()
{
    clockSprite.createSprite(
        240,
        240
    );

    uint16_t bg =
        themeBackground();

    uint16_t mainText =
        themeText();

    uint16_t line =
        themeLine();

    clockSprite.fillSprite(
        bg
    );

    clockSprite.drawCircle(
        CX,
        CY,
        118,
        line
    );

    clockSprite.drawCircle(
        CX,
        CY,
        104,
        line
    );

    for (
        int i = 0;
        i < 60;
        i++)
    {
        float angle =
            (i * 6.0f - 90.0f) *
            DEG_TO_RAD;

        int outer = 112;

        int inner =
            (i % 5 == 0)
            ? 101
            : 107;

        int x1 =
            CX +
            cos(angle) * inner;

        int y1 =
            CY +
            sin(angle) * inner;

        int x2 =
            CX +
            cos(angle) * outer;

        int y2 =
            CY +
            sin(angle) * outer;

        if (i % 5 == 0)
        {
            clockSprite.drawLine(
                x1,
                y1,
                x2,
                y2,
                mainText
            );
        }
        else
        {
            clockSprite.drawPixel(
                x2,
                y2,
                line
            );
        }
    }

    clockSprite.fillCircle(
        CX,
        13,
        3,
        mainText
    );

    clockSprite.fillCircle(
        227,
        CY,
        3,
        mainText
    );

    clockSprite.fillCircle(
        CX,
        227,
        3,
        mainText
    );

    clockSprite.fillCircle(
        13,
        CY,
        3,
        mainText
    );
}

// ============================================================
// CLOCK SECOND POSITION
// ============================================================

void getClockSecondPosition(
    int second,
    int &x,
    int &y)
{
    float angle =
        -90.0f +
        second * 6.0f;

    float rad =
        angle * DEG_TO_RAD;

    int radius = 114;

    x =
        CX +
        cos(rad) * radius;

    y =
        CY +
        sin(rad) * radius;
}

// ============================================================
// DRAW CLOCK SECOND
// ============================================================

void drawClockSecondDot(
    int second)
{
    int x;
    int y;

    getClockSecondPosition(
        second,
        x,
        y
    );

    tft.fillCircle(
        x,
        y,
        4,
        TFT_GREEN
    );
}

// ============================================================
// RESTORE CLOCK SECOND
// ============================================================

void restoreClockSecondDot(
    int second)
{
    int x;
    int y;

    getClockSecondPosition(
        second,
        x,
        y
    );

    for (
        int yy = -5;
        yy <= 5;
        yy++)
    {
        for (
            int xx = -5;
            xx <= 5;
            xx++)
        {
            int px =
                x + xx;

            int py =
                y + yy;

            if (
                px >= 0 &&
                px < 240 &&
                py >= 0 &&
                py < 240)
            {
                uint16_t colour =
                    clockSprite.readPixel(
                        px,
                        py
                    );

                tft.drawPixel(
                    px,
                    py,
                    colour
                );
            }
        }
    }
}

// ============================================================
// CLOCK TIME
// ============================================================

void drawClockTime()
{
    String currentTime =
        getIrishTime();

    if (
        currentTime ==
        "--:--:--")
        return;

    char timeText[6];

    snprintf(
        timeText,
        sizeof(timeText),
        "%c%c:%c%c",
        currentTime[0],
        currentTime[1],
        currentTime[3],
        currentTime[4]
    );

    tft.setTextDatum(
        middle_center
    );

    tft.fillRect(
        25,
        78,
        190,
        55,
        themeBackground()
    );

    tft.fillRect(
        65,
        51,
        110,
        24,
        themeBackground()
    );

    if (bmeAvailable)
    {
        char temperatureText[16];

        snprintf(
            temperatureText,
            sizeof(temperatureText),
            "%.1f C",
            temperatureC
        );

        tft.setTextColor(
            themeSecondaryText()
        );

        tft.drawString(
            temperatureText,
            CX,
            65,
            4
        );
    }

    tft.setTextColor(
        themeText()
    );

    tft.drawString(
        timeText,
        CX,
        105,
        7
    );
}

// ============================================================
// CLOCK DATE
// ============================================================

void drawClockDate()
{
    if (!gps.date.isValid())
        return;

    const char* days[] =
    {
        "SUN",
        "MON",
        "TUE",
        "WED",
        "THU",
        "FRI",
        "SAT"
    };

    const char* months[] =
    {
        "JAN",
        "FEB",
        "MAR",
        "APR",
        "MAY",
        "JUN",
        "JUL",
        "AUG",
        "SEP",
        "OCT",
        "NOV",
        "DEC"
    };

    int dow =
        dayOfWeek(
            gps.date.year(),
            gps.date.month(),
            gps.date.day()
        );

    char dateText[24];

    snprintf(
        dateText,
        sizeof(dateText),
        "%s  %02d %s",
        days[dow],
        gps.date.day(),
        months[
            gps.date.month() - 1
        ]
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        dateText,
        CX,
        158,
        2
    );
}

// ============================================================
// CLOCK PAGE
// ============================================================

void drawClockPage()
{
    tft.fillScreen(
        themeBackground()
    );

    createClockFace();

    clockSprite.pushSprite(
        0,
        0
    );

    drawClockTime();
    drawClockDate();

    tft.setTextDatum(
        middle_center
    );

    tft.setTextColor(
        getGPSColour()
    );

    char satText[16];

    snprintf(
        satText,
        sizeof(satText),
        "%d SAT",
        satellites
    );

    tft.drawString(
        satText,
        CX,
        190,
        2
    );

    if (gpsFix)
    {
        tft.drawString(
            "GPS TIME",
            CX,
            208,
            2
        );
    }
    else
    {
        tft.drawString(
            "WAITING GPS",
            CX,
            208,
            2
        );
    }

    if (gps.time.isValid())
    {
        previousClockSecond =
            gps.time.second();

        drawClockSecondDot(
            previousClockSecond
        );
    }
}

// ============================================================
// COMPASS DIRECTION
// ============================================================

const char* getCompassDirection(
    float heading)
{
    if (
        heading >= 337.5 ||
        heading < 22.5)
        return "N";

    if (heading < 67.5)
        return "NE";

    if (heading < 112.5)
        return "E";

    if (heading < 157.5)
        return "SE";

    if (heading < 202.5)
        return "S";

    if (heading < 247.5)
        return "SW";

    if (heading < 292.5)
        return "W";

    return "NW";
}

// ============================================================
// COMPASS MARKER
// ============================================================

void drawCompassMarker(
    float bearing,
    float heading,
    const char* label,
    bool major)
{
    float relative =
        bearing - heading;

    while (relative > 180.0)
        relative -= 360.0;

    while (relative < -180.0)
        relative += 360.0;

    float rad =
        radians(
            relative - 90.0
        );

    const int radius =
        major ? 91 : 92;

    int x =
        CX +
        cos(rad) * radius;

    int y =
        CY +
        sin(rad) * radius;

    compassSprite.setTextDatum(
        middle_center
    );

    if (major)
    {
        compassSprite.setFont(
            &fonts::Font4
        );

        compassSprite.setTextColor(
            themeText()
        );
    }
    else
    {
        compassSprite.setFont(
            &fonts::Font2
        );

        compassSprite.setTextColor(
            themeSecondaryText()
        );
    }

    compassSprite.drawString(
        label,
        x,
        y
    );
}

// ============================================================
// COMPASS TICK
// ============================================================

void drawCompassTick(
    float bearing,
    float heading,
    bool major)
{
    float relative =
        bearing - heading;

    while (relative > 180.0)
        relative -= 360.0;

    while (relative < -180.0)
        relative += 360.0;

    float rad =
        radians(
            relative - 90.0
        );

    int outer = 110;

    int inner =
        major ? 101 : 105;

    int x1 =
        CX +
        cos(rad) * inner;

    int y1 =
        CY +
        sin(rad) * inner;

    int x2 =
        CX +
        cos(rad) * outer;

    int y2 =
        CY +
        sin(rad) * outer;

    compassSprite.drawLine(
        x1,
        y1,
        x2,
        y2,
        major
        ? themeText()
        : themeLine()
    );
}

// ============================================================
// COMPASS PAGE
// ============================================================

void drawCompassPage()
{
    if (
        compassSprite.width() != 240 ||
        compassSprite.height() != 240)
    {
        compassSprite.createSprite(
            240,
            240
        );
    }

    compassSprite.fillSprite(
        themeBackground()
    );

    compassSprite.drawCircle(
        CX,
        CY,
        116,
        themeLine()
    );

    compassSprite.drawCircle(
        CX,
        CY,
        111,
        themeLine()
    );

    float heading =
        currentHeading;

    drawCompassTick(
        0,
        heading,
        true
    );

    drawCompassTick(
        45,
        heading,
        false
    );

    drawCompassTick(
        90,
        heading,
        true
    );

    drawCompassTick(
        135,
        heading,
        false
    );

    drawCompassTick(
        180,
        heading,
        true
    );

    drawCompassTick(
        225,
        heading,
        false
    );

    drawCompassTick(
        270,
        heading,
        true
    );

    drawCompassTick(
        315,
        heading,
        false
    );

    for (
        int bearing = 0;
        bearing < 360;
        bearing += 15)
    {
        if (bearing % 45 != 0)
        {
            drawCompassTick(
                bearing,
                heading,
                false
            );
        }
    }

    drawCompassMarker(
        0,
        heading,
        "N",
        true
    );

    drawCompassMarker(
        45,
        heading,
        "NE",
        false
    );

    drawCompassMarker(
        90,
        heading,
        "E",
        true
    );

    drawCompassMarker(
        135,
        heading,
        "SE",
        false
    );

    drawCompassMarker(
        180,
        heading,
        "S",
        true
    );

    drawCompassMarker(
        225,
        heading,
        "SW",
        false
    );

    drawCompassMarker(
        270,
        heading,
        "W",
        true
    );

    drawCompassMarker(
        315,
        heading,
        "NW",
        false
    );

    compassSprite.fillTriangle(
        CX,
        31,
        CX - 6,
        43,
        CX + 6,
        43,
        TFT_RED
    );

    compassSprite.setTextDatum(
        middle_center
    );

    if (
        gps.course.isValid() &&
        gpsFix &&
        speedKmh >= 1.0)
    {
        char headingText[12];

        snprintf(
            headingText,
            sizeof(headingText),
            "%03d",
            (int)round(
                currentHeading
            )
        );

        compassSprite.setFont(
            &fonts::Font7
        );

        compassSprite.setTextColor(
            themeText()
        );

        compassSprite.drawString(
            headingText,
            CX,
            112
        );

        compassSprite.setFont(
            &fonts::Font4
        );

        compassSprite.setTextColor(
            TFT_GREEN
        );

        compassSprite.drawString(
            getCompassDirection(
                currentHeading
            ),
            CX,
            145
        );
    }
    else
    {
        compassSprite.setFont(
            &fonts::Font7
        );

        compassSprite.setTextColor(
            themeLine()
        );

        compassSprite.drawString(
            "---",
            CX,
            112
        );

        compassSprite.setFont(
            &fonts::Font2
        );

        compassSprite.setTextColor(
            TFT_YELLOW
        );

        if (!gpsFix)
        {
            compassSprite.drawString(
                "NO GPS",
                CX,
                145
            );
        }
        else
        {
            compassSprite.drawString(
                "MOVE TO HEADING",
                CX,
                145
            );
        }
    }

    compassSprite.setFont(
        &fonts::Font2
    );

    compassSprite.setTextColor(
        getGPSColour()
    );

    char satText[16];

    snprintf(
        satText,
        sizeof(satText),
        "%d SAT",
        satellites
    );

    compassSprite.drawString(
        satText,
        CX,
        181
    );

    if (gpsFix)
    {
        compassSprite.drawString(
            "GPS HEADING",
            CX,
            202
        );
    }
    else
    {
        compassSprite.drawString(
            "WAITING GPS",
            CX,
            202
        );
    }

    compassSprite.setTextColor(
        themeSecondaryText()
    );

    char speedText[20];

    snprintf(
        speedText,
        sizeof(speedText),
        "%.1f km/h",
        speedKmh
    );

    compassSprite.drawString(
        speedText,
        CX,
        222
    );

    compassSprite.pushSprite(
        0,
        0
    );
}

// ============================================================
// BAROMETER ANGLE
// ============================================================

float barometerAngle(
    float pressure)
{
    pressure =
        constrain(
            pressure,
            BARO_MIN,
            BARO_MAX
        );

    float fraction =
        (pressure - BARO_MIN) /
        (BARO_MAX - BARO_MIN);

    return
        BARO_START_ANGLE +
        fraction *
        (BARO_END_ANGLE -
         BARO_START_ANGLE);
}

// ============================================================
// DRAW BAROMETER TICK
// ============================================================

void drawBarometerTick(
    float angle,
    int inner,
    int outer,
    uint16_t colour)
{
    float rad =
        radians(
            angle - 90.0
        );

    int x1 =
        BARO_CX +
        cos(rad) * inner;

    int y1 =
        BARO_CY +
        sin(rad) * inner;

    int x2 =
        BARO_CX +
        cos(rad) * outer;

    int y2 =
        BARO_CY +
        sin(rad) * outer;

    tft.drawLine(
        x1,
        y1,
        x2,
        y2,
        colour
    );
}

// ============================================================
// DRAW BAROMETER NEEDLE
// ============================================================

void drawBarometerNeedle(
    float pressure,
    uint16_t colour)
{
    float angle =
        barometerAngle(
            pressure
        );

    float rad =
        radians(
            angle - 90.0
        );

    int tipRadius = 83;

    int x =
        BARO_CX +
        cos(rad) *
        tipRadius;

    int y =
        BARO_CY +
        sin(rad) *
        tipRadius;

    tft.drawLine(
        BARO_CX,
        BARO_CY,
        x,
        y,
        colour
    );

    int backX =
        BARO_CX -
        cos(rad) * 15;

    int backY =
        BARO_CY -
        sin(rad) * 15;

    tft.drawLine(
        BARO_CX,
        BARO_CY,
        backX,
        backY,
        colour
    );

    tft.fillCircle(
        BARO_CX,
        BARO_CY,
        5,
        colour
    );
}

// ============================================================
// BAROMETER STATIC FACE
// ============================================================

void drawBarometerFace()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "",
        CX,
        18
    );

    tft.drawCircle(
        BARO_CX,
        BARO_CY,
        114,
        themeLine()
    );

    tft.drawCircle(
        BARO_CX,
        BARO_CY,
        111,
        themeLine()
    );

    for (
        int pressure = 980;
        pressure <= 1040;
        pressure += 2)
    {
        float angle =
            barometerAngle(
                pressure
            );

        bool major =
            (pressure % 10 == 0);

        drawBarometerTick(
            angle,
            major ? 91 : 98,
            108,
            major
            ? themeText()
            : themeLine()
        );
    }

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    const int labels[] =
    {
        980,
        990,
        1000,
        1010,
        1020,
        1030,
        1040
    };

    for (
        int i = 0;
        i < 7;
        i++)
    {
        float angle =
            barometerAngle(
                labels[i]
            );

        float rad =
            radians(
                angle - 90.0
            );

        int radius = 78;

        int x =
            BARO_CX +
            cos(rad) * radius;

        int y =
            BARO_CY +
            sin(rad) * radius;

        tft.drawNumber(
            labels[i],
            x,
            y
        );
    }

    tft.fillCircle(
        BARO_CX,
        BARO_CY,
        48,
        themeBackground()
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.drawString(
        "PRESSURE",
        BARO_CX,
        92
    );

    tft.setTextColor(
        themeText()
    );

    tft.setFont(
        &fonts::Font4
    );

    char tempText[16];

    tft.setTextColor(
        TFT_GREY
    );
    snprintf(
        tempText,
        sizeof(tempText),
        "%.1f C",
        temperatureC
    );

    tft.drawString(
        tempText,
        BARO_CX,
        200
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    char pressureText[20];

    snprintf(
        pressureText,
        sizeof(pressureText),
        "%.0f hPa",
        pressureHpa
    );

    tft.drawString(
        pressureText,
        BARO_CX,
        145
    );

    previousBarometerPressure =
        pressureHpa;

    drawBarometerNeedle(
        pressureHpa,
        TFT_GREEN
    );

    barometerStarted =
        true;
}

// ============================================================
// UPDATE BAROMETER
// ============================================================

void updateBarometerPage()
{
    if (!bmeAvailable)
    {
        tft.setTextDatum(
            middle_center
        );

        tft.setFont(
            &fonts::Font2
        );

        tft.setTextColor(
            TFT_RED
        );

        tft.fillRect(
            60,
            100,
            125,
            50,
            themeBackground()
        );

        tft.drawString(
            "BME280 NOT FOUND",
            CX,
            120
        );

        return;
    }

    tft.fillRect(
        75,
        178,
        80,
        30,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "PRESSURE",
        BARO_CX,
        92
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        themeText()
    );

    char tempText[16];
    tft.setTextColor(
        TFT_GREY
    );
    snprintf(
        tempText,
        sizeof(tempText),
        "%.1f C",
        temperatureC
    );

    tft.drawString(
        tempText,
        BARO_CX,
        200
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    char pressureText[20];

    snprintf(
        pressureText,
        sizeof(pressureText),
        "%.0f hPa",
        pressureHpa
    );

    tft.drawString(
        pressureText,
        BARO_CX,
        145
    );

    if (
        previousBarometerPressure >=
        BARO_MIN)
    {
        float oldAngle =
            barometerAngle(
                previousBarometerPressure
            );

        float oldRad =
            radians(
                oldAngle - 90.0
            );

        int oldX =
            BARO_CX +
            cos(oldRad) * 83;

        int oldY =
            BARO_CY +
            sin(oldRad) * 83;

        tft.drawLine(
            BARO_CX,
            BARO_CY,
            oldX,
            oldY,
            themeBackground()
        );

        int oldBackX =
            BARO_CX -
            cos(oldRad) * 15;

        int oldBackY =
            BARO_CY -
            sin(oldRad) * 15;

        tft.drawLine(
            BARO_CX,
            BARO_CY,
            oldBackX,
            oldBackY,
            themeBackground()
        );
    }

    drawBarometerNeedle(
        pressureHpa,
        TFT_RED
    );

    previousBarometerPressure =
        pressureHpa;
}

// ============================================================
// SYSTEM PAGE
// ============================================================

void drawSystemPage()
{
    tft.fillScreen(
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font4
    );

    tft.setTextColor(
        TFT_GREEN
    );

    tft.drawString(
        "SYSTEM",
        CX,
        24
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeSecondaryText()
    );

    tft.drawString(
        "--",
        55,
        65
    );

    tft.drawString(
        "ESP TEMP",
        55,
        105
    );

    tft.drawString(
        "GPS SAT",
        55,
        145
    );

    tft.drawString(
        "HDOP",
        65,
        185
    );

    tft.drawString(
        "UP",
        75,
        215
    );

    updateSystemPage();
}

// ============================================================
// ESP32 TEMPERATURE
// ============================================================

float getESPTemperature()
{
#ifdef ESP_ARDUINO_VERSION_MAJOR
    return temperatureRead();
#else
    return temperatureRead();
#endif
}

// ============================================================
// SYSTEM PAGE VALUES
// ============================================================

void updateSystemPage()
{
    char text[32];

    tft.fillRect(
        105,
        53,
        105,
        24,
        themeBackground()
    );

    tft.fillRect(
        105,
        93,
        105,
        24,
        themeBackground()
    );

    tft.fillRect(
        105,
        133,
        105,
        24,
        themeBackground()
    );

    tft.fillRect(
        105,
        173,
        105,
        24,
        themeBackground()
    );

    tft.fillRect(
        95,
        203,
        125,
        24,
        themeBackground()
    );

    tft.setTextDatum(
        middle_center
    );

    tft.setFont(
        &fonts::Font2
    );

    tft.setTextColor(
        themeText()
    );

    // Battery
    tft.drawString(
        "--",
        155,
        65
    );

    // ESP temperature
    float espTemp =
        getESPTemperature();

    snprintf(
        text,
        sizeof(text),
        "%.1f C",
        espTemp
    );

    tft.drawString(
        text,
        155,
        105
    );

    // GPS satellites
    tft.setTextColor(
        getGPSColour()
    );

    snprintf(
        text,
        sizeof(text),
        "%d",
        satellites
    );

    tft.drawString(
        text,
        155,
        145
    );

    // HDOP
    tft.setTextColor(
        themeText()
    );

    if (gps.hdop.isValid())
    {
        snprintf(
            text,
            sizeof(text),
            "%.1f",
            hdop
        );
    }
    else
    {
        snprintf(
            text,
            sizeof(text),
            "---"
        );
    }

    tft.drawString(
        text,
        155,
        185
    );

    // Uptime
    unsigned long totalSeconds =
        millis() / 1000;

    unsigned long hours =
        totalSeconds / 3600;

    unsigned long minutes =
        (totalSeconds % 3600) / 60;

    unsigned long seconds =
        totalSeconds % 60;

    snprintf(
        text,
        sizeof(text),
        "%02lu:%02lu:%02lu",
        hours,
        minutes,
        seconds
    );

    tft.drawString(
        text,
        CX,
        215
    );
}

// ============================================================
// TOGGLE LIGHT THEME
// ============================================================

void toggleLightTheme()
{
    lightTheme =
        !lightTheme;

    Serial.print(
        "Display theme: "
    );

    if (lightTheme)
        Serial.println("LIGHT");
    else
        Serial.println("DARK");

    // Reset dynamic display states
    speedBarStarted =
        false;

    previousBarSpeed =
        0;

    barometerStarted =
        false;

    previousBarometerPressure =
        -999.0;

    previousClockSecond =
        -1;

    lastTime =
        "";

    // Force complete redraw
    pageNeedsRedraw =
        true;

    tft.fillScreen(
        themeBackground()
    );

    drawCurrentPage();
}

// ============================================================
// DRAW CURRENT PAGE
// ============================================================

void drawCurrentPage()
{
    pageNeedsRedraw =
        false;

    if (currentPage == 0)
    {
        drawSpeedPage();
    }
    else if (currentPage == 1)
    {
        drawTripPageStatic();
        updateTripPage();
    }
    else if (currentPage == 2)
    {
        drawGPSPageStatic();
        updateGPSPage();
    }
    else if (currentPage == 3)
    {
        drawCompassPage();
    }
    else if (currentPage == 4)
    {
        drawClockPage();
    }
    else if (currentPage == 5)
    {
        drawWeatherPage();
    }
    else if (currentPage == 6)
    {
        barometerStarted = false;
        previousBarometerPressure = -999;
        drawBarometerFace();
    }
    else if (currentPage == 7)
    {
        drawSystemPage();
    }
    else if (currentPage == 8)
    {
        drawHistoryPage();
    }

    previousPage =
        currentPage;

    pageEnteredTime =
        millis();
}

// ============================================================
// CAPACITIVE TOUCH
// ============================================================

uint16_t readTouchAverage(uint8_t pin)
{
    uint32_t total = 0;

    for (int i = 0; i < TOUCH_CALIBRATION_SAMPLES; i++)
    {
        total += touchRead(pin);
        delay(5);
    }

    return (uint16_t)(total / TOUCH_CALIBRATION_SAMPLES);
}

void calibrateTouch()
{
    Serial.println();
    Serial.println("CAPACITIVE TOUCH CALIBRATION");
    Serial.println("Leave GPIO 2 and GPIO 3 untouched...");

    delay(500);

    touchBaseline1 = readTouchAverage(TOUCH_BUTTON_1);
    touchBaseline2 = readTouchAverage(TOUCH_BUTTON_2);

    Serial.print("Touch GPIO 2 baseline: ");
    Serial.println(touchBaseline1);

    Serial.print("Touch GPIO 3 baseline: ");
    Serial.println(touchBaseline2);

    Serial.println("Touch calibration complete.");
    Serial.println();
}

bool readTouchButton1()
{
    uint16_t value = touchRead(TOUCH_BUTTON_1);

    long difference =
        (long)value -
        (long)touchBaseline1;

    if (difference < 0)
        difference = -difference;

    touchButton1State =
        difference >= TOUCH_DEVIATION_THRESHOLD;

    return touchButton1State;
}

bool readTouchButton2()
{
    uint16_t value = touchRead(TOUCH_BUTTON_2);

    long difference =
        (long)value -
        (long)touchBaseline2;

    if (difference < 0)
        difference = -difference;

    touchButton2State =
        difference >= TOUCH_DEVIATION_THRESHOLD;

    return touchButton2State;
}
// ============================================================
// BUTTON 1
// ============================================================

void checkButton1()
{
    unsigned long now = millis();

    // --------------------------------------------------------
    // PHYSICAL BUTTON 1
    // --------------------------------------------------------

    bool physicalState =
        digitalRead(BUTTON_1);

    if (
        physicalState == LOW &&
        lastButton1State == HIGH)
    {
        if (
            now -
            lastButton1Change >
            BUTTON_DEBOUNCE)
        {
            button1PressStart = now;
            button1Held = true;
            tripResetTriggered = false;

            lastButton1Change = now;
        }
    }

    // Physical Button 1 long press = reset trip
    if (
        physicalState == LOW &&
        button1Held &&
        !tripResetTriggered)
    {
        if (
            now -
            button1PressStart >=
            TRIP_RESET_HOLD_TIME)
        {
            tripResetTriggered = true;
            resetTripData();
            pageNeedsRedraw = true;
        }
    }

    // Physical Button 1 released
    if (
        physicalState == HIGH &&
        lastButton1State == LOW)
    {
        if (
            button1Held &&
            !tripResetTriggered)
        {
            unsigned long duration =
                now -
                button1PressStart;

            if (
                duration <
                TRIP_RESET_HOLD_TIME)
            {
                if (currentPage == 0)
                    currentPage = 1;
                else if (currentPage == 1)
                    currentPage = 2;
                else if (currentPage == 2)
                    currentPage = 3;
                else if (currentPage == 3)
                    currentPage = 4;
                else if (currentPage == 4)
                    currentPage = 8;
                else if (currentPage == 8)
                    currentPage = 0;
                else
                    currentPage = 0;

                pageNeedsRedraw = true;
                pageEnteredTime = now;
            }
        }

        button1Held = false;
        tripResetTriggered = false;
    }

    lastButton1State =
        physicalState;

    // --------------------------------------------------------
    // GPIO 2 TOUCH
    // SHORT PRESS ONLY
    // --------------------------------------------------------

    bool touchState =
        readTouchButton1();

    if (!touchState)
    {
        // Reset touch lock when finger is removed
        touchButton1Handled = false;
    }
    else if (
        !touchButton1Handled &&
        physicalState == HIGH)
    {
        // First detection of touch
        touchButton1Handled = true;

        if (currentPage == 0)
            currentPage = 1;
        else if (currentPage == 1)
            currentPage = 2;
        else if (currentPage == 2)
            currentPage = 3;
        else if (currentPage == 3)
            currentPage = 4;
        else if (currentPage == 4)
            currentPage = 8;
        else if (currentPage == 8)
            currentPage = 0;
        else
            currentPage = 0;

        pageNeedsRedraw = true;
        pageEnteredTime = now;
    }
}

// ============================================================
// BUTTON 2
// ============================================================

void checkButton2()
{
    unsigned long now = millis();

    // --------------------------------------------------------
    // PHYSICAL BUTTON 2
    // --------------------------------------------------------

    bool physicalState =
        digitalRead(BUTTON_2);

    if (
        physicalState == LOW &&
        lastButton2State == HIGH)
    {
        if (
            now -
            lastButton2Change >
            BUTTON_DEBOUNCE)
        {
            button2PressStart = now;
            button2Held = true;
            themeTriggered = false;

            lastButton2Change = now;
        }
    }

    // Physical Button 2 long press = theme toggle
    if (
        physicalState == LOW &&
        button2Held &&
        !themeTriggered)
    {
        if (
            now -
            button2PressStart >=
            THEME_HOLD_TIME)
        {
            themeTriggered = true;
            toggleLightTheme();
        }
    }

    // Physical Button 2 released
    if (
        physicalState == HIGH &&
        lastButton2State == LOW)
    {
        if (button2Held)
        {
            unsigned long duration =
                now -
                button2PressStart;

            if (
                duration <
                THEME_HOLD_TIME)
            {
                if (currentPage == 4)
                    currentPage = 5;
                else if (currentPage == 5)
                    currentPage = 6;
                else if (currentPage == 6)
                    currentPage = 7;
                else if (currentPage == 7)
                    currentPage = 4;
                else
                    currentPage = 4;

                pageNeedsRedraw = true;
                pageEnteredTime = now;
            }
        }

        button2Held = false;
        themeTriggered = false;
    }

    lastButton2State =
        physicalState;

    // --------------------------------------------------------
    // GPIO 3 TOUCH
    // SHORT PRESS ONLY
    // --------------------------------------------------------

    bool touchState =
        readTouchButton2();

    if (!touchState)
    {
        // Reset touch lock when finger is removed
        touchButton2Handled = false;
    }
    else if (
        !touchButton2Handled &&
        physicalState == HIGH)
    {
        // First detection of touch
        touchButton2Handled = true;

        if (currentPage == 4)
            currentPage = 5;
        else if (currentPage == 5)
            currentPage = 6;
        else if (currentPage == 6)
            currentPage = 7;
        else if (currentPage == 7)
            currentPage = 4;
        else
            currentPage = 4;

        pageNeedsRedraw = true;
        pageEnteredTime = now;
    }
}
// ============================================================
// PAGE TIMEOUT
// ============================================================

void checkPageTimeout()
{
    // No timeout.
}

// ============================================================
// GAUGE MODE
// ============================================================

void updateGaugeMode()
{
    if (currentPage != 0)
        return;

    bool newMode =
        speedKmh > 35.0;

    if (
        newMode !=
        highSpeedMode)
    {
        highSpeedMode =
            newMode;

        speedBarStarted =
            false;

        previousBarSpeed =
            0;

        drawSpeedPage();
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    // --------------------------------------------------------
    // BACKLIGHT
    // --------------------------------------------------------

    pinMode(
        TFT_BL,
        OUTPUT
    );

    digitalWrite(
        TFT_BL,
        HIGH
    );

    // --------------------------------------------------------
    // BUTTONS
    // --------------------------------------------------------

    pinMode(
        BUTTON_1,
        INPUT_PULLUP
    );

    pinMode(
        BUTTON_2,
        INPUT_PULLUP
    );

    // --------------------------------------------------------
    // CAPACITIVE TOUCH
    // --------------------------------------------------------
    // GPIO 2 and GPIO 3 are touch-capable on this ESP32-S3.
    // Calibrate while the pads are untouched.
    calibrateTouch();

    // --------------------------------------------------------
    // DISPLAY
    // --------------------------------------------------------

    tft.init();

    tft.setRotation(
        2
    );

    tft.setTextWrap(
        false
    );

    showSplashScreen();

    // --------------------------------------------------------
    // I2C / BME280
    // --------------------------------------------------------

    Wire.begin(
        BME_SDA,
        BME_SCL
    );

    delay(100);

    if (
        bme.begin(
            0x76,
            &Wire))
    {
        bmeAvailable =
            true;

        Serial.println(
            "BME280 found at 0x76"
        );
    }
    else if (
        bme.begin(
            0x77,
            &Wire))
    {
        bmeAvailable =
            true;

        Serial.println(
            "BME280 found at 0x77"
        );
    }
    else
    {
        bmeAvailable =
            false;

        Serial.println(
            "BME280 NOT FOUND"
        );
    }

    // --------------------------------------------------------
    // GPS
    // --------------------------------------------------------

    GPS.begin(
        9600,
        SERIAL_8N1,
        GPS_RX_PIN,
        GPS_TX_PIN
    );

    // --------------------------------------------------------
    // FIRST WEATHER READING
    // --------------------------------------------------------

    if (bmeAvailable)
    {
        temperatureC =
            bme.readTemperature() +
            TEMP_CALIBRATION_OFFSET;

        pressureHpa =
            bme.readPressure() /
            100.0F;

        humidity =
            bme.readHumidity();

        previousTemperature =
            temperatureC;

        previousPressure =
            pressureHpa;

        previousHumidity =
            humidity;

        havePreviousWeather =
            true;

        temperatureTrend = 0;
        pressureTrend = 0;
        humidityTrend = 0;
    }

    // --------------------------------------------------------
    // START PAGE
    // --------------------------------------------------------

    currentPage =
        0;

    loadTripData();

    pageEnteredTime =
        millis();

    drawCurrentPage();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    static unsigned long lastSpeedUpdate = 0;
    static unsigned long lastGPSUpdate = 0;
    static unsigned long lastTripUpdate = 0;
    static unsigned long lastGPSPageUpdate = 0;
    static unsigned long lastWeatherPageUpdate = 0;
    static unsigned long lastBarometerPageUpdate = 0;
    static unsigned long lastAverage = 0;

    unsigned long now =
        millis();

    // ========================================================
    // GPS
    // ========================================================

    readGPS();

    recordHistorySample();

    // ========================================================
    // BME280
    // ========================================================

    readWeather();

    // ========================================================
    // BUTTONS
    // ========================================================

    checkButton1();
    checkButton2();

    // ========================================================
    // PAGE CHANGE
    // ========================================================

    if (pageNeedsRedraw)
    {
        drawCurrentPage();
    }

    // ========================================================
    // AVERAGE SPEED
    // ========================================================

    if (
        now -
        lastAverage >=
        1000)
    {
        lastAverage =
            now;

        updateAverageSpeed();
    }

    if (
        now -
        lastTripSave >=
        TRIP_SAVE_INTERVAL)
    {
        lastTripSave =
            now;

        saveTripData();
    }

    // ========================================================
    // MAIN SPEED PAGE
    // ========================================================

    if (currentPage == 0)
    {
        updateGaugeMode();

        if (
            now -
            lastSpeedUpdate >=
            250)
        {
            lastSpeedUpdate =
                now;

            updateSpeed();

            updateMainDistance();

            updateClock();
        }

        if (
            now -
            lastGPSUpdate >=
            1000)
        {
            lastGPSUpdate =
                now;

            updateSatelliteSymbol();
        }

        drawSpeedBar();
    }

    // ========================================================
    // TRIP PAGE
    // ========================================================

    else if (currentPage == 1)
    {
        if (
            now -
            lastTripUpdate >=
            500)
        {
            lastTripUpdate =
                now;

            updateTripPage();
        }
    }

    // ========================================================
    // GPS PAGE
    // ========================================================

    else if (currentPage == 2)
    {
        if (
            now -
            lastGPSPageUpdate >=
            500)
        {
            lastGPSPageUpdate =
                now;

            updateGPSPage();
        }
    }

    // ========================================================
    // COMPASS PAGE
    // ========================================================

    else if (currentPage == 3)
    {
        if (
            now -
            lastCompassUpdate >=
            250)
        {
            lastCompassUpdate =
                now;

            if (
                gps.course.isValid())
            {
                currentHeading =
                    gps.course.deg();
            }

            drawCompassPage();
        }
    }

    // ========================================================
    // CLOCK PAGE
    // ========================================================

    else if (currentPage == 4)
    {
        if (
            now -
            lastClockUpdate >=
            100)
        {
            lastClockUpdate =
                now;

            if (gps.time.isValid())
            {
                int currentSecond =
                    gps.time.second();

                if (
                    currentSecond !=
                    previousClockSecond)
                {
                    if (
                        previousClockSecond >=
                        0)
                    {
                        restoreClockSecondDot(
                            previousClockSecond
                        );
                    }

                    drawClockSecondDot(
                        currentSecond
                    );

                    previousClockSecond =
                        currentSecond;

                    drawClockTime();
                    drawClockDate();
                }
            }
        }
    }

    // ========================================================
    // WEATHER PAGE
    // ========================================================

    else if (currentPage == 5)
    {
        if (
            now -
            lastWeatherPageUpdate >=
            500)
        {
            lastWeatherPageUpdate =
                now;

            updateWeatherPage();
        }
    }

    // ========================================================
    // BAROMETER PAGE
    // ========================================================

    else if (currentPage == 6)
    {
        if (
            now -
            lastBarometerPageUpdate >=
            500)
        {
            lastBarometerPageUpdate =
                now;

            updateBarometerPage();
        }
    }

    // ========================================================
    // SYSTEM PAGE
    // ========================================================

    else if (currentPage == 7)
    {
        if (
            now -
            lastSystemUpdate >=
            1000)
        {
            lastSystemUpdate =
                now;

            updateSystemPage();
        }
    }

    // ========================================================
    // HISTORY PAGE
    // ========================================================

    else if (currentPage == 8)
    {
        if (
            now -
            lastHistoryPageUpdate >=
            1000)
        {
            lastHistoryPageUpdate =
                now;

            drawHistoryPage();
        }
    }

    delay(5);
}
