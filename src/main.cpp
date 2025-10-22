#include <Arduino.h>
#include <math.h>
#include "ads_reader.h"
#include "calculate.h"

// ===================== Geometry (SI) =====================
static constexpr float INLET_DIAMETER_M = 0.0370f;  // 3.7 cm
static constexpr float THROAT_DIAMETER_M = 0.0150f; // 1.5 cm
static constexpr float AIR_DENSITY = 1.225f;        // kg/m^3

// ===================== I2C / ADS =====================
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
ADSReader ads(0x48, GAIN_SIXTEEN, SDA_PIN, SCL_PIN); // A0-A1 diff

// ===================== Button (ZERO + RESET) =====================
static constexpr int CAL_BUTTON_PIN = 18; // active-LOW
static constexpr uint32_t DEBOUNCE_MS = 50;

// ===================== MPX10DP Scale =====================
// Can be set at runtime with "K <Pa>"
static float SCALE_PA_PER_VOLT = 4000.0f; // Pa/V (placeholder)

// ===================== Filters & thresholds =====================
static constexpr float LPF_ALPHA = 0.2f;
static constexpr int MA_WINDOW = 5;
static constexpr float INACTIVITY_S = 1.0f;

// Fix zero-creep:
static constexpr float DP_DEADBAND_PA = 0.1f;    // set ΔP=0 if |ΔP|<5 Pa
static constexpr float FLOW_IGNORE_ML_S = 20.0f; // ignore <100 mL/s for integration/breath

// ===================== Calc object =====================
Calculate calc(INLET_DIAMETER_M, THROAT_DIAMETER_M, AIR_DENSITY);

// ===================== State =====================
float zeroVolt = 0.0f;
float totalVolume_L = 0.0f;
float last_Q_L_s = 0.0f;
uint32_t lastMs = 0;

float maBuf[MA_WINDOW];
int maIdx = 0;
bool maFull = false;

struct Sample
{
    float flow_mL_s;
    uint32_t ms;
};
static const size_t FLOW_BUF_MAX = 1024;
Sample breathBuf[FLOW_BUF_MAX];
size_t breathN = 0;
uint32_t lastActiveTick = 0;

float peak_mL_s_5s = 0.0f, sum_mL_s_5s = 0.0f;
int cnt_5s = 0;

// ===================== Utils =====================
float applyMA(float x)
{
    maBuf[maIdx] = x;
    maIdx = (maIdx + 1) % MA_WINDOW;
    if (maIdx == 0)
        maFull = true;
    float s = 0;
    int n = maFull ? MA_WINDOW : maIdx;
    if (n <= 0)
        return x;
    for (int i = 0; i < n; ++i)
        s += maBuf[i];
    return s / n;
}

void clearSession()
{
    totalVolume_L = 0.0f;
    last_Q_L_s = 0.0f;
    breathN = 0;
    peak_mL_s_5s = 0.0f;
    sum_mL_s_5s = 0.0f;
    cnt_5s = 0;
    Serial.println(F("[RESET] total volume & buffers cleared"));
}

// Zero offset (avg) + RESET totals
void doZero()
{
    Serial.println(F("[ZERO] Mulai... pastikan dua port setara tekanan"));
    float sum = 0.0f;
    const int N = 500; // increased for steadier zero
    for (int i = 0; i < N; ++i)
    {
        sum += ads.readDiffVoltFiltered();
        delay(5);
    }
    zeroVolt = sum / N;
    Serial.print(F("[ZERO] zeroVolt="));
    Serial.print(zeroVolt, 6);
    Serial.println(F(" V"));
    clearSession();
}

// Span: set SCALE Pa/V using current Vdiff at known Pa
void applySpanFromPa(float known_Pa)
{
    float vdiff = ads.readDiffVoltFiltered() - zeroVolt;
    if (fabsf(vdiff) < 1e-6f)
    {
        Serial.println(F("[SPAN] Vdiff ~0 V. Naikkan ΔP lalu ulangi."));
        return;
    }
    SCALE_PA_PER_VOLT = known_Pa / vdiff;
    Serial.print(F("[SPAN] Set SCALE_PA_PER_VOLT = "));
    Serial.print(SCALE_PA_PER_VOLT, 2);
    Serial.println(F(" Pa/V"));
}

void printHelp()
{
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  ?        : help"));
    Serial.println(F("  Z        : ZERO + reset total volume"));
    Serial.println(F("  K <Pa>   : span set SCALE = Pa / Vdiff_now"));
    Serial.println(F("  R        : reset total volume only"));
    Serial.println(F("  P        : print parameters & constants"));
    Serial.println();
}

void printParams()
{
    Serial.println(F("==== PARAMETER ===="));
    Serial.print(F("Inlet Dia  (m): "));
    Serial.println(INLET_DIAMETER_M, 6);
    Serial.print(F("Throat Dia (m): "));
    Serial.println(THROAT_DIAMETER_M, 6);
    Serial.print(F("Air density  : "));
    Serial.println(AIR_DENSITY, 3);
    Serial.print(F("Scale Pa/V   : "));
    Serial.println(SCALE_PA_PER_VOLT, 2);
    Serial.print(F("K (mL/s)/sqrt(Pa): "));
    Serial.println(calc.k_mLs(), 3);
    Serial.println(F("==================="));
}

void integrateBreathIfIdle(uint32_t nowTick)
{
    float idle_s = (nowTick - lastActiveTick) * portTICK_PERIOD_MS / 1000.0f;
    if (breathN >= 2 && idle_s >= INACTIVITY_S)
    {
        double mL = 0.0;
        for (size_t i = 1; i < breathN; ++i)
        {
            double dt = (breathBuf[i].ms - breathBuf[i - 1].ms) / 1000.0;
            mL += 0.5 * (breathBuf[i - 1].flow_mL_s + breathBuf[i].flow_mL_s) * dt;
        }
        float L = mL / 1000.0f;
        totalVolume_L += L;

        float peakThis = 0.0f;
        for (size_t i = 0; i < breathN; ++i)
            if (breathBuf[i].flow_mL_s > peakThis)
                peakThis = breathBuf[i].flow_mL_s;

        Serial.print(F("[BREATH] Vol="));
        Serial.print(L, 3);
        Serial.print(F(" L, Peak="));
        Serial.print(peakThis, 0);
        Serial.println(F(" mL/s"));
        breathN = 0;
    }
}

void handleSerial()
{
    static String line;
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (line.length())
            {
                line.trim();
                if (line.equalsIgnoreCase("?"))
                    printHelp();
                else if (line.equalsIgnoreCase("Z"))
                    doZero();
                else if (line.equalsIgnoreCase("R"))
                {
                    clearSession();
                }
                else if (line.equalsIgnoreCase("P"))
                    printParams();
                else if (line.startsWith("K "))
                {
                    float pa = line.substring(2).toFloat();
                    if (pa > 0)
                        applySpanFromPa(pa);
                    else
                        Serial.println(F("[ERR] Format: K <Pa>, contoh: K 500"));
                }
                else
                {
                    Serial.println(F("[?] Tidak dikenal. Ketik '?' untuk help."));
                }
                line = "";
            }
        }
        else
            line += c;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

    ads.setClock(100000);
    ads.setLPFAlpha(LPF_ALPHA);

    Serial.println();
    Serial.println(F("=== Indirect Calorimetry Airflow Reader ==="));
    Serial.println(F("HW: ESP32 + ADS1115 (A0-A1 diff) + MPX10DP"));
    Serial.print(F("I2C: SDA="));
    Serial.print(SDA_PIN);
    Serial.print(F(" SCL="));
    Serial.println(SCL_PIN);

    if (!ads.beginAuto())
    {
        Serial.println(F("[ERR] ADS1115 not found. Cek wiring & ADDR (0x48..0x4B)."));
        ads.i2cScan();
        while (1)
            delay(300);
    }

    printParams();
    Serial.println(F("Tips: 1) Tekan D18 saat ΔP=0 untuk ZERO + reset"));
    Serial.println(F("      2) Beri ΔP known (mis. 500 Pa), lalu ketik: K 500"));
    printHelp();

    doZero(); // zero awal + reset
    lastMs = millis();
    last_Q_L_s = 0.0f;
    lastActiveTick = xTaskGetTickCount();
    Serial.println(F("Ready.\n"));
}

void loop()
{
    handleSerial();

    // Button ZERO/RESET (active LOW) + debounce
    static bool lastBtn = HIGH;
    static uint32_t lastDeb = 0;
    bool b = digitalRead(CAL_BUTTON_PIN);
    uint32_t now = millis();
    if (b != lastBtn)
        lastDeb = now;
    if (now - lastDeb > DEBOUNCE_MS)
    {
        static bool armed = true;
        if (b == LOW && armed)
        {
            doZero();
            armed = false;
        } // ZERO + reset total
        if (b == HIGH)
            armed = true;
    }
    lastBtn = b;

    // 1) Vdiff (V) corrected by zeroVolt
    float vdiff = ads.readDiffVoltFiltered() - zeroVolt;

    // 2) Volt -> ΔP (Pa), apply deadband
    float dP_Pa = vdiff * SCALE_PA_PER_VOLT;
    if (fabsf(dP_Pa) < DP_DEADBAND_PA)
        dP_Pa = 0.0f;
    if (dP_Pa < 0)
        dP_Pa = 0;

    // 3) ΔP -> Q (mL/s) [forward]
    float Q_mL_s = calc.airflow_mLs(dP_Pa, 0.0f);
    if (Q_mL_s < FLOW_IGNORE_ML_S)
        Q_mL_s = 0.0f; // ignore tiny flows for integration
    float Q_mL_s_view = applyMA(Q_mL_s);

    // 4) Q -> ΔP (Pa) [inverse] for display cross-check
    float dP_back_Pa = calc.pressure_from_Q_mLs(Q_mL_s);

    // 5) Integrate total volume (trapezoid)
    uint32_t nowMs = millis();
    float dt_s = (nowMs - lastMs) / 1000.0f;
    lastMs = nowMs;
    float Q_L_s = Q_mL_s / 1000.0f; // mL/s -> L/s
    totalVolume_L += 0.5f * (last_Q_L_s + Q_L_s) * dt_s;
    last_Q_L_s = Q_L_s;

    // 6) breath buffer (only record meaningful flow)
    if (Q_mL_s > 0.0f && breathN < FLOW_BUF_MAX)
    {
        breathBuf[breathN++] = {Q_mL_s, nowMs};
        lastActiveTick = xTaskGetTickCount();
    }
    integrateBreathIfIdle(xTaskGetTickCount());

    // 7) 5-sec stats
    peak_mL_s_5s = max(peak_mL_s_5s, Q_mL_s);
    sum_mL_s_5s += Q_mL_s;
    cnt_5s++;
    static uint32_t lastStat = 0;
    if (nowMs - lastStat >= 5000)
    {
        float avg5 = (cnt_5s > 0) ? (sum_mL_s_5s / cnt_5s) : 0.0f;
        Serial.print(F("[STAT 5s] Peak="));
        Serial.print(peak_mL_s_5s, 0);
        Serial.print(F(" mL/s, Avg="));
        Serial.print(avg5, 0);
        Serial.print(F(" mL/s, Total="));
        Serial.print(totalVolume_L, 3);
        Serial.println(F(" L"));
        peak_mL_s_5s = 0;
        sum_mL_s_5s = 0;
        cnt_5s = 0;
        lastStat = nowMs;
    }

    // 8) 10 Hz print
    static uint32_t lastPrint = 0;
    if (nowMs - lastPrint >= 100)
    {
        lastPrint = nowMs;
        Serial.print(F("dP="));
        Serial.print(dP_Pa, 0);
        Serial.print(F("Pa "));
        Serial.print(F("Q="));
        Serial.print(Q_mL_s_view, 0);
        Serial.print(F("mL/s "));
        Serial.print(F("dP<-Q="));
        Serial.print(dP_back_Pa, 1);
        Serial.print(F("Pa "));
        Serial.print(F("V="));
        Serial.print(totalVolume_L, 3);
        Serial.println(F("L"));
    }

    delay(10); // ~100 Hz
}
