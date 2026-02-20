import processing.serial.*;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.io.File;

Serial myPort;

// ============================
// CONFIG (saved outside repo)
// ============================
final String CFG_FILE = configPath();

// ============================
// ORIENTATION (rotation + mirror + fine rotate)  (applies to SERIAL + SIM)
// ============================
int ORIENT_ROT = 0;             // 0,1,2,3 => 0°,90°,180°,270°
boolean ORIENT_FLIP_X = false;  // mirror X (left/right)
boolean ORIENT_FLIP_Y = false;  // mirror Y (up/down)

// jemná rotace v deg (typicky +/- pár stupňů)
float ORIENT_FINE_DEG = 0.0;        // plynulé doladění
float ORIENT_FINE_STEP = 0.5;       // coarse step (LEFT/RIGHT)
float ORIENT_FINE_STEP_FINE = 0.1;  // SHIFT+LEFT/RIGHT

static final float DEG2RAD = PI / 180.0;

// Helper: rotate+flip in "unit" coords
PVector orientXY(float x, float y) {
  float ox = x, oy = y;

  // rotace po 90° (hrubá)
  if (ORIENT_ROT == 1) { float t = ox; ox = oy;  oy = -t; }      // +90
  else if (ORIENT_ROT == 2) { ox = -ox; oy = -oy; }             // 180
  else if (ORIENT_ROT == 3) { float t = ox; ox = -oy; oy = t; } // 270

  // zrcadlení
  if (ORIENT_FLIP_X) ox = -ox;
  if (ORIENT_FLIP_Y) oy = -oy;

  // jemná rotace (plynulá) – až nakonec
  if (abs(ORIENT_FINE_DEG) > 0.0001) {
    float a = ORIENT_FINE_DEG * DEG2RAD;
    float ca = cos(a), sa = sin(a);
    float rx = ox * ca - oy * sa;
    float ry = ox * sa + oy * ca;
    ox = rx; oy = ry;
  }

  return new PVector(ox, oy);
}

String orientText() {
  int deg = ORIENT_ROT * 90;
  return deg + "deg FX=" + (ORIENT_FLIP_X ? "1" : "0") +
         " FY=" + (ORIENT_FLIP_Y ? "1" : "0") +
         " fine=" + nf(ORIENT_FINE_DEG, 0, 2) + "deg";
}

// ============================
// SCALE (ESP data)
// ============================
float scaleFactor = 300;
float SCALE_STEP = 1.0;        // LEFT/RIGHT when SCALE selected
float SCALE_STEP_FINE = 0.1;   // SHIFT+LEFT/RIGHT when SCALE selected
float SCALE_MIN = 10;
float SCALE_MAX = 5000;

// ============================
// PARAM EDIT UI
// TAB selects param, LEFT/RIGHT changes param, SHIFT = fine step where applicable
// BACKSPACE resets selected param
// ============================
int EDIT_PARAM = 0;
// 0=ROT90, 1=FLIP_X, 2=FLIP_Y, 3=FINE_DEG, 4=SCALE
static final int EDIT_PARAM_COUNT = 5;

String editParamName() {
  if (EDIT_PARAM == 0) return "ROT90";
  if (EDIT_PARAM == 1) return "FLIP_X";
  if (EDIT_PARAM == 2) return "FLIP_Y";
  if (EDIT_PARAM == 3) return "FINE_DEG";
  return "SCALE";
}

String editParamValue() {
  if (EDIT_PARAM == 0) return str(ORIENT_ROT * 90) + "deg";
  if (EDIT_PARAM == 1) return ORIENT_FLIP_X ? "ON" : "OFF";
  if (EDIT_PARAM == 2) return ORIENT_FLIP_Y ? "ON" : "OFF";
  if (EDIT_PARAM == 3) return nf(ORIENT_FINE_DEG, 0, 2) + "deg";
  return nf(scaleFactor, 0, 2);
}

void editSelectNext() {
  EDIT_PARAM = (EDIT_PARAM + 1) % EDIT_PARAM_COUNT;
  println("Edit => " + editParamName() + " (" + editParamValue() + ")");
}

void editAdjust(int dir, boolean fineStep) {
  if (dir == 0) return;

  if (EDIT_PARAM == 0) {
    ORIENT_ROT = (ORIENT_ROT + (dir > 0 ? 1 : 3)) % 4;

  } else if (EDIT_PARAM == 1) {
    ORIENT_FLIP_X = !ORIENT_FLIP_X;

  } else if (EDIT_PARAM == 2) {
    ORIENT_FLIP_Y = !ORIENT_FLIP_Y;

  } else if (EDIT_PARAM == 3) {
    float step = fineStep ? ORIENT_FINE_STEP_FINE : ORIENT_FINE_STEP;
    ORIENT_FINE_DEG += dir * step;
    ORIENT_FINE_DEG = constrain(ORIENT_FINE_DEG, -45.0, 45.0);

  } else { // SCALE
    float step = fineStep ? SCALE_STEP_FINE : SCALE_STEP;
    scaleFactor += dir * step;
    scaleFactor = constrain(scaleFactor, SCALE_MIN, SCALE_MAX);
  }

  println("Edit => " + editParamName() + "=" + editParamValue() + " | orient=" + orientText() + " | scale=" + nf(scaleFactor, 0, 2));
}

void editResetSelected() {
  if (EDIT_PARAM == 0) ORIENT_ROT = 0;
  else if (EDIT_PARAM == 1) ORIENT_FLIP_X = false;
  else if (EDIT_PARAM == 2) ORIENT_FLIP_Y = false;
  else if (EDIT_PARAM == 3) ORIENT_FINE_DEG = 0.0;
  else autoFitScale(); // SCALE reset => autofit

  println("Reset => " + editParamName() + "=" + editParamValue() + " | orient=" + orientText() + " | scale=" + nf(scaleFactor, 0, 2));
}

// ============================
// SIMULATION toggle
// ============================
boolean USE_SIM = false;          // false = ESP, true = simulace
int SIM_MODE = 1;                 // 1=Lissajous, 2=Spiro, 3=Random walk

// SIM hodnoty chceme v "unit" rozsahu jako ESP (cca -1..1)
float SIM_GAIN = 0.008;   // 90 * 0.008 = 0.72
float simRateHz = 200;
int simPointsPerFrameMax = 12;

// SIM vykreslování VŽDY cca 300x300 px (nezávislé na scaleFactor)
float simScalePx = 150.0;          // poloměr 150px => obrazec ~300x300
float SIM_UNIT_CLAMP = 1.2;        // pojistka (clamp unit)


// ---- Serial / reconnect ----
String preferredPort = defaultPreferredPort();
int baud = 115200;

String defaultPreferredPort() {
  String os = System.getProperty("os.name").toLowerCase();
  if (os.contains("win")) return "COM7";
  if (os.contains("linux")) return "";
  return "";
}

int reconnectEveryMs = 2000;
int lastReconnectTryMs = 0;

int noDataDisconnectMs = 1200;
int connectGraceMs = 2500;
int lastConnectMs = 0;

// ---- kreslení ----
boolean havePrev = false;
float prevX, prevY;
ArrayDeque<PVector> queue = new ArrayDeque<PVector>();
int segmentsPerFrame = 20;

// ---- UI / stav ----
String status = "WAITING";
int lastDataMs = 0;

// poslední souřadnice (v UNIT)
float lastX = 0, lastY = 0;
// poslední vykreslené pixely
float lastDrawX = 0, lastDrawY = 0;
boolean haveXY = false;

boolean debugUI = false;

PGraphics canvasLayer;  // stopa
PGraphics hud;          // debug overlay

// ---- logging ----
int linesIn = 0;
int segmentsDrawn = 0;
int lastLogMs = 0;

// ============================
// SIM state
// ============================
float simT = 0;
float simCarry = 0;
float rwX = 0, rwY = 0;

// drift (unit)
float driftX = 0, driftY = 0;
float driftVX = 0, driftVY = 0;

// ============================
// RENDER modes (0=line, 1=brush, 2=sand)
// ============================
int RENDER_MODE = 1; // default BRUSH

String renderModeName() {
  if (RENDER_MODE == 0) return "LINE";
  if (RENDER_MODE == 1) return "BRUSH";
  return "SAND";
}

// ============================
// PALETTES
// ============================
float [][] STROKE_COLORS = {
  {  0,  0,  0},  // 0: BLACK
  {  0, 90, 85},  // 1: RED
  { 60, 90, 95},  // 2: YELLOW
  {220, 90, 85},  // 3: BLUE
  { 30, 90, 92},  // 4: ORANGE
  {120, 90, 80},  // 5: GREEN
  {270, 85, 80},  // 6: PURPLE
  { 25, 70, 45},  // 7: BROWN
  {  0,  0, 100}   // 8: WHITE
};
int strokeColorIdx = 1;

float[][] BG_COLORS = {
  {0, 0, 100},
  {0, 0, 94},
  {45, 10, 98},
  {220, 30, 14},
  {0, 0, 0}
};
int bgIdx = 0;

// derived colors
float lineHue, lineSat, lineBri;
int inkHue, inkSat, inkBri;
int sandHue, sandSat, sandBri;

// brush settings
boolean INK_MULTIPLY = true;
float inkBrushR = 2.0;
float inkAlpha = 18;
float inkSpacing = 2.0;

// sand settings
float sandSpacing = 2.5;
int sandGrainsPerStep = 4;
float sandSpread = 3.0;
float sandMinR = 0.8;
float sandMaxR = 1.7;
float sandAlpha = 22;

// line settings
float lineWeight = 2.0;
float MIN_LINE_SEG_PX = 0.8;

// speed mapping
float SPEED_DIST_REF = 18.0;
float SPEED_ALPHA_MIN = 0.30;
float SPEED_ALPHA_MAX = 1.25;
float SPEED_SPACING_GAIN = 0.05;

// ============================
// paths / parsing helpers
// ============================
String configPath() {
  String os = System.getProperty("os.name").toLowerCase();

  if (os.contains("win")) {
    String appdata = System.getenv("APPDATA");
    if (appdata != null) return appdata + "\\Kyvadlo\\config.txt";
    return System.getProperty("user.home") + "\\Kyvadlo\\config.txt";
  }

  return System.getProperty("user.home") + "/.config/kyvadlo/config.txt";
}

float parseFloatSafe(String s, float fallback) {
  if (s == null) return fallback;
  s = trim(s);
  if (s.length() == 0) return fallback;
  s = s.replace(',', '.');
  int hash = s.indexOf('#');
  if (hash >= 0) s = trim(s.substring(0, hash));
  try {
    float v = Float.parseFloat(s);
    if (Float.isNaN(v) || Float.isInfinite(v)) return fallback;
    return v;
  } catch (Exception e) {
    return fallback;
  }
}

int parseIntSafe(String s, int fallback) {
  if (s == null) return fallback;
  s = trim(s);
  if (s.length() == 0) return fallback;
  int hash = s.indexOf('#');
  if (hash >= 0) s = trim(s.substring(0, hash));
  try {
    return Integer.parseInt(s);
  } catch (Exception e) {
    return fallback;
  }
}

void setup() {
  size(800, 800);
  smooth();

  loadConfig();

  canvasLayer = createGraphics(width, height);
  hud = createGraphics(width, height);

  canvasLayer.beginDraw();
  canvasLayer.colorMode(HSB, 360, 100, 100, 255);
  canvasLayer.background(BG_COLORS[bgIdx][0], BG_COLORS[bgIdx][1], BG_COLORS[bgIdx][2]);
  canvasLayer.endDraw();

  applyStrokePalette();

  if (!USE_SIM) {
    println(Serial.list());
    connectSerial();
  } else {
    status = "RUN";
    rwX = 0; rwY = 0;
  }

  lastDataMs = millis();
  lastLogMs = millis();
}

void draw() {
  if (!USE_SIM) ensureConnected();

  if (USE_SIM) generateSimPoints();

  int n = 0;
  if (!queue.isEmpty()) {
    canvasLayer.beginDraw();
    canvasLayer.colorMode(HSB, 360, 100, 100, 255);

    while (n < segmentsPerFrame && !queue.isEmpty()) {
      PVector p = queue.removeFirst();

      if (havePrev) {
        if (RENDER_MODE == 0) {
          drawLineSegment(canvasLayer, prevX, prevY, p.x, p.y);
        } else if (RENDER_MODE == 1) {
          drawInkSegment(canvasLayer, prevX, prevY, p.x, p.y);
        } else {
          drawSandSegment(canvasLayer, prevX, prevY, p.x, p.y);
        }
        segmentsDrawn++;
      } else {
        havePrev = true;
      }

      prevX = p.x; prevY = p.y;
      n++;
    }

    canvasLayer.endDraw();
  }

  if (queue.size() > 500) segmentsPerFrame = 60;
  else if (queue.size() > 250) segmentsPerFrame = 35;
  else segmentsPerFrame = 20;

  if (!USE_SIM && myPort != null) {
    int now = millis();
    boolean inGrace = (now - lastConnectMs) < connectGraceMs;
    boolean timedOut = (now - lastDataMs) > noDataDisconnectMs;

    if (!inGrace && timedOut) {
      println("No data for " + (now - lastDataMs) + "ms -> disconnect & auto-reconnect");
      disconnectSerial();
      status = "ERROR";
    }
  }

  if (!USE_SIM && status.equals("RUN") && millis() - lastDataMs > 1500) {
    status = "WAITING";
  }

  int now = millis();
  if (debugUI && now - lastLogMs >= 1000) {
    println(
      "LOG fps=" + nf(frameRate, 0, 1) +
      " lines/s=" + linesIn +
      " seg/s=" + segmentsDrawn +
      " queue=" + queue.size() +
      " seg/frame=" + segmentsPerFrame +
      " src=" + (USE_SIM ? ("SIM(mode " + SIM_MODE + ")") : ("SERIAL(" + (myPort == null ? "DISCONNECTED" : preferredPort) + ")")) +
      " render=" + renderModeName() +
      " strokeColorIdx=" + strokeColorIdx +
      " bgIdx=" + bgIdx +
      " scale=" + nf(scaleFactor, 0, 2) +
      " orient=" + orientText()
    );
    linesIn = 0;
    segmentsDrawn = 0;
    lastLogMs = now;
  }

  image(canvasLayer, 0, 0);

  if (debugUI) {
    drawHUD();
    image(hud, 0, 0);
  }
}

// ============================
// CONFIG load/save
// ============================
void loadConfig() {
  try {
    String[] lines = loadStrings(CFG_FILE);
    if (lines == null) {
      println("Config: none (using defaults)");
      return;
    }

    for (String s : lines) {
      if (s == null) continue;
      s = trim(s);
      if (s.length() == 0) continue;
      if (s.startsWith("#")) continue;

      int eq = s.indexOf('=');
      if (eq <= 0) continue;

      String k = trim(s.substring(0, eq));
      String v = trim(s.substring(eq + 1));

      if (k.equalsIgnoreCase("scaleFactor")) {
        float val = parseFloatSafe(v, scaleFactor);
        if (val > 0) scaleFactor = val;

      } else if (k.equalsIgnoreCase("bgIdx")) {
        int val = parseIntSafe(v, bgIdx);
        if (val >= 0 && val < BG_COLORS.length) bgIdx = val;

      } else if (k.equalsIgnoreCase("strokeColorIdx")) {
        int val = parseIntSafe(v, strokeColorIdx);
        if (val >= 0 && val < STROKE_COLORS.length) strokeColorIdx = val;

      } else if (k.equalsIgnoreCase("renderMode")) {
        int val = parseIntSafe(v, RENDER_MODE);
        if (val >= 0 && val <= 2) RENDER_MODE = val;

      } else if (k.equalsIgnoreCase("orientRot")) {
        int val = parseIntSafe(v, ORIENT_ROT);
        if (val >= 0 && val <= 3) ORIENT_ROT = val;

      } else if (k.equalsIgnoreCase("flipX")) {
        ORIENT_FLIP_X = (parseIntSafe(v, ORIENT_FLIP_X ? 1 : 0) != 0);

      } else if (k.equalsIgnoreCase("flipY")) {
        ORIENT_FLIP_Y = (parseIntSafe(v, ORIENT_FLIP_Y ? 1 : 0) != 0);

      } else if (k.equalsIgnoreCase("orientFineDeg")) {
        ORIENT_FINE_DEG = parseFloatSafe(v, ORIENT_FINE_DEG);
      }
    }

    println("Config loaded: scale=" + nf(scaleFactor, 0, 2) + " orient=" + orientText());
  } catch(Exception e) {
    println("Config load failed: " + e);
  }
}

void saveConfig() {
  try {
    String[] out = new String[] {
      "# Kyvadlo config",
      "scaleFactor=" + Float.toString(scaleFactor),
      "bgIdx=" + bgIdx,
      "strokeColorIdx=" + strokeColorIdx,
      "renderMode=" + RENDER_MODE,
      "orientRot=" + ORIENT_ROT,
      "flipX=" + (ORIENT_FLIP_X ? 1 : 0),
      "flipY=" + (ORIENT_FLIP_Y ? 1 : 0),
      "orientFineDeg=" + Float.toString(ORIENT_FINE_DEG)
    };

    File f = new File(CFG_FILE);
    if (f.getParentFile() != null) f.getParentFile().mkdirs();
    saveStrings(CFG_FILE, out);
    println("Config saved -> " + CFG_FILE);
  } catch(Exception e) {
    println("Config save failed: " + e);
  }
}

void autoFitScale() {
  scaleFactor = min(width, height) * 0.40;
  scaleFactor = constrain(scaleFactor, SCALE_MIN, SCALE_MAX);
  println("Auto-fit scaleFactor=" + nf(scaleFactor, 0, 2));
}

// ============================
// Palette apply helpers
// ============================
void applyStrokePalette() {
  float[] c = STROKE_COLORS[strokeColorIdx];

  lineHue = c[0];
  lineSat = c[1];
  lineBri = c[2];

  inkHue = round(c[0]);
  inkSat = round(c[1]);
  inkBri = round(c[2]);

  sandHue = round(c[0]);
  sandSat = round(max(12, c[1] * 0.35));
  sandBri = round(min(95, c[2] + 55));
}

void applyBackgroundAndClear() {
  float[] b = BG_COLORS[bgIdx];
  canvasLayer.beginDraw();
  canvasLayer.colorMode(HSB, 360, 100, 100, 255);
  canvasLayer.background(b[0], b[1], b[2]);
  canvasLayer.endDraw();

  havePrev = false;
  queue.clear();
}

// ============================
// SPEED helpers
// ============================
float alphaFactorFromDist(float dist) {
  float t = constrain(dist / SPEED_DIST_REF, 0, 2.5);
  float f = lerp(SPEED_ALPHA_MAX, 1.0, constrain(t, 0, 1));
  if (t > 1) f = lerp(1.0, SPEED_ALPHA_MIN, constrain((t - 1.0) / 1.5, 0, 1));
  return f;
}

float spacingFactorFromDist(float dist) {
  return 1.0 + dist * SPEED_SPACING_GAIN;
}

// ============================
// RENDER: LINE / BRUSH / SAND
// ============================
void drawLineSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  if (dx*dx + dy*dy < MIN_LINE_SEG_PX*MIN_LINE_SEG_PX) return;

  g.blendMode(BLEND);
  g.stroke(lineHue, lineSat, lineBri, 255);
  g.strokeWeight(lineWeight);
  g.strokeCap(SQUARE);
  g.noFill();
  g.line(x1, y1, x2, y2);
}

void drawInkSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  if (INK_MULTIPLY) g.blendMode(MULTIPLY);
  else g.blendMode(BLEND);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dist = sqrt(dx*dx + dy*dy);
  if (dist < 0.0001) { g.blendMode(BLEND); return; }

  float aFac = alphaFactorFromDist(dist);
  float spFac = spacingFactorFromDist(dist);

  float effectiveSpacing = inkSpacing * spFac;
  int steps = max(1, ceil(dist / effectiveSpacing));

  g.noStroke();
  g.fill(inkHue, inkSat, inkBri, constrain(inkAlpha * aFac, 1, 255));

  float jitter = 0.10 * inkBrushR;

  for (int i = 0; i <= steps; i++) {
    float t = i / (float)steps;
    float px = lerp(x1, x2, t) + random(-jitter, jitter);
    float py = lerp(y1, y2, t) + random(-jitter, jitter);
    g.ellipse(px, py, inkBrushR*2, inkBrushR*2);
  }

  g.blendMode(BLEND);
}

void drawSandSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  g.blendMode(BLEND);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dist = sqrt(dx*dx + dy*dy);
  if (dist < 0.0001) return;

  float aFac = alphaFactorFromDist(dist);
  float spFac = spacingFactorFromDist(dist);

  float effectiveSpacing = sandSpacing * spFac;
  int steps = max(1, ceil(dist / effectiveSpacing));

  g.noStroke();
  g.fill(sandHue, sandSat, sandBri, constrain(sandAlpha * aFac, 1, 255));

  int grains = max(1, round(sandGrainsPerStep * lerp(1.2, 0.55, constrain(dist / SPEED_DIST_REF, 0, 1))));

  for (int i = 0; i <= steps; i++) {
    float t = i / (float)steps;
    float bx = lerp(x1, x2, t);
    float by = lerp(y1, y2, t);

    for (int k = 0; k < grains; k++) {
      float a = random(TWO_PI);
      float r = abs((float)randomGaussian()) * sandSpread;
      float px = bx + cos(a) * r;
      float py = by + sin(a) * r;

      float gr = random(sandMinR, sandMaxR);
      g.ellipse(px, py, gr*2, gr*2);
    }
  }
}

// ============================
// SIM generator -> queue
// ============================
void generateSimPoints() {
  float dt = 1.0 / max(frameRate, 1.0);
  simCarry += simRateHz * dt;

  int toGen = min(simPointsPerFrameMax, floor(simCarry));
  if (toGen <= 0) return;

  simCarry -= toGen;
  float step = 1.0 / simRateHz;

  float driftAccel = 0.010;
  float driftDamp  = 0.995;
  float driftMaxV  = 0.06;
  float driftMaxR  = 0.20;
  float noiseAmp   = 0.004;

  for (int i = 0; i < toGen; i++) {
    simT += step;

    float x = 0, y = 0;

    if (SIM_MODE == 1) {
      float ax = 90, ay = 70;
      float fx = 2.0, fy = 3.0;
      float phase = PI/3;
      x = ax * sin(TWO_PI * fx * simT + phase);
      y = ay * sin(TWO_PI * fy * simT);

    } else if (SIM_MODE == 2) {
      float R = 90, r = 35, d = 60;
      float t = TWO_PI * 0.20 * simT;
      x = (R - r) * cos(t) + d * cos(((R - r) / r) * t);
      y = (R - r) * sin(t) - d * sin(((R - r) / r) * t);

    } else {
      float stepSize = 2.5;
      rwX += random(-stepSize, stepSize);
      rwY += random(-stepSize, stepSize);
      rwX = constrain(rwX, -120, 120);
      rwY = constrain(rwY, -120, 120);
      x = rwX; y = rwY;
    }

    x *= SIM_GAIN;
    y *= SIM_GAIN;

    driftVX += random(-driftAccel, driftAccel) * step;
    driftVY += random(-driftAccel, driftAccel) * step;
    driftVX *= driftDamp;
    driftVY *= driftDamp;
    driftVX = constrain(driftVX, -driftMaxV, driftMaxV);
    driftVY = constrain(driftVY, -driftMaxV, driftMaxV);
    driftX += driftVX;
    driftY += driftVY;
    driftX = constrain(driftX, -driftMaxR, driftMaxR);
    driftY = constrain(driftY, -driftMaxR, driftMaxR);

    x += driftX + random(-noiseAmp, noiseAmp);
    y += driftY + random(-noiseAmp, noiseAmp);

    x = constrain(x, -SIM_UNIT_CLAMP, SIM_UNIT_CLAMP);
    y = constrain(y, -SIM_UNIT_CLAMP, SIM_UNIT_CLAMP);

    PVector o = orientXY(x, y);
    x = o.x; y = o.y;

    float drawX = width/2 + x * simScalePx;
    float drawY = height/2 - y * simScalePx;

    lastX = x; lastY = y;
    lastDrawX = drawX; lastDrawY = drawY;
    haveXY = true;

    queue.addLast(new PVector(drawX, drawY));
    lastDataMs = millis();
    status = "RUN";
    linesIn++;

    while (queue.size() > 12000) queue.removeFirst();
  }
}

void drawHUD() {
  hud.beginDraw();
  hud.clear();
  hud.textAlign(LEFT, TOP);
  hud.textSize(12);

  hud.noStroke();
  hud.fill(0, 0, 0, 120);
  hud.rect(0, 0, width, debugUI ? 70 : 22);

  hud.fill(255);
  String srcText = USE_SIM ? ("SIM " + SIM_MODE) : "SERIAL";
  String portText = (myPort == null) ? "DISCONNECTED" : ("OK " + preferredPort);

  String xyText = haveXY
    ? ("x=" + nf(lastX, 0, 2) + " y=" + nf(lastY, 0, 2))
    : "x,y=...";

  String line1 =
    "SRC:" + srcText +
    " | " + portText +
    " | ST:" + status +
    " | scale:" + nf(scaleFactor, 0, 2) +
    (USE_SIM ? (" | simPx:" + nf(simScalePx, 0, 0) + " gain:" + nf(SIM_GAIN, 0, 4)) : "") +
    " | orient:" + orientText() +
    " | edit:" + editParamName() + "=" + editParamValue() +
    " | render:" + renderModeName() +
    " | col:" + (strokeColorIdx+1) + "/" + STROKE_COLORS.length +
    " | bg:" + (bgIdx+1) + "/" + BG_COLORS.length;

  hud.text(line1, 10, 4);

  if (debugUI) {
    String line2 =
      "FPS:" + nf(frameRate, 0, 1) +
      " | queue:" + queue.size() +
      " | seg/frame:" + segmentsPerFrame +
      " | " + xyText +
      " | px=" + nf(lastDrawX, 0, 1) + " py=" + nf(lastDrawY, 0, 1);

    hud.text(line2, 10, 22);

    String help1 =
      "Keys: D debug | T SIM/SERIAL | 1/2/3 sim mode | Q color | W render | E bg+clear | SPACE clear";
    String help2 =
      "Edit: TAB select | \u2190/\u2192 change | SHIFT+\u2190/\u2192 fine | BACKSPACE reset | 0 autofit scale | S save cfg | C calibrate | R reconnect";

    hud.text(help1, 10, 40);
    hud.text(help2, 10, 56);
  }

  hud.endDraw();
}

// ============================
// SERIAL handling
// ============================
void serialEvent(Serial p) {
  if (USE_SIM) return;

  try {
    String line = p.readStringUntil('\n');
    if (line == null) return;

    line = trim(line);
    if (line.length() == 0) return;

    lastDataMs = millis();

    if (line.equals("CALIB_START")) { status = "CALIBRATING"; return; }
    if (line.equals("CALIB_OK"))    { status = "RUN"; return; }

    if (line.equals("x,y")) return;
    if (line.startsWith("OK,")) return;
    if (line.startsWith("ERR,")) { status = "ERROR"; return; }
    if (line.equals("CLR_OK")) return;

    String[] parts = split(line, ',');
    if (parts.length != 2) return;

    float x = float(parts[0]);
    float y = float(parts[1]);

    PVector o = orientXY(x, y);
    x = o.x; y = o.y;

    float drawX = width/2 + x * scaleFactor;
    float drawY = height/2 - y * scaleFactor;

    lastX = x; lastY = y;
    lastDrawX = drawX; lastDrawY = drawY;
    haveXY = true;

    queue.addLast(new PVector(drawX, drawY));
    if (!status.equals("CALIBRATING")) status = "RUN";

    linesIn++;
    while (queue.size() > 12000) queue.removeFirst();

  } catch(Exception e) {
    println("Serial error: " + e);
    disconnectSerial();
    status = "ERROR";
  }
}

void keyPressed() {
  if (key == 'd' || key == 'D') debugUI = !debugUI;

  // --- PARAM editor ---
  if (key == TAB) {
    editSelectNext();
    return;
  }
  if (keyCode == LEFT) {
    boolean fine = (keyEvent != null && keyEvent.isShiftDown());
    editAdjust(-1, fine);
    return;
  }
  if (keyCode == RIGHT) {
    boolean fine = (keyEvent != null && keyEvent.isShiftDown());
    editAdjust(+1, fine);
    return;
  }
  if (keyCode == BACKSPACE) {
    editResetSelected();
    return;
  }

  if (key == '0') autoFitScale();
  if (key == 's' || key == 'S') saveConfig();

  if (key == 'q' || key == 'Q') {
    strokeColorIdx = (strokeColorIdx + 1) % STROKE_COLORS.length;
    applyStrokePalette();
    println("STROKE_COLOR => " + (strokeColorIdx+1) + "/" + STROKE_COLORS.length);
  }

  if (key == 'w' || key == 'W') {
    RENDER_MODE = (RENDER_MODE + 1) % 3;
    println("RENDER_MODE => " + renderModeName());
  }

  if (key == 'e' || key == 'E') {
    bgIdx = (bgIdx + 1) % BG_COLORS.length;
    applyBackgroundAndClear();
    println("BACKGROUND => " + (bgIdx+1) + "/" + BG_COLORS.length + " (cleared)");
  }

  if (key == 't' || key == 'T') {
    USE_SIM = !USE_SIM;
    println("TOGGLE source => " + (USE_SIM ? "SIM" : "SERIAL"));

    havePrev = false;
    queue.clear();

    if (USE_SIM) {
      disconnectSerial();
      status = "RUN";
    } else {
      println(Serial.list());
      connectSerial();
      status = "WAITING";
    }
  }

  if (USE_SIM) {
    if (key == '1') { SIM_MODE = 1; println("SIM_MODE=1"); }
    if (key == '2') { SIM_MODE = 2; println("SIM_MODE=2"); }
    if (key == '3') { SIM_MODE = 3; println("SIM_MODE=3"); }
  }

  if (key == 'c' || key == 'C') sendCalibrate();

  if (key == 'r' || key == 'R') {
    if (!USE_SIM) {
      println("Manual reconnect...");
      connectSerial();
    }
  }

  if (key == ' ') applyBackgroundAndClear();
}

void sendCalibrate() {
  if (!USE_SIM && myPort != null) {
    myPort.write("CAL\n");
    status = "CALIBRATING";
  }
}

// --------------------
// Serial reconnect helpers
// --------------------
void ensureConnected() {
  if (USE_SIM) return;
  if (myPort != null) return;

  int now = millis();
  if (now - lastReconnectTryMs >= reconnectEveryMs) {
    lastReconnectTryMs = now;
    connectSerial();
  }
}

void disconnectSerial() {
  try {
    if (myPort != null) {
      myPort.clear();
      myPort.stop();
    }
  } catch(Exception e) {}
  myPort = null;
}

// Helpers pro filtraci portů na Linuxu
boolean isBadLinuxPort(String p) {
  if (p == null) return true;
  if (p.equals("/dev/serial0") || p.equals("/dev/serial1")) return true;
  if (p.indexOf("ttyAMA") >= 0) return true;
  if (p.indexOf("rfcomm") >= 0) return true;
  return false;
}

void connectSerial() {
  disconnectSerial();

  try {
    String[] ports = Serial.list();
    println("Ports: " + join(ports, ", "));

    if (ports == null || ports.length == 0) {
      println("No serial ports found.");
      status = "ERROR";
      return;
    }

    String os = System.getProperty("os.name").toLowerCase();

    ArrayList<String> candidates = new ArrayList<String>();
    for (String p : ports) {
      if (os.contains("linux") && isBadLinuxPort(p)) continue;
      candidates.add(p);
    }
    String[] candArr = candidates.toArray(new String[0]);
    println("Candidates: " + join(candArr, ", "));

    if (candArr.length == 0) {
      println("No suitable serial ports after filtering.");
      status = "ERROR";
      return;
    }

    if (preferredPort != null && preferredPort.length() > 0) {
      for (String p : candArr) {
        if (p.equals(preferredPort)) {
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected to " + p);
          lastConnectMs = millis();
          status = "WAITING";
          return;
        }
      }
    }

    if (os.contains("linux")) {
      for (String p : candArr) {
        if (p.indexOf("/dev/serial/by-id/") >= 0) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (by-id) to " + p);
          lastConnectMs = millis();
          status = "WAITING";
          return;
        }
      }
      for (String p : candArr) {
        if (p.indexOf("ttyACM") >= 0) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (fallback ttyACM) to " + p);
          lastConnectMs = millis();
          status = "WAITING";
          return;
        }
      }
      for (String p : candArr) {
        if (p.indexOf("ttyUSB") >= 0) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (fallback ttyUSB) to " + p);
          lastConnectMs = millis();
          status = "WAITING";
          return;
        }
      }

      println("No ttyACM/ttyUSB/by-id device found. (Not using /dev/serial0)");
      status = "ERROR";
      return;
    }

    if (os.contains("win")) {
      for (String p : candArr) {
        if (p.startsWith("COM")) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (fallback COM) to " + p);
          lastConnectMs = millis();
          status = "WAITING";
          return;
        }
      }
    }

    println("No suitable serial port found.");
    status = "ERROR";

  } catch(Exception e) {
    println("Connect failed: " + e);
    myPort = null;
    status = "ERROR";
  }
}
