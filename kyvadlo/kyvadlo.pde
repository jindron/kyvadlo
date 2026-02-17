import processing.serial.*;
import java.util.ArrayDeque;

Serial myPort;

// ============================
// SIMULATION toggle
// ============================
boolean USE_SIM = false;          // <-- false = ESP, true = simulace
int SIM_MODE = 1;                 // 1=Lissajous, 2=Spiro, 3=Random walk
float simRateHz = 200;
int simPointsPerFrameMax = 12;

// ---- Serial / reconnect ----
String preferredPort = defaultPreferredPort();
int baud = 115200;

String defaultPreferredPort() {
  String os = System.getProperty("os.name").toLowerCase();
  if (os.contains("win")) return "COM7";
  if (os.contains("linux")) return "/dev/ttyACM0";
  return "";
}

int reconnectEveryMs = 2000;
int lastReconnectTryMs = 0;
// když je port objekt pořád "živý", ale data netečou (po odpojení USB),
// tak ho po timeoutu shodíme a necháme ensureConnected() znovu připojit
int noDataDisconnectMs = 1200;  // po kolika ms bez dat udělat disconnect
int connectGraceMs = 2500;      // grace po connectu (aby se to hned neshodilo)
int lastConnectMs = 0;


// ---- kreslení ----
float scaleFactor = 3.0;
boolean havePrev = false;
float prevX, prevY;
ArrayDeque<PVector> queue = new ArrayDeque<PVector>();
int segmentsPerFrame = 20;

// ---- UI / stav ----
String status = "WAITING";
int lastDataMs = 0;

// poslední souřadnice
float lastX = 0, lastY = 0;
float lastDrawX = 0, lastDrawY = 0;
boolean haveXY = false;

boolean debugUI = false;

PGraphics canvasLayer;  // stopa
PGraphics hud;          // debug overlay

// tlačítko (viditelné jen v debug)
int btnX = 20, btnY = 20, btnW = 170, btnH = 44;

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

// drift (aby se to v SIM tolik nepřekrývalo)
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
// 10 barev stopy (HSB: hue,sat,bri)

// HSB palette: {H, S, B}  (H:0-360, S:0-100, B:0-100)
float [][] STROKE_COLORS = {
  {  0,  0,  0},  // 0: BLACK
  {  0, 90, 85},  // 1: RED
  { 60, 90, 95},  // 2: YELLOW
  {220, 90, 85},  // 3: BLUE

  { 30, 90, 92},  // 4: ORANGE   (RED + YELLOW)
  {120, 90, 80},  // 5: GREEN    (YELLOW + BLUE)
  {270, 85, 80},  // 6: PURPLE   (BLUE + RED)

  { 25, 70, 45},  // 7: BROWN    (RED + YELLOW + BLUE) ~ “pigment mix”
  {  0,  0, 100}   // 8: WHITE (hodí se třeba pro pozadí / speciální režimy)
};


int strokeColorIdx = 1;

// 5 pozadí (HSB: hue,sat,bri)
float[][] BG_COLORS = {
  {0, 0, 100},    // bílá
  {0, 0, 94},     // světle šedá
  {45, 10, 98},   // papír
  {220, 30, 14},  // tmavě modrá
  {0, 0, 0}       // černá
};
int bgIdx = 0;

// ============================
// CURRENT COLORS (derived from palette)
// ============================
float lineHue, lineSat, lineBri;
int inkHue, inkSat, inkBri;
int sandHue, sandSat, sandBri;

// ============================
// BRUSH (ink) settings
// ============================
boolean INK_MULTIPLY = true;
float inkBrushR = 2.0;
float inkAlpha = 18;
float inkSpacing = 2.0;

// ============================
// SAND settings
// ============================
float sandSpacing = 2.5;
int sandGrainsPerStep = 4;
float sandSpread = 3.0;
float sandMinR = 0.8;
float sandMaxR = 1.7;
float sandAlpha = 22;

// ============================
// LINE settings
// ============================
float lineWeight = 2.0;
float MIN_LINE_SEG_PX = 0.8;

// ============================
// “rychlejší pohyb = světlejší/řidší”
// (dist = délka segmentu v px)
// ============================
float SPEED_DIST_REF = 18.0;      // kolem téhle délky už se to výrazně zesvětluje
float SPEED_ALPHA_MIN = 0.30;     // min násobek alfy (rychlé = ~0.3x)
float SPEED_ALPHA_MAX = 1.25;     // max násobek alfy (pomalé = ~1.25x)
float SPEED_SPACING_GAIN = 0.05;  // rychlé = větší rozestupy stampů

void setup() {
  size(800, 800); // pro fullscreen dej fullScreen() nebo fullScreen(2)
  smooth();

  canvasLayer = createGraphics(width, height);
  hud = createGraphics(width, height);

  // canvas init
  canvasLayer.beginDraw();
  canvasLayer.colorMode(HSB, 360, 100, 100, 255);
  canvasLayer.background(BG_COLORS[bgIdx][0], BG_COLORS[bgIdx][1], BG_COLORS[bgIdx][2]);
  canvasLayer.endDraw();

  // nastav výchozí barvy stopy podle palety
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

  // 0) simulace -> queue
  if (USE_SIM) generateSimPoints();

  // 1) kresli segmenty do canvasLayer
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

  // adaptivně
  if (queue.size() > 500) segmentsPerFrame = 60;
  else if (queue.size() > 250) segmentsPerFrame = 35;
  else segmentsPerFrame = 20;

  // hard watchdog: po odpojení USB myPort často zůstane != null,
  // ale data už netečou -> po timeoutu udělej disconnect, aby se spustil auto-reconnect
  if (!USE_SIM && myPort != null) {
    int now = millis();
    boolean inGrace = (now - lastConnectMs) < connectGraceMs;
    boolean timedOut = (now - lastDataMs) > noDataDisconnectMs;
  
    if (!inGrace && timedOut) {
      println("No data for " + (now - lastDataMs) + "ms -> disconnect & auto-reconnect");
      disconnectSerial();
      status = "ERROR"; // nebo "WAITING" pokud nechceš červený stav
    }
  }


  // data watchdog
  if (!USE_SIM && status.equals("RUN") && millis() - lastDataMs > 1500) {
    status = "WAITING";
  }

  // log (jen debug)
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
      " bgIdx=" + bgIdx
    );
    linesIn = 0;
    segmentsDrawn = 0;
    lastLogMs = now;
  }

  // display
  image(canvasLayer, 0, 0);

  if (debugUI) {
    drawHUD();
    image(hud, 0, 0);
  }
}

// ============================
// Palette apply helpers
// ============================
void applyStrokePalette() {
  float[] c = STROKE_COLORS[strokeColorIdx];

  // LINE
  lineHue = c[0];
  lineSat = c[1];
  lineBri = c[2];

  // BRUSH (ink) – stejné, jen jako int
  inkHue = round(c[0]);
  inkSat = round(c[1]);
  inkBri = round(c[2]);

  // SAND – lehce “pískovější” = menší sat, větší bri
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
  // dist 0..SPEED_DIST_REF -> vyšší alfa, dist vyšší -> nižší alfa
  float t = constrain(dist / SPEED_DIST_REF, 0, 2.5);
  // t=0 => ~MAX, t=1 => ~1.0, t>1 => jde k MIN
  float f = lerp(SPEED_ALPHA_MAX, 1.0, constrain(t, 0, 1));
  if (t > 1) f = lerp(1.0, SPEED_ALPHA_MIN, constrain((t - 1.0) / 1.5, 0, 1));
  return f;
}

float spacingFactorFromDist(float dist) {
  return 1.0 + dist * SPEED_SPACING_GAIN;
}

// ============================
// RENDER: LINE
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

// ============================
// RENDER: BRUSH (ink)
// ============================
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

// ============================
// RENDER: SAND
// ============================
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

  // rychlejší = méně zrnek
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

  float driftAccel = 0.20;
  float driftDamp  = 0.995;
  float driftMaxV  = 12.0;
  float driftMaxR  = 140.0;
  float noiseAmp = 0.8;

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

    // drift
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

    float drawX = width/2 + x * scaleFactor;
    float drawY = height/2 - y * scaleFactor;

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
  hud.noStroke();
  hud.fill(255);
  hud.rect(10, 10, 560, 270);

  boolean hover = mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= btnY && mouseY <= btnY+btnH;
  hud.stroke(0);
  hud.fill(hover ? 230 : 245);
  hud.rect(btnX, btnY, btnW, btnH, 10);

  hud.fill(0);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(16);
  hud.text("CALIBRATE (C)", btnX + btnW/2, btnY + btnH/2);

  hud.textAlign(LEFT, TOP);
  hud.textSize(14);
  hud.fill(0);

  String srcText = USE_SIM ? ("SIM (T=toggle, 1/2/3 mode)") : ("SERIAL (T=toggle)");
  String portText = (myPort == null) ? "DISCONNECTED" : "CONNECTED";
  hud.text("Source: " + srcText, 20, 70);
  hud.text("Serial: " + portText + " (R=reconnect)", 20, 90);
  hud.text("Status: " + status, 20, 110);
  hud.text("Q=color (" + (strokeColorIdx+1) + "/10)   W=render (" + renderModeName() + ")   E=bg (" + (bgIdx+1) + "/5)", 20, 130);
  hud.text("Queue: " + queue.size() + "   FPS: " + nf(frameRate, 0, 1), 20, 150);

  if (haveXY) {
    hud.text("x: " + nf(lastX, 0, 2) + "   y: " + nf(lastY, 0, 2), 20, 170);
    hud.text("px: " + nf(lastDrawX, 0, 1) + "  py: " + nf(lastDrawY, 0, 1), 20, 190);
  } else {
    hud.text("x,y: (čekám na data)", 20, 170);
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
    if (line.equals("CALIB_OK"))    { status = "RUN"; lastDataMs = millis(); return; }

    if (line.equals("x,y")) return;
    if (line.startsWith("OK,")) return;
    if (line.startsWith("ERR,")) { status = "ERROR"; return; }
    if (line.equals("CLR_OK")) return;

    String[] parts = split(line, ',');
    if (parts.length != 2) return;

    float x = float(parts[0]);
    float y = float(parts[1]);

    float drawX = width/2 + x * scaleFactor;
    float drawY = height/2 - y * scaleFactor;

    lastX = x; lastY = y;
    lastDrawX = drawX; lastDrawY = drawY;
    haveXY = true;

    queue.addLast(new PVector(drawX, drawY));
    lastDataMs = millis();
    if (!status.equals("CALIBRATING")) status = "RUN";

    linesIn++;
    while (queue.size() > 12000) queue.removeFirst();

  } catch(Exception e) {
    println("Serial error: " + e);
    disconnectSerial();
    status = "ERROR";
  }
}

void mousePressed() {
  if (!debugUI) return;
  if (mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= btnY && mouseY <= btnY+btnH) {
    sendCalibrate();
  }
}

void keyPressed() {
  if (key == 'd' || key == 'D') debugUI = !debugUI;

  // Q = barva (10)
  if (key == 'q' || key == 'Q') {
    strokeColorIdx = (strokeColorIdx + 1) % STROKE_COLORS.length;
    applyStrokePalette();
    println("STROKE_COLOR => " + (strokeColorIdx+1) + "/10");
  }

  // W = render mód (line/brush/sand)
  if (key == 'w' || key == 'W') {
    RENDER_MODE = (RENDER_MODE + 1) % 3;
    println("RENDER_MODE => " + renderModeName());
  }

  // E = pozadí (5) + clear
  if (key == 'e' || key == 'E') {
    bgIdx = (bgIdx + 1) % BG_COLORS.length;
    applyBackgroundAndClear();
    println("BACKGROUND => " + (bgIdx+1) + "/5 (cleared)");
  }

  // toggle zdroj dat
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

  // sim módy
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

  if (key == ' ') {  // clear canvas
    applyBackgroundAndClear();
  }
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
    if (myPort != null) myPort.stop();
  } catch(Exception e) {}
  myPort = null;
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

    // 1) preferovaný port
    if (preferredPort != null && preferredPort.length() > 0) {
      for (String p : ports) {
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

    String os = System.getProperty("os.name").toLowerCase();

    // 2) Linux/RPi fallback
    if (os.contains("linux")) {
      for (String p : ports) {
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
      for (String p : ports) {
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
    }

    // 3) Windows fallback
    if (os.contains("win")) {
      for (String p : ports) {
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

    // 4) last resort
    preferredPort = ports[0];
    myPort = new Serial(this, ports[0], baud);
    myPort.bufferUntil('\n');
    println("Connected (last resort) to " + ports[0]);
    lastConnectMs = millis();
    status = "WAITING";

  } catch(Exception e) {
    println("Connect failed: " + e);
    myPort = null;
    status = "ERROR";
  }
}
