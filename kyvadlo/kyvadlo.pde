import processing.serial.*;
import java.util.ArrayDeque;

Serial myPort;

// ============================
// SIMULATION toggle
// ============================
boolean USE_SIM = false;           // <-- dej false, když chceš jet jen z ESP
int SIM_MODE = 1;                 // 1=Lissajous, 2=Spiro, 3=Random walk
float simRateHz = 200;            // kolik bodů za sekundu generovat (simulace)
int simPointsPerFrameMax = 12;    // pojistka, aby to nezabilo FPS

// ---- Serial / reconnect ----
// ---- Serial / reconnect ----
// Windows: COM7 (pokud existuje)
// Linux/RPi: /dev/ttyACM0 (pokud existuje)
String preferredPort = defaultPreferredPort();
int baud = 115200;

String defaultPreferredPort() {
  String os = System.getProperty("os.name").toLowerCase();
  if (os.contains("win")) return "COM7";
  if (os.contains("linux")) return "/dev/ttyACM0";
  return ""; // fallback později
}

int reconnectEveryMs = 2000;
int lastReconnectTryMs = 0;

// ---- kreslení ----
float scaleFactor = 3.0;
boolean havePrev = false;
float prevX, prevY;
ArrayDeque<PVector> queue = new ArrayDeque<PVector>();
int segmentsPerFrame = 20;

// ---- UI / stav ----
String status = "WAITING";  // WAITING / CALIBRATING / RUN / ERROR
int lastDataMs = 0;

// poslední souřadnice
float lastX = 0, lastY = 0;               // raw ze senzoru/simulace
float lastDrawX = 0, lastDrawY = 0;       // v pixelech na obrazovce
boolean haveXY = false;

boolean debugUI = false;                  // default: nic nevidět

PGraphics canvasLayer;                    // <-- stopa
PGraphics hud;                            // <-- debug overlay

// tlačítko (viditelné jen v debug)
int btnX = 20, btnY = 20, btnW = 170, btnH = 44;

// ---- logging ----
int linesIn = 0;          // kolik řádků přišlo za 1s (serial nebo sim)
int segmentsDrawn = 0;    // kolik segmentů vykresleno za 1s
int lastLogMs = 0;

// ============================
// SIM state
// ============================
float simT = 0;                      // čas simulace
float simCarry = 0;                  // pro přesné dávkování bodů
float rwX = 0, rwY = 0;              // random walk stav

// --- SIM "random drift" state (aby se to nepřekrývalo)
float driftX = 0, driftY = 0;
float driftVX = 0, driftVY = 0;

// ============================
// RENDER modes (0=line, 1=ink, 2=sand)
// Q/W/E přepínají render
// ============================
int RENDER_MODE = 1; // default INK

String renderModeName() {
  if (RENDER_MODE == 0) return "LINE";
  if (RENDER_MODE == 1) return "INK";
  return "SAND";
}

// ============================
// INK (brush) settings
// ============================
boolean INK_MULTIPLY = true;   // multiply = ink look

float inkBrushR = 2.0;         // poloměr otisku (px)
float inkAlpha = 18;           // 1..255 (nižší = pomalejší tmavnutí)
float inkSpacing = 2;          // vzdálenost otisků po segmentu (px)

int inkHue = 210;              // 0..360
int inkSat = 80;               // 0..100
int inkBri = 20;               // 0..100 (nižší = tmavší)

// ============================
// SAND settings
// ============================
float sandSpacing = 2.5;       // krok po segmentu (px)
int sandGrainsPerStep = 4;     // kolik zrnek na krok
float sandSpread = 3.0;        // rozptyl zrnek kolem stopy (px)
float sandMinR = 0.8;          // min velikost zrna
float sandMaxR = 1.7;          // max velikost zrna
float sandAlpha = 22;          // 1..255 (vrstvení = tmavne)

// barva písku v HSB (jemně hnědá)
int sandHue = 35;
int sandSat = 25;
int sandBri = 55;

// ============================
// LINE settings (pixelová čára)
// ============================
float lineWeight = 2.0;

// minimální délka segmentu (px), aby nevznikaly "tmavé tečky/kroužky"
float MIN_LINE_SEG_PX = 0.8;

void setup() {
  size(800, 800);   // pro fullscreen dej fullScreen() nebo fullScreen(2)
  smooth();

  // vrstvy
  canvasLayer = createGraphics(width, height);
  hud = createGraphics(width, height);

  // init canvas (HSB kvůli ink/sand barvě + alfa)
  canvasLayer.beginDraw();
  canvasLayer.colorMode(HSB, 360, 100, 100, 255);
  canvasLayer.background(0, 0, 100); // bílá v HSB
  canvasLayer.endDraw();

  if (!USE_SIM) {
    println(Serial.list());
    connectSerial();
  } else {
    status = "RUN";
    rwX = 0;
    rwY = 0;
  }

  lastDataMs = millis();
  lastLogMs = millis();
}

void draw() {
  if (!USE_SIM) ensureConnected();

  // 0) pokud jedeme simulaci, nasyp body do queue
  if (USE_SIM) generateSimPoints();

  // 1) vykresli další segmenty do canvasLayer
  int n = 0;
  if (!queue.isEmpty()) {
    canvasLayer.beginDraw();

    while (n < segmentsPerFrame && !queue.isEmpty()) {
      PVector p = queue.removeFirst();

      if (havePrev) {
        // render podle módu
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

      prevX = p.x;
      prevY = p.y;
      n++;
    }

    canvasLayer.endDraw();
  }

  // adaptivně (když se fronta hromadí)
  if (queue.size() > 500) segmentsPerFrame = 60;
  else if (queue.size() > 250) segmentsPerFrame = 35;
  else segmentsPerFrame = 20;

  // 2) hlídání, jestli “chodí data”
  if (!USE_SIM && status.equals("RUN") && millis() - lastDataMs > 1500) {
    status = "WAITING";
  }

  // 3) log jen když debug
  int now = millis();
  if (debugUI && now - lastLogMs >= 1000) {
    println(
      "LOG fps=" + nf(frameRate, 0, 1) +
      " lines/s=" + linesIn +
      " seg/s=" + segmentsDrawn +
      " queue=" + queue.size() +
      " seg/frame=" + segmentsPerFrame +
      " src=" + (USE_SIM ? ("SIM(mode " + SIM_MODE + ")") : ("SERIAL(" + (myPort == null ? "DISCONNECTED" : preferredPort) + ")")) +
      " render=" + renderModeName()
    );
    linesIn = 0;
    segmentsDrawn = 0;
    lastLogMs = now;
  }

  // 4) zobraz vrstvy
  image(canvasLayer, 0, 0);

  if (debugUI) {
    drawHUD();
    image(hud, 0, 0);
  }
}

// ============================
// RENDER: LINE (pixelová čára)
// ============================
void drawLineSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;

  // přeskoč moc krátké segmenty -> eliminuje "sytější kroužky"
  if (dx*dx + dy*dy < MIN_LINE_SEG_PX*MIN_LINE_SEG_PX) return;

  g.blendMode(BLEND);
  g.stroke(0, 0, 0, 255);       // černá v HSB
  g.strokeWeight(lineWeight);
  g.strokeCap(SQUARE);          // méně "koleček" než ROUND
  g.noFill();
  g.line(x1, y1, x2, y2);
}

// ============================
// RENDER: INK (stamps)
// ============================
void drawInkSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  if (INK_MULTIPLY) g.blendMode(MULTIPLY);
  else g.blendMode(BLEND);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dist = sqrt(dx*dx + dy*dy);
  if (dist < 0.0001) { g.blendMode(BLEND); return; }

  int steps = max(1, ceil(dist / inkSpacing));

  g.noStroke();
  g.fill(inkHue, inkSat, inkBri, inkAlpha);

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
// RENDER: SAND (grains)
// ============================
void drawSandSegment(PGraphics g, float x1, float y1, float x2, float y2) {
  g.blendMode(BLEND);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dist = sqrt(dx*dx + dy*dy);
  if (dist < 0.0001) return;

  int steps = max(1, ceil(dist / sandSpacing));

  g.noStroke();
  g.fill(sandHue, sandSat, sandBri, sandAlpha);

  for (int i = 0; i <= steps; i++) {
    float t = i / (float)steps;
    float bx = lerp(x1, x2, t);
    float by = lerp(y1, y2, t);

    for (int k = 0; k < sandGrainsPerStep; k++) {
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
      float ax = 90;
      float ay = 70;
      float fx = 2.0;
      float fy = 3.0;
      float phase = PI/3;
      x = ax * sin(TWO_PI * fx * simT + phase);
      y = ay * sin(TWO_PI * fy * simT);

    } else if (SIM_MODE == 2) {
      float R = 90;
      float r = 35;
      float d = 60;
      float t = TWO_PI * 0.20 * simT;
      x = (R - r) * cos(t) + d * cos(((R - r) / r) * t);
      y = (R - r) * sin(t) - d * sin(((R - r) / r) * t);

    } else {
      float stepSize = 2.5;
      rwX += random(-stepSize, stepSize);
      rwY += random(-stepSize, stepSize);
      rwX = constrain(rwX, -120, 120);
      rwY = constrain(rwY, -120, 120);
      x = rwX;
      y = rwY;
    }

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

    lastX = x;
    lastY = y;
    lastDrawX = drawX;
    lastDrawY = drawY;
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
  hud.rect(10, 10, 520, 260);

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

  String srcText = USE_SIM ? ("SIM  (T=toggle, 1/2/3 mode)") : ("SERIAL (T=toggle)");
  String portText = (myPort == null) ? "DISCONNECTED" : "CONNECTED";
  hud.text("Source: " + srcText, 20, 70);
  hud.text("Serial: " + portText + "  (R=reconnect)", 20, 90);
  hud.text("Status: " + status, 20, 110);
  hud.text("Render: " + renderModeName() + "  (Q=line, W=brush, E=sand)", 20, 130);
  hud.text("Queue: " + queue.size() + "   FPS: " + nf(frameRate, 0, 1), 20, 150);

  if (haveXY) {
    hud.text("x:  " + nf(lastX, 0, 2) + "    y:  " + nf(lastY, 0, 2), 20, 170);
    hud.text("px: " + nf(lastDrawX, 0, 1) + "   py: " + nf(lastDrawY, 0, 1), 20, 190);
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

    lastX = x;
    lastY = y;
    lastDrawX = drawX;
    lastDrawY = drawY;
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

  // Q/W/E => přepnutí renderu (funguje vždy: SIM i SERIAL)
  if (key == 'q' || key == 'Q') { RENDER_MODE = 0; println("RENDER => LINE"); }
  if (key == 'w' || key == 'W') { RENDER_MODE = 1; println("RENDER => INK"); }
  if (key == 'e' || key == 'E') { RENDER_MODE = 2; println("RENDER => SAND"); }

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

  // přepnutí sim módů (teď na 1/2/3)
  if (USE_SIM) {
    if (key == '1') { SIM_MODE = 1; println("SIM_MODE=1 (Lissajous)"); }
    if (key == '2') { SIM_MODE = 2; println("SIM_MODE=2 (Spiro)"); }
    if (key == '3') { SIM_MODE = 3; println("SIM_MODE=3 (Random walk)"); }
  }

  if (key == 'c' || key == 'C') sendCalibrate();

  if (key == 'r' || key == 'R') {
    if (!USE_SIM) {
      println("Manual reconnect...");
      connectSerial();
    }
  }

  if (key == ' ') {  // clear canvas
    canvasLayer.beginDraw();
    canvasLayer.background(0, 0, 100);
    canvasLayer.endDraw();

    havePrev = false;
    queue.clear();
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

    // 1) Zkus natvrdo preferredPort (COM7 na Win, /dev/ttyACM0 na Linux)
    if (preferredPort != null && preferredPort.length() > 0) {
      for (String p : ports) {
        if (p.equals(preferredPort)) {
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected to " + p);
          status = "WAITING";
          return;
        }
      }
    }

    String os = System.getProperty("os.name").toLowerCase();

    // 2) Linux/RPi fallback: ttyACM*, pak ttyUSB*, pak cokoliv
    if (os.contains("linux")) {
      for (String p : ports) {
        if (p.indexOf("ttyACM") >= 0) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (fallback ttyACM) to " + p);
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
          status = "WAITING";
          return;
        }
      }
    }

    // 3) Windows fallback: první COM*, pak cokoliv
    if (os.contains("win")) {
      for (String p : ports) {
        if (p.startsWith("COM")) {
          preferredPort = p;
          myPort = new Serial(this, p, baud);
          myPort.bufferUntil('\n');
          println("Connected (fallback COM) to " + p);
          status = "WAITING";
          return;
        }
      }
    }

    // 4) Poslední fallback: první port
    preferredPort = ports[0];
    myPort = new Serial(this, ports[0], baud);
    myPort.bufferUntil('\n');
    println("Connected (last resort) to " + ports[0]);
    status = "WAITING";

  } catch(Exception e) {
    println("Connect failed: " + e);
    myPort = null;
    status = "ERROR";
  }
}
