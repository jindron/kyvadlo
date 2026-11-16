# Kyvadlo — Processing vizualizace

Vykresluje stopu fyzického kreslícího kyvadla. Data (x,y výchylka, rozsah cca −1..1)
posílá **ESP32-S3 s BNO085** po USB serialu (115200 Bd); v produkci běží tato appka
na Raspberry Pi 5 připojeném k TV. Hlavní soubor: `kyvadlo/kyvadlo.pde`.

**Firmware ESP je v samostatném repozitáři `C:\Git\kyvadlo_esp`** — tam je
`CLAUDE.md` a `docs/` s popisem celého systému, sériového protokolu (`docs/protokol.md`)
a vnitřku tohoto sketche (`docs/architektura.md`, kap. 4). Orientaci obrazce řeší yaw
korekce v ESP; zdejší orientační parametry (ROT90, flip, fine rotate) jsou jen konstantní
pootočení vůči obrazovce a mají zůstat stejné. (`SET_HOME` / yaw snap z firmware 1 leží jen
v archivní ESP větvi `archiv/yaw-home`; firmware 2 to řeší jinak, viz
`C:\Git\kyvadlo_esp\docs\navrh-kalibrace.md`.)

## Klíčové body kódu

- `serialEvent()` → `handleSerialLine()` — parsuje řádky `x,y`; řádek se 2 částmi, kde
  `float()` dá NaN (text), se zahodí. Zprávy `CALIB_*` a platná data = ESP je „živé“
  (`espValidMs`).
- Klávesy k ESP: `C` = CAL (ruční nula), `R` = reconnect, `T` = ruční SIM/SERIAL
  (vypne auto-SIM do restartu), `D` = debug HUD.
- **`A` = srovnání natočení:** zobrazí vodicí čáru; po rozhoupání kyvadla podél ní druhé `A`
  spočítá směr kývání ze surových dat (`alignToGuide()`) a nastaví `ORIENT_FINE_DEG`.
  Firmware 2 drží výstup pevně vůči magnetickému poli, tohle je konstantní pootočení
  vůči obrazovce a dělá se jednou při instalaci.
- Auto-SIM: když ESP 3 s neposílá data ani `CALIB_*` (chybí, mlčí, hlásí jen `ERR,…`),
  běží náhodná simulace kyvadla (`updateSim()`: kývání → dokmitá → 10 s pauza → smazat →
  nové kývání). Žádný nápis o odpojení. Jakmile ESP pošle platný řádek, vrátí se na serial.
- Config (scale, orientace, barvy, render mód) se ukládá mimo repo:
  Windows `%APPDATA%\Kyvadlo\config.txt`, Linux `~/.config/kyvadlo/config.txt`.
- Render módy: BRUSH (inkoust) a SAND (písek); pozadí bílé/černé.

Komunikace s uživatelem česky.
