# Kyvadlo — Processing vizualizace

Vykresluje stopu fyzického kreslícího kyvadla. Data (x,y výchylka, rozsah cca −1..1)
posílá **ESP32-S3 s BNO085** po USB serialu (115200 Bd); v produkci běží tato appka
na Raspberry Pi 5 připojeném k TV. Hlavní soubor: `kyvadlo/kyvadlo.pde`.

**Firmware ESP je v samostatném repozitáři `c:\Git\kyvadlo_esp\kyvadlo`** — tam je
`CLAUDE.md` s popisem celého systému, sériového protokolu a yaw korekce. Orientaci
obrazce (zarovnání po restartu, kompenzace yaw driftu) řeší primárně ESP
(snap na magnetometr + `yawHome` v NVS, příkaz `SET_HOME`); zdejší orientační
parametry (ROT90, flip, fine rotate) jsou jen doladění a měly by zůstat konstantní.

## Klíčové body kódu

- `serialEvent()` — parsuje řádky `x,y`; **každá textová zpráva ESP protokolu musí
  být explicitně odfiltrována**, jinak se ji pokusí parsovat jako souřadnice.
- Klávesy k ESP: `C` = CAL (kalibrace), `H` = SET_HOME, `Shift+H` = CLR_HOME,
  `R` = reconnect, `T` = přepnutí SIM/SERIAL, `D` = debug HUD.
- Auto-fallback: bez ESP běží simulace (fyzikální model kyvadla), po připojení ESP
  se automaticky vrátí na serial.
- Config (scale, orientace, barvy, render mód) se ukládá mimo repo:
  Windows `%APPDATA%\Kyvadlo\config.txt`, Linux `~/.config/kyvadlo/config.txt`.
- Render módy: BRUSH (inkoust) a SAND (písek); pozadí bílé/černé.

Komunikace s uživatelem česky.
