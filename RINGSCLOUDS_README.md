# RingsClouds su M-VAVE FM-1: guida breve

Pre-release. Felucca 1.6.1 con il motore PHYS (Rings), l'effetto Clouds, la mod matrix estesa, LFO B, STEP e CC come sorgenti.

> **Attenzione.** Il firmware non è stato provato sul dispositivo. La CPU sul dispositivo non è misurata; i numeri sono stime
> dall'host (vedi `PORTING_NOTES.md`, §4–§6). Installalo a tuo rischio e fai prima il backup (sezione 4).

## 1. Installazione

1. Apri il [web installer](https://hugelton.github.io/Felucca/) in Chrome o Edge con l'FM-1 collegato via USB.
2. Scegli il pacchetto `felucca-<versione>.fwsc` della release (quando la release lo include) e premi Install.
3. Per compilarlo da sorgente invece, vedi [BUILDING.md](BUILDING.md): `./build.sh --release <versione>`.

## 2. Pagine nuove

Con quattro manopole per pagina.

| Pagina | Manopole | Cosa fa |
| --- | --- | --- |
| `CLOUDS` | MODE, POSN, SIZE, PTCH | Modo (OFF, GRAN, STRT, LOOP, SPEC), posizione e dimensione del grano, pitch |
| `CLOUDS 2` | DENS, TEXT, FRZ, QUAL | Densità, texture, freeze, qualità (HI ST, HI MO, LO ST, LO MO) |
| `BLEND` | WET, SPRD, FDBK, REV | Mix dry/wet, spread stereo, feedback, riverbero di Clouds |
| `CLOUDS 3` | SRC, TRIG | Quali parti entrano in Clouds (PHYS, T1–T4, ALL) e trigger da nota |
| `LFO B` | RATE, WAVE, SYNC, POL | Secondo LFO per traccia (SIN, TRI, SAW, SQR, S&H) |
| `MOD SRC` | STEP, CC#, CSEQ | Valore dello step, controller MIDI scelto, parametro CSEQ |

Su tasti, nel layer FX (tieni FX): **F5 = CLOUDS FREEZE**, **G5 = CLOUDS TRIG**. Con FX LATCH il freeze si commuta.

Parametri del motore PHYS (Rings), come destinazioni della matrice o via pagine: E1..E8 (MODEL, STRUCT, BRIT, DAMP, POS, …).

## 3. Mod matrix

- **Sorgenti nuove:** `LFOB` (secondo LFO per traccia), `STEP` (valore dello step, bloccabile per step), `CC` (controller scelto con `CC#`).
- **Sorgenti già presenti:** `LFO`, `ENV`, `VEL`, `KEY`, `RAND`, `MODW`, `AT`, `EXPR`, `S&H`, `SLEW`.
- **Destinazioni nuove per Clouds:** `CPOS CSIZE CPTCH CDENS CTEX CWET CSPRD CFDBK CREV`, più i cancelli `CFRZ` (freeze) e `CTRG` (trig).
- **Destinazioni Rings:** E1..E8 del motore PHYS.
- Quattro slot per traccia (SRC / DST / AMT), quindi 16 in totale su quattro tracce.
- Freeze di Clouds: manopola FRZ, tasto F5, CC 107, `CSEQ` FRZ/BOTH, cancello `CFRZ`.
- Trig di Clouds: tasto G5, CC 108, `CSEQ` TRIG/BOTH su uno step, cancello `CTRG`.

## 4. Mappa CC MIDI

| CC | Funzione |
| --- | --- |
| 20–27 | CLOUDS POSN, SIZE, PTCH, DENS, TEXT, WET, SPRD, FDBK |
| 102 | CLOUDS REV |
| 103 | CLOUDS MODE (0–127 diviso in OFF, GRAN, STRT, LOOP, SPEC) |
| 104 / 105 / 106 | CLOUDS QUAL / SRC / TRIG |
| 107 | FREEZE (≥ 64 attivo; CC 121 lo spegne) |
| 108 | TRIG (impulso al passaggio a ≥ 64) |
| 109–116 | RINGS E1..E8 della traccia del canale |
| 117 / 118 / 119 | LFO B RATE / WAVE / SYNC |

I CC di Clouds sono globali (un solo Clouds). Quelli di Rings riguardano la traccia che il canale suona. Un CC assegnato con MIDI LEARN ha la precedenza su questa mappa. Qualunque CC può modulare una destinazione come sorgente `CC`.

## 5. Tornare al firmware ufficiale V15

**Prima fai il backup.** Il passaggio cancella il firmware Felucca. **Le tue canzoni Felucca non si aprono con la V15.**

1. Scarica il firmware ufficiale **FM-1 V15** da M-VAVE e tienilo come `FM-1.fwsc` non modificato.
2. Apri il [web installer](https://hugelton.github.io/Felucca/) con l'FM-1 collegato via USB.
3. Scegli **Return to official V15**.
4. Seleziona il file `FM-1.fwsc`. Il browser lo verifica in locale, senza inviarlo a un server.
5. Premi **Back up and return to official V15**. Prima salva il backup completo sul computer: l'installer chiede conferma che sia salvato.
6. Aspetta la fine dell'installazione e riavvia l'FM-1.

In alternativa, l'aggiornatore di M-VAVE ripristina la V15 ufficiale.

---

Documentazione completa, mappa CC e note tecniche: [PORTING_NOTES.md](PORTING_NOTES.md) (in italiano).
