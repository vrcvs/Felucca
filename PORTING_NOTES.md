# PORTING_NOTES — RingsClouds su M-VAVE FM-1

Fase di sola analisi, senza codice. Base: fork `vrcvs/Felucca` al commit `3a91058` (Felucca 1.6.1), con il submodule `firmware/engines` al commit `c46e008`.
Obiettivo: Rings → Clouds in cascata (pichenettes/eurorack, MIT), più modalità dei firmware alternativi, scale, sequencer e arpeggiatore.

Legenda: **[codice]** = letto nel repo · **[misura]** = numero misurato da Felucca (sul dispositivo o sull'host) · **[stima]** = mia stima, da verificare sul dispositivo.

---

## 1. Piattaforma (cosa c'è già in Felucca)

### Callback audio [codice]
- `firmware/src/audio.c` → `fm1_alnk0_irq()`, interrupt DMA ALNK0 (I2S verso codec esterno), doppio buffer.
- **Sample rate: 44 117,6 Hz** (24 MHz / 544). Nel codice vale `FS = 44100` (`firmware/engines/tools/gen_engine_tables.py`).
- **Block size**: mezzo buffer `HALF_FRAMES = 128` frame (2,9 ms), renderizzato a sotto-blocchi di **`CTL = 32` campioni** (control rate ≈ 1378 Hz). Formato: Q15 interno, uscita 24 bit left-justified con ceiling a −6 dBFS.
- Carico misurato nell'ISR stessa (`song.cpu_q8`, media su ~93 ms). Se due mezzi buffer consecutivi superano l'85 % scatta il **voice shedding** (`shed_voice`).

### Secondo core [codice]
- `firmware/hal/fm1_cpu1.h` (preso da X0X, Melodee e Jangada) e `fx.c` (`mix_part`, `mix_split`).
- Il core 1 riceve un job alla volta tramite una mailbox, e solo dentro l'ISR audio, che aspetta che finisca. Lo split è **per parte**: `part_cost` con `ENG_VCOST` decide quali parti renderizzare sul core 1.
- **Attenzione: il secondo core non è affidabile su tutti gli FM-1** (fm1-x0x#10). Su alcuni esemplari il core 1 legge parole sbagliate dalla RAM. Felucca lo tiene **OFF di default** (MENU > SYSTEM > 2ND CORE) e torna a un solo core al primo job sbagliato.
- Conseguenza per noi: il firmware deve **funzionare con un core solo**. Il secondo core può solo aggiungere qualità.

### Budget CPU [misura]
- Il clock della CPU non è scritto nel codice. La conversione disponibile è **~1,7 % di un core ogni 100 istruzioni host per campione** (`tests/fm6_test.c`), quindi un core intero equivale a circa 5 900 istruzioni host per campione a 44,1 kHz.
- La conversione non vale per tutti i motori: per **PHYS MODAL (12 modi) il costo misurato sul dispositivo è ~13 % per voce** (`tests/phys_test.c`), circa il doppio di quanto direbbe il rapporto host. Per i filtri risonanti in virgola fissa uso quindi il dato misurato di PHYS.
- Strumenti esistenti:
  - `tests/regress.c` + `cpu_baseline.txt`: istruzioni host per campione, preset per preset (PHYS 840–1451, GRAIN 674–906, a 3 voci o con il pool di grani).
  - `tests/target_budget.py`: costo statico dei loop nel disassemblato pi32v2.
- **La CPU (pi32v2) non ha FPU**: tutto il DSP di Felucca è in virgola fissa ("no float anywhere", `phys_dsp.c`).

### RAM e PSRAM [codice + stima]
- SRAM interna 512 KiB (0x01C00000–0x01C80000), divisa da `firmware/app.ld`:

  | Regione | Dimensione | Uso |
  | --- | --- | --- |
  | RAMTEXT | 24 KiB | codice in RAM |
  | RAM (.data/.bss) | 72 KiB | variabili toccate dagli interrupt |
  | POOL | 360 KiB | buffer grandi |
  | NOINIT | ~15 KiB | dati che sopravvivono al reset |

- Occupazione stimata compilando `felucca.c` a 32 bit con gcc host (le stesse dimensioni degli oggetti):
  - POOL ≈ **326 KB su 368 KB → ~40 KB liberi** (`build.py` impone almeno 8 KiB di margine);
  - .bss ≈ 55 KB e .data ≈ 16 KB su 72 KiB → pochi KB liberi.
- Blocchi principali nel POOL:

  | Buffer | Dimensione | Uso |
  | --- | --- | --- |
  | `dly_buf` | 128 KB | linea del DELAY FX |
  | `cv_px` | 58 KB | canvas UI |
  | `eng_mem` | 51 KB | 4 × 12,9 KB, stato dei motori per parte |
  | `sl_buf` | 32 KB | SLICER |
  | `rev_comb` | ~10 KB | riverbero |

- **PSRAM: nessun uso e nessun riferimento** nel codice (nessuna PSRAM o SDRAM in `.ld`, `.c`, `.h`). Da trattare come assente finché non si verifica l'hardware.
- Flash: immagine XIP fino a ~568 KB (`LENGTH = 0x8DFBC`). Le tabelle vanno in flash e costano poco in RAM.

### PHYS: quanto Rings c'è già [codice]
`eng_phys.c` + `phys_dsp.c` (DaisySP, MIT) + `phys_symp.c` (Rings, MIT). Tutto in virgola fissa, 3 voci per parte.

| Modello | Origine | Differenze da Rings |
| --- | --- | --- |
| MODAL | ModalVoice di DaisySP (derivato da Rings) | **12 modi** (Rings ne usa fino a ~64 divisi tra le voci); eccitazione a impulso o "bow" a colpi casuali |
| STRNG | String di DaisySP, Karplus-Strong esteso | ponte curvo e dispersione come Rings; pluck a burst di rumore |
| MEMB | banco modale con i modi di una membrana | non è un modello di Rings |
| SYMP | stringa + **3** corde simpatiche (`phys_symp.c`, da `part.cc`/`string.cc`) | Rings ne ha fino a 7, con copie detunate e la String completa (IIR, pickup mosso da LFO); qui ci sono linee semplici da 256 campioni a metà rate |

**Cosa manca rispetto a Rings:**
- il resonator modale completo (risoluzione variabile con la polifonia, posizione con CosineOscillator);
- i modelli FM voice e "String and Reverb" (sympathetic quantizzato);
- l'easter egg "Disastrous Peace" (string synth con chorus e reverb);
- la gestione della polifonia 1/2/4 e lo strum interno;
- il riverbero di Rings;
- l'ingresso audio come eccitatore: l'FM-1 non ha ingresso audio, quindi solo eccitazione interna.

Il test di riferimento `tests/phys_ref.cpp` confronta già il fixed point con il float di DaisySP. È un modello da replicare.

### Toolchain [codice]
- JieLi clang 4.0.1 per pi32v2, solo Linux x86-64 (Docker su macOS), più 3 file dell'SDK AC79 (Apache-2.0). Si lancia con `./build.sh`, che usa `tools/build.py`.
- Felucca è **C puro**, in unity build (`felucca.c` include tutto) e senza libstdc++. Rings e Clouds sono C++ in float, quindi **vanno tradotti in C a virgola fissa**, come è già stato fatto per PHYS.
- Test host: `tests/run_tests.sh`, `hostsim.c`.

### Emulatore browser [codice]
**Sì.** `web/emu/` (Emscripten → WASM in AudioWorklet) usa lo stesso ISR audio e la stessa UI su un FM-1 simulato. È online come "Try it in your browser". È ottimo per sviluppare l'interfaccia, ma **non misura la CPU del dispositivo**.

### Già presenti e riutilizzabili [codice]
- Sequencer a 64 step: `seq.c`
- Scale e quantizzazione (`P_ROOT`, `P_SCALE`, `P_QUANT`) e accordi: `chord.c`
- Arpeggiatore (in `seq.c`, `chord.c`, `params.c`)
- MIDI USB e TRS, preset e progetti in flash, editor web

---

## 2. Stime per il port (a 44,1 kHz, virgola fissa)

### Rings (4 voci, 48 kHz in origine) [stima]
**CPU**, in percentuale di un core. Il passaggio da 48 a 44,1 kHz fa risparmiare circa l'8 %.

| Modalità | Stima | Come è calcolata |
| --- | --- | --- |
| Modal (~64 modi totali) | **~70–85 %** | dal dato misurato di PHYS, ~1,1 % per modo, più eccitatore e limiter |
| String 4 voci (dispersione, IIR) | ~30–40 % | |
| Sympathetic (fino a 8 corde) | ~45–60 % | |
| FM voice | ~20–30 % | |
| Riverbero | +5–8 % | se non si riusa quello di Felucca |

**RAM** (16 bit dove possibile):

| Voce | Dimensione |
| --- | --- |
| 4 String (linee 2048 + 512 campioni) | ~20 KB (40 KB in 32 bit) |
| Stato dei modi | ~2 KB |
| Buffer riverbero (originale 32 768 × 16 bit) | ~64 KB, oppure 0 riusando il riverbero di Felucca |

Totale: **~25–90 KB**.

### Clouds (buffer ~118 KB, 32 kHz in origine) [stima]
**CPU**, in percentuale di un core. Il passaggio da 32 a 44,1 kHz moltiplica il costo per 1,38.

| Modalità | Stima | Note |
| --- | --- | --- |
| Granulare, fino a 64 grani stereo, con diffuser, pitch shifter e riverbero | **~60–95 %** | dal costo di GRAIN su host, corretto ×2 come PHYS |
| Looping delay o stretch (WSOLA) | ~30–45 % | |
| Spectral (phase vocoder, FFT grande) | **~80–120 %** | la modalità più a rischio |

**RAM:**
- Buffer audio: 118 KB. A 44,1 kHz in stereo a 16 bit fanno **~1,3 s** invece di ~1,85 s. Con la modalità 8 bit μ-law di Clouds diventano ~2,7 s.
- Area di lavoro dei processori (in originale nella CCM da ~64 KB): ~20–64 KB, al massimo nello spectral.

Totale: **~140–185 KB**.

### Totale e dove trovare la RAM
Servono circa **165–275 KB**, ma con Felucca così com'è ne restano liberi ~40 KB.

Un firmware dedicato RingsClouds può però rinunciare a parti di Felucca:

| Cosa si toglie | KB recuperati |
| --- | --- |
| `dly_buf` (sostituito dal delay e looper di Clouds) | 128 |
| `eng_mem` (gli altri motori non servono) | 51 |
| `sl_buf` (SLICER) | 32 |
| riverbero di Felucca (se si usa quello di Rings o Clouds) | ~10 |

Si arriva a **~260 KB**. Nel caso peggiore bisogna stringere `cv_px` (UI).

---

## 3. Verdetto: **fattibile con compromessi**

Tutto in qualità piena non è realistico su un core solo. Rings Modal da solo prende già ~75 % del core, e Clouds granulare un altro 60–95 %. In più il secondo core non è garantito su tutti gli esemplari.

**Compromessi proposti, in ordine di impatto:**

1. **Virgola fissa e C**: è obbligatorio, la CPU non ha FPU. Ogni modulo va validato contro l'originale in float con un test host (come `phys_ref.cpp`).
2. **Rings sul core 0, Clouds sul core 1** quando il secondo core è disponibile. Senza secondo core si passa a "Clouds lite": 16–24 grani, niente spectral, Rings con meno modi.
3. **Modi di Rings ridotti**: ~32–40 totali invece di ~64, oppure piena risoluzione solo con polifonia 1 o 2.
4. **Grani di Clouds limitati** (32, al massimo 40) con riduzione automatica sotto carico, sfruttando lo shedding che Felucca ha già.
5. **Buffer di Clouds più corto** (~1,3 s) oppure a 8 bit μ-law (~2,7 s). In alternativa si fa girare Clouds internamente a 22,05 kHz con SRC: più tempo di buffer, ma meno alte frequenze.
6. **Spectral mode** come opzione, con FFT ridotta, oppure tolto.
7. **Un solo riverbero condiviso** invece di tre (Rings, Clouds, Felucca).
8. **Firmware alternativi**: Parasites (Clouds: Oliverb, Resonestor, Beat repeat, Spectral clouds) e l'easter egg di Rings.
   - Prima della fase di codice va **verificata la licenza di Parasites**. Il fork è GPL-3, che accetta codice MIT e GPL.
   - La priorità va data ai modi più leggeri: Oliverb e Beat repeat prima di Resonestor.
9. **Niente ingresso audio** sull'FM-1: Rings usa solo l'eccitazione interna (strum, MIDI, sequencer), Clouds processa solo l'uscita di Rings.

**Scale, sequencer e arpeggiatore** ci sono già in Felucca. Si riusano agganciando Rings come motore e Clouds come FX di bus.

**Primo passo consigliato:** portare un solo resonator di Rings in virgola fissa, misurarlo sul dispositivo (`cpu_q8` e il meter di sistema) e ricalibrare queste stime prima di decidere numero di modi e di grani.

---

## 4. Stato dell'implementazione: Rings integrale in PHYS

Aggiornato dopo la fase di codice. Tutto sotto è **[misura]** sull'host (Linux x86-64, gcc), salvo dove scritto altrimenti. **Niente è stato provato sul dispositivo.**

### Fatto

- `firmware/src/rings_dsp.c` (MIT, porto in virgola fissa del codice di Emilie Gillet) + `tools/gen_rings_tables.py` (tabelle: rigidità, decadi, compensazione di ritardo, quantizzatore FM). In `eng_phys.c` il MODEL passa da 4 a **11 valori**: `MODAL STRNG MEMB SYMP` (invariati, stessi hash golden) più `R.MOD R.SYM R.STR R.FM R.QNT R.S+R R.DSP`.
- **Sei modelli**: modale, corde simpatiche, corda, voce FM, corde simpatiche quantizzate, corda + riverbero. **Più l'easter egg Disastrous Peace** (`R.DSP`): 12 voci × 3 armoniche polyBLEP, registrazione, inviluppo AD, i sei effetti (formanti, chorus, riverbero, formanti 2, ensemble, riverbero 2) e il limitatore.
- **Polifonia 1/2/4** (E5 POLY), **eccitatore interno** (il Plucker di Rings: rumore filtrato, posizione, cutoff da BRIGHTNESS) e **strum** (ogni nota-on colpisce la voce successiva, come `Part::Process`; le precedenti continuano a suonare).
- **Parametri** (EDIT 1/2): E1 STRU(cture), E2 BRIG(htness), E3 DAMP(ing), E4 POSN, E5 POLY, E6 TONC (R.SYM: nota di riferimento, 0 = una quarta sotto la prima nota) / CHRD (R.QNT e R.DSP, 0..10), E7 TUNE (±12 semitoni) / FX (R.DSP). **Frequency** = la nota (tastiera, MIDI, sequencer, arp, bend e glide: l'ultima voce colpita segue il bend). Scale, sequencer e arpeggiatore di Felucca funzionano come sempre.
- 7 preset (`R MODAL` … `R DISASTROUS`, con ADSR a sustain pieno), voci nei golden (`tests/golden.txt`: +7 righe, le 98 esistenti non cambiano), demo in `build/phys_demo`.
- Conclusioni del §3 rispettate: virgola fissa, un core solo (nessun thread, nessuno stato condiviso tra core: l'arena è di una parte sola), **niente ingresso audio**, modi ridotti, linee più corte, un riverbero scalato, validazione contro il float.

### Come è fatto (differenze dall'originale)

| Cosa | Rings | Qui |
| --- | --- | --- |
| Modi totali (tutte le voci) | 64 | 40 (`RG_MODES_TOTAL`; i test contro il float usano 64) |
| Linea della corda / dispersione | 2048 / 1024 | 1024 / 512 (le note molto basse hanno una linea più corta: intonazione limitata in basso) |
| Riverbero | 16 384 campioni a 32 kHz | 8192 campioni, tempi scalati; il guadagno del loop segue `time^0.55` |
| Uscita | stereo (out/aux) | **mono**: modale out+aux, corde out−aux per voce, DP (L+R)/2; il pan di PHYS fa il resto |
| Blocco | 24 campioni | 32 (`CTL`) |
| Note filter di DP | sì | no (la nota entra a ogni blocco senza filtro) |
| Stato | per voce | **un'arena unica** (31 872 B, `.pool`) di una parte alla volta, non in `eng_mem`; chi suona per ultimo un modello R.* la prende, le altre parti R.* tacciono |
| Voci host | — | la voce host che ha iniziato l'ultima nota è la "portante" di tutto il suono (il suo ADSR e la velocity lo modulano); le altre voci della parte non renderizzano |

I tre punti che contano per la precisione (da non rompere): le linee a 16 bit (corde in Q12, riverbero in Q14) **arrotondano** e quantizzano con **error feedback**, altrimenti ogni passaggio del loop di ritardo introduce un bias e una perdita (DC e decadimenti sbagliati); l'interpolazione di Hermite lavora in Q20; il filtro FIR/IIR di smorzamento arrotonda.

### Validazione contro il float (`tests/rings_ref.cpp`, `tests/rings_fixed.c`)

Il float è il codice di Rings (`rings/dsp/part.cc`, `string_synth_part.cc`) con `kSampleRate = 44100` e le tabelle di accordi di Bryan Noll (come nel build originale: `tests/rings_override`). Per ogni modello: griglia di note, parametri, polifonia 1/2/4, accordi e strum; si confrontano **picchi spettrali** (entro 5 ct + la risoluzione FFT), **livello** (±8 dB, media in dB su 24 semi per i modelli a rumore), **decadimento** (±5 dB). Il modale e la FM sono deterministici e sono confrontati anche campione per campione.

| Modello | Casi | Falliti |
| --- | --- | --- |
| 0 modale / 1 simpatiche / 2 corda / 3 FM / 4 quantizzate / 5 corda+riverbero | 72–144 ciascuno | **0** |
| 6 Disastrous Peace (poly 1/2/4 × 6 fx × 3 config × 3 accordi) | 162 | **12** (vedi problemi aperti) |

Altri test nuovi (`tests/phys_test.c`, in `run_tests.sh`): stabilità di tutti e 7 i modelli per poly 1/2/4, note 36/60/96 e gli angoli dei parametri (3888 render: picco ≤ 1,5, niente NaN, DC e silenzio), torture a 11 modelli, e `tests/rings_cpu.sh`. Le suite esistenti che ho potuto eseguire (regress/golden, union, dualcore, project, phys) restano invariate. `run_tests.sh` intero **non** gira qui: si ferma a "run ./build.sh first" perché il firmware non si compila (vedi sotto).

### Build

- Le tabelle (`build/gen/rings_tables.h`) si generano con `tools/build.py` (voce aggiunta); I test (che includono tutto il firmware via `hostsim.c`) si compilano su host con gcc senza errori né warning nuovi rilevanti.
- **Cross-build per pi32v2 NON eseguito**: la toolchain JieLi e l'SDK rispondono 403 dal proxy (non aggirato). Quindi la compilazione per il chip, l'ingombro reale in `.pool`/`.bss`, i controlli di `build.py` (margine 8 KiB nel POOL) e il `target_budget.py` sul disassemblato non sono verificati.

### CPU misurata

**Nell'emulatore: non misurata** (`emcc` non c'è in questo ambiente; non ho provato altri modi per raggiungerlo). Misura disponibile: istruzioni host per campione sotto valgrind/callgrind (`tests/rings_cpu.sh`), una parte, 8 tasti tenuti (3 voci PHYS / strum a rotazione sui Rings), differenza tra render e solo setup. Questa macchina conta istruzioni x86-64 e i numeri non sono quelli del Mac in `cpu_baseline.txt` (circa ×1,4 più alti): vanno letti **contro i preset PHYS di questa stessa esecuzione**.

| Preset | Poly | Istruzioni / campione |
| --- | --- | --- |
| PLUCK (riferimento, il più leggero) | — | 1145 |
| BOWED METAL (riferimento, il più pesante) | — | 2047 |
| R MODAL | 1 / 4 | 1637 / 2351 |
| R SYMPATHETIC | 1 / 4 | 2364 / 2508 |
| R STRING | 2 / 4 | 1339 / 2078 |
| R FM VOICE | 1 / 4 | 816 / 1461 |
| R QUANTISED | 2 / 4 | 2407 / 2500 |
| R STR+REVERB | 1 / 4 | 1491 / 2600 |
| R DISASTROUS | 1 / 4 | 2324 / 1324 (a poly 4 solo un gruppo suona e gli altri oscillatori a guadagno zero sono saltati) |

**[stima, da verificare sul dispositivo]** Il dato misurato sul dispositivo per BOWED METAL (3 voci × ~13 %) è ~39 % di un core; scalando con questi rapporti il peggior caso Rings (R STR+REVERB a 4 voci, ~2600) sta attorno al **50 % di un core**, e i preset di default tra il 25 % e il 45 %. È sotto il budget di un core, ma senza secondo core non resta spazio per Clouds in cascata nel caso peggiore: serve ridurre i modi o la polifonia come previsto in §3.

### Problemi aperti

1. **Cross-build e RAM non verificati** (toolchain 403). L'arena è 31 872 B nel POOL; con i ~42 KB liberi stimati restano ~10 KB contro i 8 KiB richiesti da `build.py`: margine di ~2 KB, da confermare con `build.py`. Se non basta: scalare ancora il riverbero (8 KB in meno) o limitare R.S+R a poly ≤ 2.
2. **CPU sul dispositivo e nell'emulatore non misurata** (numeri sopra: host, relativi).
3. **Disastrous Peace con riverbero (fx REVB, REV2) e POSN alto (≥ 0,9)**: livello +2…+6 dB sopra il float nella coda (12 casi su 162; gli altri 150 e tutte le altre fx passano). Il riverbero fisso decade ~1 dB in meno del float per finestra a tempi alti; il suono non cambia carattere. Da rifinire (o allargare la tolleranza).
4. **Un'arena sola**: due parti con modelli R.* contemporanei non si possono avere; la seconda che suona toglie la voce alla prima (silenzio, non crash). Il motore conserva comunque i suoi 3 voci/parte per MODAL/STRNG/MEMB/SYMP.
5. **Coda dopo il note-off**: i risuonatori e il riverbero tacciono quando finisce il release dell'ADSR della voce portante (`phys_post` per far continuare la coda come in §3 non è stato aggiunto, per non toccare `voice.c`/`fx.c`). Con release lungo (i preset hanno R 40–100) il suono si spegne come in Rings, ma non oltre.
6. **Modi/linee/riverbero ridotti, uscita mono, blocco da 32, niente NoteFilter in DP** (tabella sopra): l'intonazione e il livello sono entro le tolleranze, il timbro delle note molto basse delle corde e le code del riverbero differiscono dall'originale.
7. **Test float-vs-fixed**: i modelli a rumore (1, 2, 4, 5) sono confrontati su medie di 24 semi e sulla risoluzione FFT, non campione per campione; il DC della corda+riverbero a BRIG 0 è un po' più negativo del float (~−0,005 contro ~0 di media, −40 dB). Il limite di DC in `phys_test` è largo per questo (35 % del picco).
8. **Il TUNE/FREQ fine e il TONC** sono parametri di PHYS, non il CV/V-oct di Rings; l'interfaccia non ha ancora nomi dedicati nelle pagine HOME (knob 1–4 = STRU, BRIG, DAMP, POSN).
9. **Gain** dell'uscita (24000 su tutti i modelli R.*) tarato a orecchio sui livelli delle demo (picchi 4 000–16 000 su 32 768), non misurato su altoparlante.
10. **Clouds, ingresso audio, Parasites**: non toccati in questa fase.
11. **`tests/cpu_baseline.txt` senza i 7 preset nuovi**: il contatore di `regress.c` (kernel macOS) qui non esiste; sul Mac `BUDGET_UPDATE=1` li aggiunge. Anche `tests/target_budget.txt` (disassemblato pi32v2) va rigenerato col cross-build.
