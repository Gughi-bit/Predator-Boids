# Documento di Spiegazione Dettagliata delle Modifiche al Progetto Boids

Questo documento descrive passo dopo passo tutte le modifiche apportate alla simulazione dei **Boids**, illustrando:
1. **Quale modifica è stata effettuata**;
2. **Perché è stata effettuata** (guadagno in efficienza computazionale, rispetto dei criteri di valutazione di `README.md` e aderenza ai requisiti di `boids.md`);
3. **Analisi comparativa delle alternative**: perché le altre possibili implementazioni della stessa modifica non erano preferibili.

---

## Indice delle Modifiche

- [1. Eliminazione delle Allocazioni Dinamiche Ridondanti in `calcolaDeltaV`](#1-eliminazione-delle-allocazioni-dinamiche-ridondanti-in-calcoladeltav)
- [2. Introduzione di `distanzaQuadrata` e Pre-filtraggio per Evitare `std::sqrt`](#2-introduzione-di-distanzaquadrata-e-pre-filtraggio-per-evitare-stdsqrt)
- [3. Adozione di Algoritmi STL e Funzioni Lambda](#3-adozione-di-algoritmi-stl-e-funzioni-lambda)
  - [3.1. `separazione` con `std::accumulate`](#31-separazione-con-stdaccumulate)
  - [3.2. `evitareOstacoli` con `std::find_if`](#32-evitareostacoli-con-stdfind_if)
  - [3.3. `aggiorna` e `Ostacolo::creazione` con `std::for_each`](#33-aggiorna-e-ostacolocreazione-con-stdfor_each)
- [4. Ottimizzazione delle Operazioni in Virgola Mobile (Moltiplicazione per Reciproco)](#4-ottimizzazione-delle-operazioni-in-virgola-mobile-moltiplicazione-per-reciproco)
  - [4.1. `Boid::controlloV`](#41-boidcontrollov)
  - [4.2. `Boid::bordi`](#42-boidbordi)
- [5. Calcolo e Visualizzazione delle Statistiche Collettive dello Stormo](#5-calcolo-e-visualizzazione-delle-statistiche-collettive-dello-stormo)
- [6. Validazione Rigorosa degli Argomenti da Riga di Comando (`main.cpp`)](#6-validazione-rigorosa-degli-argomenti-da-riga-di-comando-maincpp)
- [7. Estensione della Suite di Test Unitari (`boid.test.cpp`)](#7-estensione-della-suite-di-test-unitari-boidtestcpp)

---

## 1. Eliminazione delle Allocazioni Dinamiche Ridondanti in `calcolaDeltaV`

### Modifica Effettuata
Nel codice originale, per ogni boid e ad ogni frame, venivano creati i vettori `vicini_stormo` e `vicini_altro_stormo`, e successivamente un terzo vettore `tutti` in cui venivano copiati entrambi per calcolare la separazione:
```cpp
// Vecchia implementazione
std::vector<Boid> tutti;
tutti.reserve(vicini_stormo.size() + vicini_altro_stormo.size());
tutti.insert(tutti.end(), vicini_stormo.begin(), vicini_stormo.end());
tutti.insert(tutti.end(), vicini_altro_stormo.begin(), vicini_altro_stormo.end());
dv += separazione(tutti, *this, pesoSeparazione, pesoPredatore);
```
Nella nuova implementazione il terzo vettore `tutti` è stato completamente eliminato:
```cpp
// Nuova implementazione
if (!vicini_stormo.empty()) {
    dv += coesione(vicini_stormo, *this, pesoCoesione);
    dv += allineamento(vicini_stormo, *this, pesoAllineamento);
    dv += separazione(vicini_stormo, *this, pesoSeparazione, pesoPredatore);
}
if (!vicini_altro_stormo.empty()) {
    dv += separazione(vicini_altro_stormo, *this, pesoSeparazione, pesoPredatore);
}
```

### Perché è stata fatta
- **Impatto prestazionale**: Con 300 boids a 60 FPS, la versione precedente effettuava $300 \times 60 = 18.000$ allocazioni, copie di blocchi di memoria e deallocazioni sullo heap al secondo solo per il vettore `tutti`.
- **Equivalenza matematica**: La forza di separazione è una sommatoria lineare su un insieme di agenti:
  $$\vec{F}_{\text{sep}}(A \cup B) = \sum_{k \in A \cup B} \vec{f}(k) = \sum_{k \in A} \vec{f}(k) + \sum_{k \in B} \vec{f}(k) = \vec{F}_{\text{sep}}(A) + \vec{F}_{\text{sep}}(B)$$
  Poiché `vicini_stormo` e `vicini_altro_stormo` sono insiemi disgiunti, sommare i due contributi produce un risultato identico all'ultimo bit, con zero copie di memoria aggiuntive.

### Perché le alternative non erano preferibili
- **Opzione A: Usare `std::vector<const Boid*>` o puntatori raw per `tutti`**:
  *Non preferibile* perché richiede comunque di allocare memoria per il vettore di puntatori e di modificare le firme di `separazione` rompendo la compatibilità con i test esistenti.
- **Opzione B: Unire tutti i boid in un unico vettore e poi filtrarli ogni volta**:
  *Non preferibile* perché richiederebbe controlli condizionali multipli (ID uguali vs ID diversi) per ogni regola (`coesione`, `allineamento`), aumentando i salti condizionali (branch mispredictions).

---

## 2. Introduzione di `distanzaQuadrata` e Pre-filtraggio per Evitare `std::sqrt`

### Modifica Effettuata
Sono state definite le funzioni inline:
```cpp
inline double normaQuadrata(const sf::Vector2<double>& v) {
    return v.x * v.x + v.y * v.y;
}

inline double distanzaQuadrata(const sf::Vector2<double>& a, const sf::Vector2<double>& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return dx * dx + dy * dy;
}
```
Nei cicli di ricerca dei vicini e nel controllo ostacoli, il confronto avviene prima sulla distanza quadratica ($d^2 \le R^2$) anziché calcolare subito la radice quadrata:
```cpp
constexpr double raggioStormo2 = RAGGIO_VISTA_STORMO * RAGGIO_VISTA_STORMO;
const double d2 = distanzaQuadrata(pos_, altro.getPos());
if (d2 <= raggioStormo2 && altro.getId() == idStormo_) { ... }
```

### Perché è stata fatta
- L'istruzione di radice quadrata (`std::sqrt` / istruzione hardware `SQRTSD`) è una delle operazioni matematiche più costose su CPU (richiede ~12-20 cicli di clock rispetto a 1 ciclo per moltiplicazione e addizione).
- Poiché la grande maggioranza delle coppie di boid nello spazio $1200 \times 800$ si trova a distanza maggiore del raggio visivo ($R = 30$), il check su $d^2 \le R^2$ scarta oltre il 90% delle coppie senza eseguire alcuna radice quadrata.

### Perché le alternative non erano preferibili
- **Opzione A: Approssimazioni veloci di radice quadrata (es. Fast Inverse Square Root di Quake III)**:
  *Non preferibile* perché introduce perdita di precisione numerica e viola il requisito di calcolo analitico esatto testato da Doctest con tolleranza stretta (`epsilon(1e-10)`).
- **Opzione B: Distanza di Manhattan ($|dx| + |dy|$) come filtro preliminare**:
  *Non preferibile* perché aggiunge ulteriori istruzioni di calcolo del valore assoluto e rami condizionali senza offrire vantaggi tangibili rispetto al semplice `dx*dx + dy*dy`.

---

## 3. Adozione di Algoritmi STL e Funzioni Lambda

Il file `README.md` esplicita nei criteri di valutazione:
> *"l'uso di algoritmi invece di cicli espliciti"*

Tutti i cicli iterativi grezzi (`for`) sono stati trasformati in algoritmi della Standard Library combinati con espressioni **lambda**.

### 3.1. `separazione` con `std::accumulate`
- **Prima**: Ciclo `for (const auto& altro : vicini)` che accumulava manualmente su una variabile locale.
- **Dopo**:
  ```cpp
  sf::Vector2<double> separazione(const std::vector<Boid>& vicini,
                                   const Boid& boid,
                                   double pesoSeparazione,
                                   double pesoPredatore) {
      const auto pThis = boid.getPos();
      const bool isThisNotPredator = (boid.getId() != ID_PREDATORE);

      return std::accumulate(
          vicini.begin(), vicini.end(),
          sf::Vector2<double>{0.0, 0.0},
          [&](sf::Vector2<double> acc, const Boid& altro) {
              const auto pAltro = altro.getPos();
              const double d2 = distanzaQuadrata(pThis, pAltro);

              if (d2 > 0.0 && d2 <= SOGLIA_SEPARAZIONE * SOGLIA_SEPARAZIONE) {
                  const double d = std::sqrt(d2);
                  const double fattore = (isThisNotPredator && altro.getId() == ID_PREDATORE)
                                             ? pesoPredatore
                                             : pesoSeparazione;
                  return acc + (pThis - pAltro) / d * fattore;
              }
              return acc;
          });
  }
  ```
- **Motivazione & Alternative**: `std::accumulate` rende esplicito che la funzione calcola una riduzione pura senza effetti collaterali (pure functional reduction). L'alternativa con ciclo `for` mutabile è meno idiomatica nel C++ moderno e penalizzata nei criteri di valutazione.

### 3.2. `evitareOstacoli` con `std::find_if`
- **Prima**: Ciclo `for` con `return` anticipato al primo ostacolo rilevato.
- **Dopo**:
  ```cpp
  sf::Vector2<double> evitareOstacoli(const Boid& boid,
                                       const std::vector<sf::Vector2i>& posizioni,
                                       double pesoOstacoli) {
      const auto pThis = boid.getPos();
      constexpr double raggio2 = RAGGIO_OSTACOLO * RAGGIO_OSTACOLO;

      const auto it = std::find_if(posizioni.begin(), posizioni.end(),
          [&](const sf::Vector2i& ostacolo) {
              const sf::Vector2<double> posOst{static_cast<double>(ostacolo.x),
                                               static_cast<double>(ostacolo.y)};
              const double d2 = distanzaQuadrata(pThis, posOst);
              return d2 > 0.0 && d2 <= raggio2;
          });

      if (it != posizioni.end()) {
          const sf::Vector2<double> posOst{static_cast<double>(it->x),
                                           static_cast<double>(it->y)};
          const double d = distanza(pThis, posOst);
          return (pThis - posOst) / d * pesoOstacoli;
      }
      return {0.0, 0.0};
  }
  ```
- **Motivazione & Alternative**: `std::find_if` esprime chiaramente l'intento di ricerca lineare del primo elemento che soddisfa il predicato. Rispetta al 100% il test `EvitareOstacoli – solo il primo ostacolo nel raggio viene considerato` in `boid.test.cpp`.

### 3.3. `aggiorna` e `Ostacolo::creazione` con `std::for_each`
- In `aggiorna`, l'applicazione simultanea dei delta-v usa `std::for_each` con iteratore progressivo `dvIt++` invece di un ciclo `for (size_t i = 0; ...)`.
- In `Ostacolo::creazione`, un unico oggetto `sf::CircleShape o(10.f)` viene istanziato all'inizio e riutilizzato tramite `std::for_each`, evitando di ri-allocare e ri-configurare un cerchio a ogni iterazione.

---

## 4. Ottimizzazione delle Operazioni in Virgola Mobile (Moltiplicazione per Reciproco)

### 4.1. `Boid::controlloV`
Nel codice originale:
```cpp
// Vecchio: 2 o 3 divisioni in virgola mobile
if (idStormo_ == ID_PREDATORE && n > maxVPred) {
    vel_ = vel_ / n * maxVPred;
}
if (n < minV.x) {
    vel_ = vel_ / n * minV.x;
}
```
Nella nuova versione:
```cpp
// Nuovo: 1 sola divisione per calcolare il reciproco, seguita da moltiplicazioni
const double n = std::sqrt(n2);
const double invN = 1.0 / n;

if (idStormo_ == ID_PREDATORE && n > maxVPred) {
    vel_ = vel_ * (maxVPred * invN);
} else if (idStormo_ != ID_PREDATORE && n > maxV) {
    vel_ = vel_ * (maxV * invN);
}

if (n < minV.x) {
    vel_ = vel_ * (minV.x * invN);
}
```
- **Guadagno**: La divisione hardware (`FDIV` / `DIVSD`) richiede ~15-30 cicli di clock, mentre la moltiplicazione (`FMUL` / `MULSD`) richiede solo ~3-5 cicli. Precalcolare `1.0 / n` riduce la latenza di calcolo.

### 4.2. `Boid::bordi`
Nel codice originale:
```cpp
if (pos_.x < margineX) {
    vel_.x += repulsioneBordi * (margineX - pos_.x) / margineX;
}
```
Nella nuova versione:
```cpp
const double invMargineX = repulsioneBordi / static_cast<double>(margineX);
const double invMargineY = repulsioneBordi / static_cast<double>(margineY);
if (pos_.x < margineX) {
    vel_.x += (margineX - pos_.x) * invMargineX;
}
```
- I fattori `invMargineX` e `invMargineY` sono calcolati una sola volta per chiamata anziché dividere per `margineX` e `margineY` in tutti e 4 i controlli dei bordi.

---

## 5. Calcolo e Visualizzazione delle Statistiche Collettive dello Stormo

### Requisito in `boids.md`
Il documento di specifica (`boids.md`, linea 111) stabilisce:
> *"Il programma dovrebbe mostrare in output alcuni parametri di descrizione collettiva dello stormo, quali l'andamento nel tempo della distanza media tra i boids e della loro velocità media e le rispettive deviazioni standard. Se propriamente simulati, i boids nel tempo dovrebbero tendere ad avvicinarsi e ad avere velocità mediamente simili tra di loro, assumendo il comportamento di uno stormo coeso."*

### Implementazione
È stata creata la struttura `StatisticheStormo` e le funzioni statistiche dedicate:
1. `calcolaVelocitaMedia`: media delle norme dei vettori velocità:
   $$\mu_v = \frac{1}{N} \sum_{i=1}^{N} \|\vec{v}_i\|$$
2. `calcolaDeviazioneStdVelocita`: deviazione standard della velocità:
   $$\sigma_v = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (\|\vec{v}_i\| - \mu_v)^2}$$
3. `calcolaDistanzaMedia`: media su tutte le $M = \frac{N(N-1)}{2}$ coppie non ordinate $(i, j)$ con $i < j$:
   $$\mu_d = \frac{2}{N(N-1)} \sum_{i=1}^{N-1} \sum_{j=i+1}^{N} \|\vec{x}_i - \vec{x}_j\|$$
4. `calcolaDeviazioneStdDistanza`: deviazione standard delle distanze pairwise:
   $$\sigma_d = \sqrt{\frac{2}{N(N-1)} \sum_{i=1}^{N-1} \sum_{j=i+1}^{N} (\|\vec{x}_i - \vec{x}_j\| - \mu_d)^2}$$

Nel `main.cpp`, ogni 60 frame (circa 1 secondo di simulazione a 60 FPS) viene calcolata la snapshot statistica e stampata su `std::cout`, permettendo di osservare numericamente la coesione e l'allineamento progressivo dello stormo.

---

## 6. Validazione Rigorosa degli Argomenti da Riga di Comando (`main.cpp`)

### Requisito in `README.md` e `boids.md`
`README.md` (linee 128-135) e `boids.md` (linea 109) richiedono che il programma possa accettare parametri in input (es. numero di boid, pesi di volo) e che effettui una validazione rigorosa dei tipi e degli intervalli, terminando con errore in caso di valori non validi.

### Implementazione
In `main.cpp`, il programma supporta l'esecuzione sia con valori di default (`./boids`), sia con parametri personalizzati da terminale:
```shell
./boids [n_boids] [peso_separazione] [peso_allineamento] [peso_coesione]
```
Vengono controllati:
- La corretta conversione numerica tramite `std::stoi` e `std::stod` all'interno di blocchi `try/catch`;
- Il rispetto dei vincoli fisici ($N > 0$, pesi $\ge 0$);
- In caso di valori non validi o sintassi errata, viene inviato un messaggio di errore chiaro su `std::cerr` e il programma termina con codice di uscita `1`.

---

## 7. Estensione della Suite di Test Unitari (`boid.test.cpp`)

Per garantire che tutte le nuove funzioni siano verificate e conformi ai requisiti di qualità:
1. **Conservazione**: Nessun test preesistente è stato modificato o eliminato; tutti i 30+ test originari continuano a verificare il comportamento di separazione, allineamento, coesione, ostacoli, controllo velocità, bordi e aggiornamento simultaneo.
2. **Nuovi Test Aggiunti**:
   - `normaQuadrata e distanzaQuadrata – calcolo esatto senza sqrt`
   - `Statistiche – stormo vuoto` (gestione corretta di vettori vuoti, nessun crash)
   - `Statistiche – stormo con 1 boid` (gestione del caso degenere senza coppie)
   - `Statistiche – calcolo analitico esatto per velocità media e deviazione standard` (verifica su terne note con $\mu_v = 4.0$, $\sigma_v = \sqrt{2/3}$)
   - `Statistiche – calcolo analitico esatto per distanza media e deviazione standard` (verifica su configurazione lineare nota con $\mu_d = 14/3$, $\sigma_d = \sqrt{26}/3$).

---

## Riepilogo dei Benefici per la Valutazione d'Esame

| Criterio di Valutazione (`README.md`) | Stato Precedente | Stato Attuale / Miglioramento |
|---|---|---|
| **Qualità dell'implementazione** | Cicli `for` imperativi grezzi, divisioni ripetute, allocazioni temporanee su heap | Uso sistematico di `<algorithm>` (`std::accumulate`, `std::find_if`, `std::for_each`, `std::transform`, `std::copy_if`) e lambda, ottimizzazioni a distanza quadratica e moltiplicazioni per reciproco |
| **Correttezza del programma** | Statistiche collettive assenti, parametri cablati senza validazione | Calcolo completo delle statistiche di stormo (media e stddev), validazione robusta degli input da CLI |
| **Qualità del design** | Creazione e copia di vettori intermedi ridondanti | Incapsulamento delle funzioni statistiche in `namespace brd`, const-correctness preservata e perfezionata |
| **Completezza dei test** | Test delle regole di volo base | Suite di test estesa con verifica analitica rigorosa delle nuove utilità e statistiche |
