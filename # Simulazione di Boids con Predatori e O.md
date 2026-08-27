# Simulazione di Boids con Predatori e Ostacoli

**Relazione del progetto**

## Indice

1. [Introduzione](#1-introduzione)
2. [Il Modello dei Boids](#2-il-modello-dei-boids)
3. [Estensione con Predatori e Ostacoli](#3-estensione-con-predatori-e-ostacoli)
4. [Descrizione del Codice](#4-descrizione-del-codice)
5. [Scelte di Design](#5-scelte-di-design)
6. [Riferimenti](#6-riferimenti)

---

## 1. Introduzione

La simulazione di comportamenti di gruppo in animali come stormi di uccelli, branchi di pesci o sciami di insetti è stata studiata per molti anni. Uno degli algoritmi più noti per simulare questi comportamenti è l'algoritmo dei Boids, introdotto da Craig Reynolds nel 1986 [1]. Questo algoritmo è basato su tre regole principali: coesione, allineamento e separazione, che governano il comportamento di ogni singolo boid all'interno di un gruppo.

In questa relazione viene descritto un programma che implementa tale modello in C++, con il supporto della libreria grafica SFML, estendendolo con la presenza di predatori e di ostacoli disegnati interattivamente dall'utente. L'obiettivo è accompagnare il lettore nella comprensione del programma, illustrando le scelte di design, le decisioni di implementazione e la strategia seguita per verificarne la correttezza.

## 2. Il Modello dei Boids

Il modello dei Boids è un esempio di simulazione basata su agenti, in cui ogni boid agisce autonomamente in base a semplici regole locali:

- **Coesione**: ogni boid si muove verso il centro di massa dei boid vicini, cercando di mantenere il gruppo unito.
- **Allineamento**: ogni boid cerca di allineare la propria direzione con quella dei boid vicini.
- **Separazione**: ogni boid cerca di evitare la collisione con i boid vicini, mantenendo una distanza di sicurezza.

Queste regole permettono di simulare comportamenti complessi a livello di gruppo, emergenti dall'interazione di regole semplici a livello individuale.

Ogni boid è descritto da una posizione `pos` e da una velocità `vel`, entrambe vettori bidimensionali. A ogni passo della simulazione ciascuna regola produce un contributo di steering `Δv`; i contributi vengono sommati e applicati secondo lo schema:

```
Δv = Δv_coesione + Δv_allineamento + Δv_separazione + Δv_ostacoli
vel ← vel + Δv
pos ← pos + vel
```

Ogni contributo è moltiplicato per un peso configurabile, che consente di bilanciare la compattezza dello stormo, l'uniformità del moto, la prevenzione delle collisioni e la capacità di evitare gli ostacoli.

## 3. Estensione con Predatori e Ostacoli

Nel progetto descritto in questa relazione, il modello base dei Boids è stato esteso per includere la presenza di predatori e ostacoli. I predatori si muovono secondo regole leggermente diverse, cercando di avvicinarsi alle prede (i boid normali) per inseguirle e disperderne lo stormo. Gli ostacoli, d'altra parte, costringono i boid a modificare la propria traiettoria per evitarli.

La simulazione supporta inoltre più stormi distinti, identificati da un id numerico, e presenta un riquadro informativo con statistiche aggiornate in tempo reale.

## 4. Descrizione del Codice

### 4.1 Contenuto del Progetto

Per creare questa simulazione è stato utilizzato CMake per la creazione dei Makefile. Il progetto consiste di 5 file totali più la cartella contenente la libreria SFML:

- **`PredatorBoids.hpp`**: file header contenente la dichiarazione di tutte le funzioni e delle classi usate nel progetto. In questo file vengono definiti il namespace `brd` e le costanti fisiche della simulazione.
- **`PredatorBoids.cpp`**: contiene le definizioni delle funzioni dichiarate nell'header, comprese le quattro regole di interazione, i metodi della classe `Boid` e le funzioni statistiche.
- **`boid.test.cpp`**: qui si trovano i `TEST_CASE` usati per controllare il corretto funzionamento del codice e delle regole di interazione.
- **`main.cpp`**: qui troviamo il ciclo principale che permette alla simulazione di funzionare. Sempre in questo file sono definiti tutti i pesi per le varie funzioni, il numero di boids da disegnare, la dimensione della finestra e le altre parti necessarie al corretto funzionamento della simulazione.
- **`CMakeLists.txt`**: permette a CMake di individuare i file e le librerie da linkare (come SFML e le sue sottolibrerie) correttamente per creare i nostri eseguibili, il test e l'eseguibile per la parte grafica della simulazione.

### 4.2 Namespace, costanti e funzioni ausiliarie

Tutto il codice della simulazione è racchiuso nel namespace `brd`, in modo da evitare conflitti di nomi. Nell'header sono definite come `inline constexpr` le costanti che determinano la fisica della simulazione:

| Costante | Valore | Significato |
|---|---|---|
| `ID_PREDATORE` | 999 | Valore convenzionale dell'id che identifica un predatore |
| `RAGGIO_VISTA_STORMO` | 30.0 | Raggio entro cui un boid percepisce i compagni del proprio stormo |
| `RAGGIO_VISTA_ALTRO` | 50.0 | Raggio entro cui un boid percepisce boid di altri stormi e predatori |
| `RAGGIO_VISTA_PRED` | 40.0 | Raggio di vista dei predatori per individuare le prede |
| `SOGLIA_SEPARAZIONE` | 25.0 | Distanza al di sotto della quale scatta la repulsione |
| `RAGGIO_OSTACOLO` | 50.0 | Raggio di influenza di ciascun ostacolo |

Accanto alle costanti sono presenti le funzioni ausiliarie `normaQuadrata`, `distanzaQuadrata`, `norma` e `distanza`. Le prime due calcolano le quantità al quadrato senza chiamare `std::sqrt`: tutti i confronti con i raggi di vista usano le distanze quadratiche, producendo lo stesso risultato logico ma evitando una radice quadrata per ogni coppia di boid a ogni fotogramma. Le versioni "con radice" sono usate solo dove serve il valore effettivo, per esempio nella normalizzazione dei contributi repulsivi.

### 4.3 Classe Boid

La classe `Boid` rappresenta un singolo boid. I dati sono incapsulati: gli attributi sono privati e lo stato può essere modificato unicamente tramite le operazioni previste. Di seguito una descrizione dettagliata degli attributi e dei metodi principali:

- `sf::Vector2<double> pos_`: rappresenta la posizione del boid nello spazio 2D.
- `sf::Vector2<double> vel_`: rappresenta la velocità del boid.
- `int idStormo_`: identifica a quale stormo appartiene il boid. Se è uguale a `ID_PREDATORE`, il boid è considerato un predatore.
- `Boid(p, v, id)`: costruttore della classe, con parametri di default che inizializzano posizione e velocità nulle e id pari a 0.
- `getPos()`, `getVel()`, `getId()`: metodi di accesso in sola lettura allo stato del boid.
- `calcolaDeltaV(...)`: calcola la variazione complessiva di velocità del boid in base a coesione, allineamento, separazione, evasione dagli ostacoli e presenza di predatori. Il metodo è `const`: non modifica il boid, ma restituisce il vettore `Δv`.
- `applica(deltaV)`: applica la variazione calcolata, aggiornando prima la velocità e poi la posizione.
- `controlloV(maxV, minV, maxVPred)`: limita la velocità al valore massimo `maxV`, oppure a `maxVPred` per i predatori, preservandone la direzione; se la velocità scende sotto il minimo fissato tramite `minV`, viene riportata a tale valore. Se la norma è numericamente nulla il metodo non interviene, per evitare divisioni per zero.
- `bordiToroidali(lunghezzaFinestra, altezzaFinestra)`: implementa i bordi toroidali: un boid che esce da un lato della finestra rientra dal lato opposto.
- `disegno(window, c)`: disegna il boid nella finestra, colorandolo di rosso se è un predatore, altrimenti con un colore diverso per ciascuno stormo (blu, bianco o verde); i predatori hanno un raggio maggiore per essere riconoscibili.

### 4.4 Le regole di interazione

Le tre regole fondamentali e l'evasione dagli ostacoli sono implementate come funzioni libere nel namespace `brd`. Ciascuna riceve i dati necessari e restituisce il vettore di steering calcolato, senza effetti collaterali: questa scelta rende le funzioni più facili da ragionare, da comporre e da testare singolarmente.

- `coesione(vicini, boid, pesoCoesione)`: calcola il centro di massa dei vicini (mediante `std::accumulate`) e restituisce il vettore che punta dal boid verso tale centro, scalato dal peso. Se la lista dei vicini è vuota il contributo è nullo.
- `allineamento(vicini, boid, pesoAllineamento)`: restituisce la differenza tra la velocità media dei vicini e la velocità del boid, scalata dal peso; anche qui una lista vuota produce un contributo nullo.
- `separazione(vicini, boid, pesoSeparazione, pesoPredatore)`: somma i contributi repulsivi dei vicini entro la soglia di separazione. ogni contributo ha direzione opposta al vicino e modulo costante pari al peso; la divisione per la distanza serve unicamente a estrarre il versore direzionale. Se il vicino è un predatore e il boid corrente non lo è, viene usato `pesoPredatore`, modellando la fuga come una reazione più intensa. I boid esattamente sulla soglia sono inclusi; il caso di due boid nella stessa posizione è escluso per evitare divisioni per zero.
- `evitareOstacoli(boid, posizioni, pesoOstacoli)`: somma i contributi repulsivi degli ostacoli entro il raggio di influenza, con la stessa legge di scala. Se un boid si trova esattamente sulla posizione di un ostacolo, dove la direzione di fuga non è definita, viene applicata una spinta deterministica verso l'alto di modulo pari al peso.

### 4.5 Comportamento dei Predatori

Quando un boid ha un `idStormo_` uguale a `ID_PREDATORE`, viene considerato un predatore e segue una logica diversa, implementata nel ramo dedicato di `calcolaDeltaV`:

- I predatori individuano le prede entro il raggio `RAGGIO_VISTA_PRED`, selezionandole con `std::copy_if`.
- Una volta individuate le prede, il predatore applica coesione e allineamento verso di esse, dirigendosi verso il loro centro di massa e adeguandosi alla loro velocità; applica inoltre la separazione per non sovrapporsi.
- I predatori hanno una velocità massima propria (`MAX_V_PRED`), limitata da `controlloV`, distinta da quella dei boid normali: ciò permette di regolare il bilanciamento tra cacciatori e prede senza modificare il resto della simulazione.
- Anche i predatori evitano gli ostacoli, poiché il contributo di `evitareOstacoli` viene sommato a ogni boid indipendentemente dal suo id.

I boid normali applicano coesione e allineamento soltanto verso i compagni del proprio stormo (entro `RAGGIO_VISTA_STORMO`), mentre la separazione è calcolata anche rispetto a boid di stormi diversi e ai predatori (entro `RAGGIO_VISTA_ALTRO`): in questo modo stormi differenti si attraversano senza fondersi, e la presenza di un predatore provoca la dispersione dello stormo.

### 4.6 Aggiornamento simultaneo dello stormo

La funzione libera `aggiorna` coordina l'evoluzione di tutto lo stormo in un singolo fotogramma, seguendo uno schema in due fasi: prima calcola il vettore `Δv` di ogni boid tramite `calcolaDeltaV` (con `std::transform`), leggendo lo stato dello stormo com'è all'inizio del fotogramma, poi applica a ciascun boid il proprio `Δv` tramite `applica`. Questa organizzazione garantisce un aggiornamento *simultaneo*: le decisioni di tutti i boid si basano sulla stessa configurazione e il risultato non dipende dall'ordine dei boid nel contenitore. Se ogni boid fosse aggiornato subito dopo il calcolo del proprio `Δv`, i boid successivi osserverebbero una configurazione già modificata, introducendo un bias legato all'ordinamento del vettore.

### 4.7 Ostacoli e statistiche

Gli ostacoli sono punti dello schermo scelti interattivamente dall'utente: le posizioni sono raccolte in un vettore nel `main` e passate alla simulazione a ogni fotogramma; la funzione `disegnaOstacoli` li rappresenta come cerchi arancioni centrati sul punto del click. Essendo entità puramente passive e prive di stato oltre alla posizione, non richiedono una classe dedicata: un vettore di coordinate è sufficiente e semplifica il codice.

Per osservare quantitativamente il comportamento della simulazione, la struttura `StatisticheStormo` raccoglie velocità media e distanza media tra tutte le coppie di boid, con le rispettive deviazioni standard. Le funzioni di calcolo gestiscono esplicitamente i casi degeneri di stormo vuoto o con un solo elemento, restituendo 0 invece di dividere per zero.

### 4.8 Esecuzione del Codice

Il file `main.cpp` gestisce l'inizializzazione della finestra grafica e la creazione dei boid. I parametri sono raccolti come costanti in testa al file, in modo da poter essere modificati senza cercare nel corpo del programma:

| Parametro | Valore | Significato |
|---|---|---|
| `LUNGHEZZA_FINESTRA` × `ALTEZZA_FINESTRA` | 1200 × 800 | Dimensioni in pixel della finestra |
| `DEFAULT_N_BOIDS` | 200 | Numero di boid generati |
| `N_STORMI` | 3 | Numero di stormi distinti |
| `FREQ_PREDATORI` | 10 | Un boid ogni 10 è un predatore |
| `MAX_VELOCITY` / `MIN_VELOCITY` | 2.0 / 0.2 | Limiti di velocità dei boid normali |
| `MAX_V_PRED` | 0.4 | Velocità massima dei predatori |
| `DEFAULT_PESO_COESIONE` | 0.005 | Peso della coesione |
| `DEFAULT_PESO_ALLINEAMENTO` | 0.5 | Peso dell'allineamento |
| `DEFAULT_PESO_SEPARAZIONE` | 0.7 | Peso della separazione |
| `PESO_PREDATORE` | 1.0 | Peso della fuga dai predatori |
| `PESO_OSTACOLI` | 0.09 | Peso dell'evasione dagli ostacoli |

Il ciclo principale di esecuzione comprende:

1. La gestione degli eventi: la chiusura della finestra termina il programma; il click sinistro aggiunge un ostacolo nella posizione del mouse; il click destro rimuove l'ultimo ostacolo inserito.
2. L'aggiornamento di posizioni e velocità di tutti i boid tramite `aggiorna`.
3. Il controllo dei bordi toroidali e dei limiti di velocità per ciascun boid.
4. Il disegno degli ostacoli, dei boid e del riquadro con le statistiche.

Le posizioni e le velocità iniziali vengono generate in maniera pseudocasuale attraverso la libreria `<random>`, usando il generatore `std::mt19937` accoppiato al seme fornito da `std::random_device`; le distribuzioni uniformi permettono di scegliere gli intervalli di generazione garantendo che ogni valore abbia la stessa probabilità di essere generato. Il riquadro informativo mostra in tempo reale numero di boid, medie e deviazioni standard, numero di ostacoli e comandi disponibili; le stesse statistiche sono stampate periodicamente sul terminale. Si consiglia di adattare velocità, numero di boids e pesi alla dimensione della finestra scelta.

## 5. Scelte di Design

- **Separazione tra interfaccia e implementazione**: l'header dichiara l'interfaccia pubblica del modulo, il file `.cpp` l'implementazione; chi legge può farsi un'idea completa delle funzionalità dal solo header.
- **Funzioni pure per le regole**: le regole non modificano il boid ma restituiscono il vettore di steering, eliminando effetti collaterali e consentendo di testare ciascuna regola isolatamente.
- **Aggiornamento in due fasi**: calcolare prima tutti i `Δv` e applicarli solo dopo garantisce la simultaneità dell'aggiornamento e l'indipendenza dall'ordine dei boid.
- **Incapsulamento dello stato**: i dati del boid sono privati e modificabili solo tramite `applica`, `controlloV` e `bordiToroidali`, le uniche trasformazioni lecite dello stato.
- **Confronti su quantità quadratiche**: tutti i test di prossimità usano `distanzaQuadrata`, evitando radici quadrate nel ciclo più interno della simulazione.
- **Uso degli algoritmi standard**: `std::accumulate`, `std::transform`, `std::copy_if` e `std::for_each` esprimono direttamente l'intenzione delle operazioni e riducono la probabilità di errori rispetto a cicli scritti a mano.
- **Costanti `inline constexpr`**: i parametri fisici sono costanti di compilazione raccolte in un unico punto; il valore speciale 999 compare solo come `ID_PREDATORE`, mai come numero "magico".
- **Rappresentazione unificata**: predatori e boid normali sono istanze della stessa classe e condividono il ciclo di aggiornamento; la diversità di comportamento è localizzata in `calcolaDeltaV` e `controlloV`.
- **Una sola forma grafica riutilizzata**: `disegno` riceve un unico `sf::CircleShape` creato nel `main` e ne modifica colore, raggio e posizione per ciascun boid, evitando di costruire centinaia di oggetti grafici a ogni fotogramma.


## 7. Riferimenti

[1] Reynolds, Craig W. *Flocks, herds and schools: A distributed behavioral model.* ACM SIGGRAPH Computer Graphics. Vol. 21. No. 4. 1987.