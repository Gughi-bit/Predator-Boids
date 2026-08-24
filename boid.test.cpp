#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "PredatorBoids.hpp"

// ────────────────────────────────────────────────────────────────────────────
// API usata nei test:
//   getPos() / getVel() / getId()
//   funzioni libere: coesione(), allineamento(), separazione(),
//                    evitareOstacoli(), norma(), distanza()
//   metodi: applica(), controlloV(), bordi()
//   funzione libera: aggiorna()
// ────────────────────────────────────────────────────────────────────────────

namespace brd {

// ════════════════════════════════════════════════════════════════════════════
// SEPARAZIONE
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Separazione – stesso stormo, distanza entro soglia") {
    // diff = (-10,-10), d = 10√2 ≈ 14.14 < SOGLIA_SEPARAZIONE(25)
    // dv = (-10,-10) / (10√2) * pSep = (-1/√2, -1/√2)
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 2);
    Boid b2({110.0, 110.0}, {0.0, 0.0}, 2);

    auto dv = separazione({b2}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(-1.0 / std::sqrt(2.0)).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(-1.0 / std::sqrt(2.0)).epsilon(1e-5));
}

TEST_CASE("Separazione – fuga dal predatore, peso amplificato") {
    // preda in (510,510), predatore in (500,500)
    // diff = (10,10), d = 10√2, fattore = pPred = 2
    // dv = (10,10) / (10√2) * 2 = (2/√2, 2/√2) = (√2, √2)
    Boid predatore({500.0, 500.0}, {0.0, 0.0}, ID_PREDATORE);
    Boid preda    ({510.0, 510.0}, {0.0, 0.0}, 1);

    auto dv = separazione({predatore}, preda, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(2.0 / std::sqrt(2.0)).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(2.0 / std::sqrt(2.0)).epsilon(1e-5));
}

TEST_CASE("Separazione – distanza esattamente uguale a SOGLIA_SEPARAZIONE, nessuna repulsione") {
    // d = SOGLIA_SEPARAZIONE esatto → condizione d <= 25 NON soddisfatta (d == 25 lo è)
    // Verifichiamo il caso al limite: d = SOGLIA_SEPARAZIONE
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({SOGLIA_SEPARAZIONE, 0.0}, {0.0, 0.0}, 0);  // d == 25 esatto

    auto dv = separazione({b2}, b1, 1.0, 2.0);

    // d <= SOGLIA_SEPARAZIONE → repulsione attesa sulla componente x
    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));  // diff.x = -25, d = 25 → -25/25 * 1 = -1
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – distanza appena oltre la soglia, dv nullo") {
    Boid b1({0.0, 0.0},   {0.0, 0.0}, 0);
    Boid b2({100.0, 0.0}, {0.0, 0.0}, 0);  // d = 100 >> SOGLIA_SEPARAZIONE

    auto dv = separazione({b2}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – lista vicini vuota, dv nullo") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);
    auto dv = separazione({}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – più vicini, contributi si sommano") {
    // b1 in (0,0); b2 a destra (10,0), b3 a sinistra (-10,0)
    // contributo b2: diff = (-10,0)/10 * 1 = (-1, 0)
    // contributo b3: diff = (10,0)/10  * 1 = ( 1, 0)
    // somma x = 0 per simmetria
    Boid b1({  0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({ 10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 0.0}, 0);

    auto dv = separazione({b2, b3}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – predatore non fugge da boid normali") {
    // Il predatore (999) si separa con pesoSeparazione, non pesoPredatore
    Boid pred ({0.0, 0.0},  {0.0, 0.0}, ID_PREDATORE);
    Boid preda({10.0, 0.0}, {0.0, 0.0}, 1);

    // pSep = 1, pPred = 5: il predatore usa sempre pSep verso i normali
    auto dv = separazione({preda}, pred, 1.0, 5.0);

    // diff = (-10, 0) / 10 * 1.0 = (-1.0, 0)
    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – peso zero, dv nullo anche entro soglia") {
    Boid b1({0.0, 0.0},  {0.0, 0.0}, 0);
    Boid b2({5.0, 0.0},  {0.0, 0.0}, 0);

    auto dv = separazione({b2}, b1, 0.0, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// ALLINEAMENTO
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Allineamento – un vicino, steering verso la sua velocità") {
    // velMedia = (0,1), velB1 = (1,0) → dv = (0-1, 1-0)*1 = (-1, 1)
    Boid b1({300.0, 300.0}, {1.0, 0.0}, 0);
    Boid b2({320.0, 320.0}, {0.0, 1.0}, 0);

    auto dv = allineamento({b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 1.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – lista vicini vuota, dv nullo") {
    Boid b1({300.0, 300.0}, {1.0, 0.5}, 0);
    auto dv = allineamento({}, b1, 5.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – velocità già allineata ai vicini, dv nullo") {
    // Tutti si muovono nella stessa direzione → nessuna correzione
    Boid b1({0.0, 0.0},   {1.0, 0.0}, 0);
    Boid b2({10.0, 0.0},  {1.0, 0.0}, 0);
    Boid b3({20.0, 0.0},  {1.0, 0.0}, 0);

    auto dv = allineamento({b2, b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – più vicini, media ponderata uniforme") {
    // b2 ha vel (2,0), b3 ha vel (0,2) → velMedia = (1,1)
    // b1 ha vel (0,0) → dv = (1,1) * peso
    Boid b1({0.0,  0.0},  {0.0, 0.0}, 0);
    Boid b2({10.0, 0.0},  {2.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 2.0}, 0);

    auto dv = allineamento({b2, b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – peso zero, dv nullo") {
    Boid b1({0.0, 0.0},  {0.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {5.0, 3.0}, 0);

    auto dv = allineamento({b2}, b1, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – velocità opposte, steering doppio") {
    // b1 ha vel (1,0), b2 ha vel (-1,0)
    // velMedia = (-1,0), dv = (-1-1, 0-0)*1 = (-2, 0)
    Boid b1({0.0,  0.0}, { 1.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {-1.0, 0.0}, 0);

    auto dv = allineamento({b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(-2.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// COESIONE
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Coesione – due vicini, centro di massa corretto") {
    // centroMassa = ((250+300)/2, (250+300)/2) = (275, 275)
    // dv = (275-200, 275-200) * 1 = (75, 75)
    Boid b1({200.0, 200.0}, {0.0, 0.0}, 0);
    Boid b2({250.0, 250.0}, {0.0, 0.0}, 0);
    Boid b3({300.0, 300.0}, {0.0, 0.0}, 0);

    auto dv = coesione({b2, b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(75.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(75.0).epsilon(1e-10));
}

TEST_CASE("Coesione – lista vicini vuota, dv nullo") {
    Boid b1({200.0, 200.0}, {0.0, 0.0}, 0);
    auto dv = coesione({}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – boid già al centro di massa, dv nullo") {
    // b1 è già il centroide di {b2, b3}
    Boid b1({0.0,  0.0}, {0.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0},{0.0, 0.0}, 0);
    // centroMassa dei vicini = (10 + (-10))/2 = 0 → stesso di b1

    auto dv = coesione({b2, b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – un solo vicino, steering diretto verso di lui") {
    // centroMassa = posizione di b2 = (50, 0)
    // dv = (50-0, 0-0) * 1 = (50, 0)
    Boid b1({0.0,  0.0}, {0.0, 0.0}, 0);
    Boid b2({50.0, 0.0}, {0.0, 0.0}, 0);

    auto dv = coesione({b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(50.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – peso zero, dv nullo") {
    Boid b1({0.0,   0.0}, {0.0, 0.0}, 0);
    Boid b2({100.0, 0.0}, {0.0, 0.0}, 0);

    auto dv = coesione({b2}, b1, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – molti vicini distribuiti simmetricamente, dv nullo") {
    // 4 vicini simmetrici rispetto a b1 → centro di massa coincide con b1
    Boid b1({0.0,  0.0},  {0.0, 0.0}, 0);
    Boid b2({10.0, 0.0},  {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b4({0.0,  10.0}, {0.0, 0.0}, 0);
    Boid b5({0.0, -10.0}, {0.0, 0.0}, 0);

    auto dv = coesione({b2, b3, b4, b5}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// EVITARE OSTACOLI
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("EvitareOstacoli – ostacolo entro raggio, repulsione corretta") {
    // diff = (100-102, 100-102) = (-2,-2), d = 2√2
    // dv = (-2,-2)/(2√2) * 1.5 = (-1.5/√2, -1.5/√2)
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(102, 102)};

    auto dv = evitareOstacoli(b1, ostacoli, 1.5);

    const double atteso = -1.5 / std::sqrt(2.0);
    CHECK(dv.x == doctest::Approx(atteso).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(atteso).epsilon(1e-5));
}

TEST_CASE("EvitareOstacoli – ostacolo esattamente sul bordo del raggio") {
    // Boid in (0,0), ostacolo in (RAGGIO_OSTACOLO, 0) → d == RAGGIO_OSTACOLO
    // Condizione d <= RAGGIO_OSTACOLO soddisfatta → repulsione attesa
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const int r = static_cast<int>(RAGGIO_OSTACOLO);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(r, 0)};

    auto dv = evitareOstacoli(b1, ostacoli, 1.0);

    // diff = (0 - r, 0) = (-r, 0), d = r → dv.x = -r/r * 1 = -1
    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-5));
}

TEST_CASE("EvitareOstacoli – ostacolo appena fuori raggio, dv nullo") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(500, 500)};

    auto dv = evitareOstacoli(b1, ostacoli, 1.5);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("EvitareOstacoli – lista ostacoli vuota, dv nullo") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);
    auto dv = evitareOstacoli(b1, {}, 1.5);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("EvitareOstacoli – solo il primo ostacolo nel raggio viene considerato") {
    // La funzione restituisce al primo match: due ostacoli vicini,
    // ma solo il primo dovrebbe contribuire
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    // primo ostacolo: a destra → spinge a sinistra (dv.x < 0)
    // secondo ostacolo: in basso → spinge in alto (dv.y > 0)
    // ci aspettiamo solo il contributo del primo
    const std::vector<sf::Vector2i> ostacoli = {
        sf::Vector2i(10, 0),
        sf::Vector2i(0, 10)
    };

    auto dv = evitareOstacoli(b1, ostacoli, 1.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));  // solo primo
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-5));  // secondo ignorato
}

TEST_CASE("EvitareOstacoli – peso zero, dv nullo anche a contatto") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(5, 0)};

    auto dv = evitareOstacoli(b1, ostacoli, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// CONTROLLO VELOCITÀ
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("ControlloV – velocità superiore al massimo, ridotta preservando direzione") {
    // vel = (20, 20), norma = 20√2 >> maxV=1
    // dopo il clamping: norma = 1, direzione (1/√2, 1/√2)
    Boid b1({0.0, 0.0}, {20.0, 20.0}, 0);
    const sf::Vector2<double> minV{0.08, 0.08};

    b1.controlloV(1.0, minV, 0.5);

    const double n = norma(b1.getVel());
    CHECK(n <= doctest::Approx(1.0).epsilon(1e-5));
    // La direzione deve essere preservata: vx == vy (simmetria)
    CHECK(b1.getVel().x == doctest::Approx(b1.getVel().y).epsilon(1e-10));
}

TEST_CASE("ControlloV – velocità sotto il minimo, alzata preservando direzione") {
    // vel = (0.001, 0) → norma = 0.001 < minV = 0.08
    // dopo: norma = 0.08, direzione (1,0)
    Boid b1({0.0, 0.0}, {0.001, 0.0}, 0);
    const sf::Vector2<double> minV{0.08, 0.08};

    b1.controlloV(1.0, minV, 0.5);

    const double n = norma(b1.getVel());
    CHECK(n >= doctest::Approx(0.08).epsilon(1e-5));
    // direzione mantenuta: componente y deve restare nulla
    CHECK(b1.getVel().y == doctest::Approx(0.0).epsilon(1e-8));
}

TEST_CASE("ControlloV – predatore rispetta maxVPred, non maxV") {
    // vel = (1,1), norma = √2 > maxVPred = 0.5
    Boid pred({0.0, 0.0}, {1.0, 1.0}, ID_PREDATORE);
    const sf::Vector2<double> minV{0.08, 0.08};

    pred.controlloV(1.0, minV, 0.5);

    CHECK(norma(pred.getVel()) <= doctest::Approx(0.5).epsilon(1e-5));
}

TEST_CASE("ControlloV – velocità già entro i limiti, nessuna modifica") {
    // vel = (0.3, 0.3), norma ≈ 0.424 → tra minV(0.08) e maxV(1.0)
    Boid b1({0.0, 0.0}, {0.3, 0.3}, 0);
    const sf::Vector2<double> minV{0.08, 0.08};

    b1.controlloV(1.0, minV, 0.5);

    const auto vel = b1.getVel();
    CHECK(vel.x == doctest::Approx(0.3).epsilon(1e-10));
    CHECK(vel.y == doctest::Approx(0.3).epsilon(1e-10));
}

TEST_CASE("ControlloV – velocità esattamente al massimo, non modificata") {
    // norma = 1.0 == maxV → nessun clamping
    const double v = 1.0 / std::sqrt(2.0);  // norma = 1
    Boid b1({0.0, 0.0}, {v, v}, 0);
    const sf::Vector2<double> minV{0.08, 0.08};

    b1.controlloV(1.0, minV, 0.5);

    CHECK(norma(b1.getVel()) <= doctest::Approx(1.0).epsilon(1e-8));
    CHECK(b1.getVel().x == doctest::Approx(v).epsilon(1e-8));
}

TEST_CASE("ControlloV – predatore con velocità entro maxVPred, non modificata") {
    Boid pred({0.0, 0.0}, {0.1, 0.0}, ID_PREDATORE);  // norma = 0.1 < maxVPred=0.5
    const sf::Vector2<double> minV{0.08, 0.08};

    pred.controlloV(1.0, minV, 0.5);

    CHECK(pred.getVel().x == doctest::Approx(0.1).epsilon(1e-10));
    CHECK(pred.getVel().y == doctest::Approx(0.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// BORDI
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Bordi – boid vicino al bordo sinistro, spinta verso destra") {
    // pos.x = 10 < margineX = 100 → vel.x deve aumentare
    Boid b1({10.0, 400.0}, {0.0, 0.0}, 0);
    const double velx_prima = b1.getVel().x;

    b1.bordi(1.5, 1200, 800, 100, 100);

    CHECK(b1.getVel().x > velx_prima);
}

TEST_CASE("Bordi – boid vicino al bordo destro, spinta verso sinistra") {
    // pos.x = 1150 > 1200-100 = 1100 → vel.x deve diminuire
    Boid b1({1150.0, 400.0}, {0.0, 0.0}, 0);
    const double velx_prima = b1.getVel().x;

    b1.bordi(1.5, 1200, 800, 100, 100);

    CHECK(b1.getVel().x < velx_prima);
}

TEST_CASE("Bordi – boid vicino al bordo superiore, spinta verso il basso") {
    // pos.y = 10 < margineY = 100 → vel.y deve aumentare
    Boid b1({600.0, 10.0}, {0.0, 0.0}, 0);
    const double vely_prima = b1.getVel().y;

    b1.bordi(1.5, 1200, 800, 100, 100);

    CHECK(b1.getVel().y > vely_prima);
}

TEST_CASE("Bordi – boid vicino al bordo inferiore, spinta verso l'alto") {
    // pos.y = 750 > 800-100 = 700 → vel.y deve diminuire
    Boid b1({600.0, 750.0}, {0.0, 0.0}, 0);
    const double vely_prima = b1.getVel().y;

    b1.bordi(1.5, 1200, 800, 100, 100);

    CHECK(b1.getVel().y < vely_prima);
}

TEST_CASE("Bordi – boid al centro, nessuna forza applicata") {
    Boid b1({600.0, 400.0}, {0.1, 0.1}, 0);

    b1.bordi(1.5, 1200, 800, 100, 100);

    // Nessun bordo attivo: velocità invariata
    CHECK(b1.getVel().x == doctest::Approx(0.1).epsilon(1e-10));
    CHECK(b1.getVel().y == doctest::Approx(0.1).epsilon(1e-10));
}

TEST_CASE("Bordi – repulsione più forte più si è vicini al bordo") {
    // Boid a distanza 5 dal bordo sinistro vs boid a distanza 50
    Boid vicino({5.0,  400.0}, {0.0, 0.0}, 0);
    Boid lontano({50.0, 400.0}, {0.0, 0.0}, 0);

    vicino.bordi(1.5, 1200, 800, 100, 100);
    lontano.bordi(1.5, 1200, 800, 100, 100);

    // Chi è più vicino deve ricevere una spinta maggiore
    CHECK(vicino.getVel().x > lontano.getVel().x);
}

// ════════════════════════════════════════════════════════════════════════════
// FUNZIONI MATEMATICHE
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("norma – vettore nullo") {
    CHECK(norma({0.0, 0.0}) == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("norma – vettore unitario") {
    CHECK(norma({1.0, 0.0}) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(norma({0.0, 1.0}) == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("norma – vettore (3,4) → norma = 5") {
    CHECK(norma({3.0, 4.0}) == doctest::Approx(5.0).epsilon(1e-10));
}

TEST_CASE("norma – componenti negative") {
    CHECK(norma({-3.0, -4.0}) == doctest::Approx(5.0).epsilon(1e-10));
}

TEST_CASE("distanza – punti identici, distanza zero") {
    const sf::Vector2<double> p{100.0, 200.0};
    CHECK(distanza(p, p) == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("distanza – asse x, distanza esatta") {
    const sf::Vector2<double> a{0.0, 0.0}, b{5.0, 0.0};
    CHECK(distanza(a, b) == doctest::Approx(5.0).epsilon(1e-10));
}

TEST_CASE("distanza – (0,0) → (3,4) = 5") {
    const sf::Vector2<double> a{0.0, 0.0}, b{3.0, 4.0};
    CHECK(distanza(a, b) == doctest::Approx(5.0).epsilon(1e-10));
}

TEST_CASE("distanza – simmetria: d(a,b) == d(b,a)") {
    const sf::Vector2<double> a{10.0, 20.0}, b{-5.0, 35.0};
    CHECK(distanza(a, b) == doctest::Approx(distanza(b, a)).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// AGGIORNAMENTO SIMULTANEO
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("aggiorna – la posizione cambia dopo un tick") {
    Boid b1({200.0, 200.0}, {0.1, 0.0}, 0);
    Boid b2({220.0, 200.0}, {0.1, 0.0}, 0);
    std::vector<Boid> stormo = {b1, b2};

    const double px_prima = stormo[0].getPos().x;
    aggiorna(stormo, 0.001, 0.1, 0.4, 1.0, 0.09, {});

    CHECK(stormo[0].getPos().x != doctest::Approx(px_prima).epsilon(1e-10));
}

TEST_CASE("aggiorna – stormo con un solo boid, si muove per inerzia") {
    // Nessun vicino → nessuna regola attiva → il boid avanza di vel
    Boid b1({200.0, 200.0}, {0.5, 0.3}, 0);
    std::vector<Boid> stormo = {b1};

    aggiorna(stormo, 0.001, 0.1, 0.4, 1.0, 0.09, {});

    // pos_new = pos + vel (nessun delta-v da vicini)
    CHECK(stormo[0].getPos().x == doctest::Approx(200.5).epsilon(1e-8));
    CHECK(stormo[0].getPos().y == doctest::Approx(200.3).epsilon(1e-8));
}

TEST_CASE("aggiorna – stormo vuoto, nessun crash") {
    std::vector<Boid> stormo;
    CHECK_NOTHROW(aggiorna(stormo, 0.001, 0.1, 0.4, 1.0, 0.09, {}));
}

TEST_CASE("aggiorna – simultaneità: posizioni pre-tick usate per tutti i calcoli") {
    // b1 e b2 si muovono in direzioni opposte partendo vicini.
    // Se l'aggiornamento fosse sequenziale, b2 vedrebbe b1 già spostato.
    // Con l'aggiornamento simultaneo il comportamento è simmetrico.
    Boid b1({0.0, 0.0},  {0.1, 0.0}, 0);
    Boid b2({5.0, 0.0},  {-0.1, 0.0}, 0);
    std::vector<Boid> stormo_ab = {b1, b2};
    std::vector<Boid> stormo_ba = {b2, b1};  // ordine invertito

    aggiorna(stormo_ab, 0.001, 0.1, 0.4, 1.0, 0.09, {});
    aggiorna(stormo_ba, 0.001, 0.1, 0.4, 1.0, 0.09, {});

    // Le posizioni di b1 nei due stormi devono essere identiche
    // (la simulazione è order-independent)
    CHECK(stormo_ab[0].getPos().x ==
          doctest::Approx(stormo_ba[1].getPos().x).epsilon(1e-8));
    CHECK(stormo_ab[1].getPos().x ==
          doctest::Approx(stormo_ba[0].getPos().x).epsilon(1e-8));
}

TEST_CASE("aggiorna – predatore non modifica il proprio ID dopo il tick") {
    Boid pred({300.0, 300.0}, {0.1, 0.0}, ID_PREDATORE);
    Boid preda({320.0, 300.0}, {0.0, 0.0}, 1);
    std::vector<Boid> stormo = {pred, preda};

    aggiorna(stormo, 0.001, 0.1, 0.4, 1.0, 0.09, {});

    CHECK(stormo[0].getId() == ID_PREDATORE);
    CHECK(stormo[1].getId() == 1);
}

// ════════════════════════════════════════════════════════════════════════════
// COSTRUTTORE E ACCESSORI
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Boid – costruttore default, posizione e velocità in (0,0)") {
    Boid b;
    CHECK(b.getPos().x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(b.getPos().y == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(b.getVel().x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(b.getVel().y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Boid – costruttore con parametri espliciti") {
    Boid b({12.5, -3.0}, {0.4, -0.2}, 2);
    CHECK(b.getPos().x == doctest::Approx(12.5).epsilon(1e-10));
    CHECK(b.getPos().y == doctest::Approx(-3.0).epsilon(1e-10));
    CHECK(b.getVel().x == doctest::Approx(0.4).epsilon(1e-10));
    CHECK(b.getVel().y == doctest::Approx(-0.2).epsilon(1e-10));
    CHECK(b.getId()    == 2);
}

TEST_CASE("Boid – applica aggiorna posizione e velocità correttamente") {
    Boid b({100.0, 200.0}, {0.5, -0.3}, 0);

    // dv = (0.1, 0.2)
    // vel_new = (0.5+0.1, -0.3+0.2) = (0.6, -0.1)
    // pos_new = (100 + 0.6, 200 + (-0.1)) = (100.6, 199.9)
    b.applica({0.1, 0.2});

    CHECK(b.getVel().x == doctest::Approx(0.6).epsilon(1e-10));
    CHECK(b.getVel().y == doctest::Approx(-0.1).epsilon(1e-10));
    CHECK(b.getPos().x == doctest::Approx(100.6).epsilon(1e-10));
    CHECK(b.getPos().y == doctest::Approx(199.9).epsilon(1e-10));
}

TEST_CASE("Boid – applica dv nullo, posizione avanza solo per inerzia") {
    Boid b({0.0, 0.0}, {0.3, 0.7}, 0);
    b.applica({0.0, 0.0});

    CHECK(b.getPos().x == doctest::Approx(0.3).epsilon(1e-10));
    CHECK(b.getPos().y == doctest::Approx(0.7).epsilon(1e-10));
    CHECK(b.getVel().x == doctest::Approx(0.3).epsilon(1e-10));
    CHECK(b.getVel().y == doctest::Approx(0.7).epsilon(1e-10));
}

TEST_CASE("Boid – ID_PREDATORE riconosciuto correttamente") {
    Boid pred({0.0, 0.0}, {0.0, 0.0}, ID_PREDATORE);
    Boid norm({0.0, 0.0}, {0.0, 0.0}, 1);

    CHECK(pred.getId() == ID_PREDATORE);
    CHECK(norm.getId() != ID_PREDATORE);
}

// ════════════════════════════════════════════════════════════════════════════
// UTILITÀ MATEMATICHE QUADRATICHE
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("normaQuadrata e distanzaQuadrata – calcolo esatto senza sqrt") {
    const sf::Vector2<double> a{0.0, 0.0};
    const sf::Vector2<double> b{3.0, 4.0};

    CHECK(normaQuadrata(b) == doctest::Approx(25.0).epsilon(1e-10));
    CHECK(distanzaQuadrata(a, b) == doctest::Approx(25.0).epsilon(1e-10));
}

// ════════════════════════════════════════════════════════════════════════════
// STATISTICHE COLLETTIVE DELLO STORMO
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Statistiche – stormo vuoto") {
    const std::vector<Boid> vuoto;
    const auto stats = calcolaStatistiche(vuoto);

    CHECK(stats.velocitaMedia == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.distanzaMedia == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Statistiche – stormo con 1 boid") {
    const std::vector<Boid> singolo = { Boid({10.0, 20.0}, {3.0, 4.0}, 0) };
    const auto stats = calcolaStatistiche(singolo);

    CHECK(stats.velocitaMedia == doctest::Approx(5.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.distanzaMedia == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Statistiche – calcolo analitico esatto per velocità media e deviazione standard") {
    // 3 boid con velocità 5.0, 3.0, 4.0
    // Media = (5 + 3 + 4) / 3 = 4.0
    // Varianza = ((5-4)^2 + (3-4)^2 + (4-4)^2) / 3 = 2/3 ≈ 0.6666667
    // Dev std = sqrt(2/3) ≈ 0.81649658
    const std::vector<Boid> stormo = {
        Boid({0.0, 0.0}, {3.0, 4.0}, 0),  // norma = 5
        Boid({0.0, 0.0}, {0.0, 3.0}, 0),  // norma = 3
        Boid({0.0, 0.0}, {4.0, 0.0}, 0)   // norma = 4
    };

    const double media = calcolaVelocitaMedia(stormo);
    const double devStd = calcolaDeviazioneStdVelocita(stormo, media);

    CHECK(media == doctest::Approx(4.0).epsilon(1e-10));
    CHECK(devStd == doctest::Approx(std::sqrt(2.0 / 3.0)).epsilon(1e-5));
}

TEST_CASE("Statistiche – calcolo analitico esatto per distanza media e deviazione standard") {
    // 3 boid allineati su asse X: posizioni 0, 3, 7
    // Distanze: d(1,2)=3, d(1,3)=7, d(2,3)=4
    // Distanza media = (3 + 7 + 4) / 3 = 14/3 ≈ 4.6666667
    // Varianza = ((3 - 14/3)^2 + (7 - 14/3)^2 + (4 - 14/3)^2) / 3 = (25/9 + 49/9 + 4/9) / 3 = 26/9
    // Dev std = sqrt(26/9) = sqrt(26)/3 ≈ 1.70029386
    const std::vector<Boid> stormo = {
        Boid({0.0, 0.0}, {1.0, 0.0}, 0),
        Boid({3.0, 0.0}, {1.0, 0.0}, 0),
        Boid({7.0, 0.0}, {1.0, 0.0}, 0)
    };

    const auto stats = calcolaStatistiche(stormo);

    CHECK(stats.distanzaMedia == doctest::Approx(14.0 / 3.0).epsilon(1e-5));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(std::sqrt(26.0) / 3.0).epsilon(1e-5));
    CHECK(stats.velocitaMedia == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
}

}  // namespace brd
