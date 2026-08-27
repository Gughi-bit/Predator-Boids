#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "PredatorBoids.hpp"

#include <cmath>
#include <vector>

namespace brd {

// ---------------------------------------------------------------------------
// Separazione
// ---------------------------------------------------------------------------

TEST_CASE("Separazione – stesso stormo, distanza entro soglia") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 2);
    Boid b2({110.0, 110.0}, {0.0, 0.0}, 2);

    const auto dv = separazione({&b2}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(-1.0 / std::sqrt(2.0)).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(-1.0 / std::sqrt(2.0)).epsilon(1e-5));
}

TEST_CASE("Separazione – fuga dal predatore, peso amplificato") {
    Boid predatore({500.0, 500.0}, {0.0, 0.0}, ID_PREDATORE);
    Boid preda    ({510.0, 510.0}, {0.0, 0.0}, 1);

    const auto dv = separazione({&predatore}, preda, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(2.0 / std::sqrt(2.0)).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(2.0 / std::sqrt(2.0)).epsilon(1e-5));
}

TEST_CASE("Separazione – fuga dal predatore tra SOGLIA_SEPARAZIONE e RAGGIO_VISTA_ALTRO") {
    // A 35px la preda è oltre la soglia di contatto ma entro il raggio di
    // vista del predatore: la fuga (amplificata) deve essere già attiva.
    Boid preda({0.0,  0.0}, {0.0, 0.0}, 0);
    Boid pred ({35.0, 0.0}, {0.0, 0.0}, ID_PREDATORE);

    const auto dv = separazione({&pred}, preda, 0.5, 2.0);

    CHECK(dv.x == doctest::Approx(-2.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – predatore esattamente a RAGGIO_VISTA_ALTRO, fuga al limite") {
    Boid preda({0.0, 0.0}, {0.0, 0.0}, 0);
    Boid pred ({RAGGIO_VISTA_ALTRO, 0.0}, {0.0, 0.0}, ID_PREDATORE);

    const auto dv = separazione({&pred}, preda, 0.5, 2.0);

    CHECK(dv.x == doctest::Approx(-2.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – predatore oltre RAGGIO_VISTA_ALTRO, nessuna fuga") {
    Boid preda({0.0, 0.0}, {0.0, 0.0}, 0);
    Boid pred ({RAGGIO_VISTA_ALTRO + 1.0, 0.0}, {0.0, 0.0}, ID_PREDATORE);

    const auto dv = separazione({&pred}, preda, 0.5, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – boid di altro stormo oltre SOGLIA_SEPARAZIONE, nessuna repulsione") {
    // Un boid ordinario di altro stormo, pur essendo entro RAGGIO_VISTA_ALTRO,
    // respinge solo al contatto ravvicinato (SOGLIA_SEPARAZIONE).
    Boid b1({0.0,  0.0}, {0.0, 0.0}, 0);
    Boid b2({35.0, 0.0}, {0.0, 0.0}, 1);

    const auto dv = separazione({&b2}, b1, 0.5, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – distanza esattamente a SOGLIA_SEPARAZIONE, repulsione al limite") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({SOGLIA_SEPARAZIONE, 0.0}, {0.0, 0.0}, 0);

    const auto dv = separazione({&b2}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – distanza appena oltre la soglia, dv nullo") {
    Boid b1({0.0,   0.0}, {0.0, 0.0}, 0);
    Boid b2({100.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = separazione({&b2}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – lista vicini vuota, dv nullo") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);

    const auto dv = separazione({}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – più vicini, contributi si sommano") {
    Boid b1({  0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({ 10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = separazione({&b2, &b3}, b1, 1.0, 2.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – predatore non fugge da boid normali") {
    Boid pred ({0.0,  0.0}, {0.0, 0.0}, ID_PREDATORE);
    Boid preda({10.0, 0.0}, {0.0, 0.0}, 1);

    const auto dv = separazione({&preda}, pred, 1.0, 5.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Separazione – peso zero, dv nullo anche entro soglia") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({5.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = separazione({&b2}, b1, 0.0, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Allineamento
// ---------------------------------------------------------------------------

TEST_CASE("Allineamento – un vicino, steering verso la sua velocità") {
    Boid b1({300.0, 300.0}, {1.0, 0.0}, 0);
    Boid b2({320.0, 320.0}, {0.0, 1.0}, 0);

    const auto dv = allineamento({&b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 1.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – lista vicini vuota, dv nullo") {
    Boid b1({300.0, 300.0}, {1.0, 0.5}, 0);

    const auto dv = allineamento({}, b1, 5.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – velocità già allineate ai vicini, dv nullo") {
    Boid b1({0.0,  0.0}, {1.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {1.0, 0.0}, 0);
    Boid b3({20.0, 0.0}, {1.0, 0.0}, 0);

    const auto dv = allineamento({&b2, &b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – più vicini, media uniforme") {
    Boid b1({0.0,   0.0}, {0.0, 0.0}, 0);
    Boid b2({10.0,  0.0}, {2.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 2.0}, 0);

    const auto dv = allineamento({&b2, &b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – peso zero, dv nullo") {
    Boid b1({0.0,  0.0}, {0.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {5.0, 3.0}, 0);

    const auto dv = allineamento({&b2}, b1, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Allineamento – velocità opposte, steering doppio") {
    Boid b1({0.0,  0.0}, { 1.0, 0.0}, 0);
    Boid b2({10.0, 0.0}, {-1.0, 0.0}, 0);

    const auto dv = allineamento({&b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(-2.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Coesione
// ---------------------------------------------------------------------------

TEST_CASE("Coesione – due vicini, centro di massa corretto") {
    Boid b1({200.0, 200.0}, {0.0, 0.0}, 0);
    Boid b2({250.0, 250.0}, {0.0, 0.0}, 0);
    Boid b3({300.0, 300.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({&b2, &b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(75.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(75.0).epsilon(1e-10));
}

TEST_CASE("Coesione – lista vicini vuota, dv nullo") {
    Boid b1({200.0, 200.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – boid già al centro di massa, dv nullo") {
    Boid b1({  0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({ 10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({&b2, &b3}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – un solo vicino, steering diretto verso di lui") {
    Boid b1({ 0.0, 0.0}, {0.0, 0.0}, 0);
    Boid b2({50.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({&b2}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(50.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – peso zero, dv nullo") {
    Boid b1({0.0,   0.0}, {0.0, 0.0}, 0);
    Boid b2({100.0, 0.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({&b2}, b1, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Coesione – molti vicini distribuiti simmetricamente, dv nullo") {
    Boid b1({0.0,   0.0}, {0.0, 0.0}, 0);
    Boid b2({10.0,  0.0}, {0.0, 0.0}, 0);
    Boid b3({-10.0, 0.0}, {0.0, 0.0}, 0);
    Boid b4({0.0,  10.0}, {0.0, 0.0}, 0);
    Boid b5({0.0, -10.0}, {0.0, 0.0}, 0);

    const auto dv = coesione({&b2, &b3, &b4, &b5}, b1, 1.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Evitare ostacoli
// ---------------------------------------------------------------------------

TEST_CASE("EvitareOstacoli – ostacolo entro raggio, repulsione corretta") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(102, 102)};

    const auto dv = evitareOstacoli(b1, ostacoli, 1.5);

    const double atteso = -1.5 / std::sqrt(2.0);
    CHECK(dv.x == doctest::Approx(atteso).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(atteso).epsilon(1e-5));
}

TEST_CASE("EvitareOstacoli – ostacolo esattamente sul bordo del raggio") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const int r = static_cast<int>(RAGGIO_OSTACOLO);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(r, 0)};

    const auto dv = evitareOstacoli(b1, ostacoli, 1.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx( 0.0).epsilon(1e-5));
}

TEST_CASE("EvitareOstacoli – ostacolo appena fuori raggio, dv nullo") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(500, 500)};

    const auto dv = evitareOstacoli(b1, ostacoli, 1.5);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("EvitareOstacoli – lista ostacoli vuota, dv nullo") {
    Boid b1({100.0, 100.0}, {0.0, 0.0}, 0);

    const auto dv = evitareOstacoli(b1, {}, 1.5);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("EvitareOstacoli – tutti gli ostacoli nel raggio contribuiscono") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {
        sf::Vector2i(10, 0),
        sf::Vector2i(0, 10)
    };

    const auto dv = evitareOstacoli(b1, ostacoli, 1.0);

    CHECK(dv.x == doctest::Approx(-1.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(-1.0).epsilon(1e-5));
}

TEST_CASE("EvitareOstacoli – boid esattamente sull'ostacolo, spinta deterministica") {
    Boid b1({10.0, 10.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(10, 10)};

    const auto dv = evitareOstacoli(b1, ostacoli, 1.5);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(-1.5).epsilon(1e-10));
}

TEST_CASE("EvitareOstacoli – peso zero, dv nullo anche a contatto") {
    Boid b1({0.0, 0.0}, {0.0, 0.0}, 0);
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(5, 0)};

    const auto dv = evitareOstacoli(b1, ostacoli, 0.0);

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// ControlloV
// ---------------------------------------------------------------------------

TEST_CASE("ControlloV – velocità superiore al massimo, ridotta preservando direzione") {
    Boid b1({0.0, 0.0}, {20.0, 20.0}, 0);

    b1.controlloV(1.0, 0.08, 0.5);

    CHECK(norma(b1.getVel()) <= doctest::Approx(1.0).epsilon(1e-5));
    CHECK(b1.getVel().x == doctest::Approx(b1.getVel().y).epsilon(1e-10));
}

TEST_CASE("ControlloV – velocità sotto il minimo, alzata preservando direzione") {
    Boid b1({0.0, 0.0}, {0.001, 0.0}, 0);

    b1.controlloV(1.0, 0.08, 0.5);

    CHECK(norma(b1.getVel()) >= doctest::Approx(0.08).epsilon(1e-5));
    CHECK(b1.getVel().y == doctest::Approx(0.0).epsilon(1e-8));
}

TEST_CASE("ControlloV – predatore rispetta maxVPred, non maxV") {
    Boid pred({0.0, 0.0}, {1.0, 1.0}, ID_PREDATORE);

    pred.controlloV(1.0, 0.08, 0.5);

    CHECK(norma(pred.getVel()) <= doctest::Approx(0.5).epsilon(1e-5));
}

TEST_CASE("ControlloV – velocità già entro i limiti, nessuna modifica") {
    Boid b1({0.0, 0.0}, {0.3, 0.3}, 0);

    b1.controlloV(1.0, 0.08, 0.5);

    const auto vel = b1.getVel();
    CHECK(vel.x == doctest::Approx(0.3).epsilon(1e-10));
    CHECK(vel.y == doctest::Approx(0.3).epsilon(1e-10));
}

TEST_CASE("ControlloV – velocità esattamente al massimo, non modificata") {
    const double v = 1.0 / std::sqrt(2.0);
    Boid b1({0.0, 0.0}, {v, v}, 0);

    b1.controlloV(1.0, 0.08, 0.5);

    CHECK(norma(b1.getVel()) <= doctest::Approx(1.0).epsilon(1e-8));
    CHECK(b1.getVel().x == doctest::Approx(v).epsilon(1e-8));
}

TEST_CASE("ControlloV – predatore con velocità entro maxVPred, non modificata") {
    Boid pred({0.0, 0.0}, {0.1, 0.0}, ID_PREDATORE);

    pred.controlloV(1.0, 0.08, 0.5);

    CHECK(pred.getVel().x == doctest::Approx(0.1).epsilon(1e-10));
    CHECK(pred.getVel().y == doctest::Approx(0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Bordi toroidali
// ---------------------------------------------------------------------------

TEST_CASE("Bordi toroidali – uscita a sinistra rientra a destra") {
    Boid b({-5.0, 400.0}, {0.0, 0.0}, 0);
    b.bordiToroidali(1200, 800);
    CHECK(b.getPos().x == doctest::Approx(1195.0));
}

TEST_CASE("Bordi toroidali – uscita a destra rientra a sinistra") {
    Boid b({1205.0, 400.0}, {0.0, 0.0}, 0);
    b.bordiToroidali(1200, 800);
    CHECK(b.getPos().x == doctest::Approx(5.0));
}

TEST_CASE("Bordi toroidali – uscita dall'alto rientra dal basso") {
    Boid b({400.0, -5.0}, {0.0, 0.0}, 0);
    b.bordiToroidali(1200, 800);
    CHECK(b.getPos().y == doctest::Approx(795.0));
}

TEST_CASE("Bordi toroidali – uscita dal basso rientra dall'alto") {
    Boid b({400.0, 805.0}, {0.0, 0.0}, 0);
    b.bordiToroidali(1200, 800);
    CHECK(b.getPos().y == doctest::Approx(5.0));
}

TEST_CASE("Bordi toroidali – uscita in angolo, rientro su entrambi gli assi") {
    Boid b({-5.0, -5.0}, {0.0, 0.0}, 0);
    b.bordiToroidali(1200, 800);
    CHECK(b.getPos().x == doctest::Approx(1195.0));
    CHECK(b.getPos().y == doctest::Approx(795.0));
}

// ---------------------------------------------------------------------------
// Norma e distanza
// ---------------------------------------------------------------------------

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

TEST_CASE("normaQuadrata e distanzaQuadrata – calcolo esatto senza sqrt") {
    const sf::Vector2<double> a{0.0, 0.0};
    const sf::Vector2<double> b{3.0, 4.0};

    CHECK(normaQuadrata(b) == doctest::Approx(25.0).epsilon(1e-10));
    CHECK(distanzaQuadrata(a, b) == doctest::Approx(25.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// calcolaDeltaV (scelta dei vicini secondo ruolo e distanza)
// ---------------------------------------------------------------------------

TEST_CASE("calcolaDeltaV – il boid considera solo il proprio stormo entro RAGGIO_VISTA_STORMO") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, 0),  // osservatore
        Boid({10.0, 0.0}, {1.0, 0.0}, 0),  // stesso stormo, entro 30
        Boid({28.0, 0.0}, {0.0, 1.0}, 1),  // altro stormo, oltre 25: nessun effetto
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // coesione (0.1) + allineamento (0.1) + separazione (-0.5)
    CHECK(dv.x == doctest::Approx(-0.3).epsilon(1e-5));
    // il boid di altro stormo non trascina l'allineamento
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – compagno di stormo oltre RAGGIO_VISTA_STORMO, nessun effetto") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, 0),
        Boid({35.0, 0.0}, {1.0, 1.0}, 0),  // stesso stormo ma oltre 30
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – compagno esattamente a RAGGIO_VISTA_STORMO, coesione al limite") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, 0),
        Boid({30.0, 0.0}, {0.0, 0.0}, 0),  // esattamente a 30
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // solo coesione: la separazione agirebbe entro 25
    CHECK(dv.x == doctest::Approx(0.3).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – preda fugge dal predatore a 35px, entro RAGGIO_VISTA_ALTRO") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, 0),           // preda osservatrice
        Boid({35.0, 0.0}, {0.0, 0.0}, ID_PREDATORE), // predatore a 35px
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // fuga amplificata: (0-35)/35 * pesoPredatore(2.0)
    CHECK(dv.x == doctest::Approx(-2.0).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – preda ignora il predatore oltre RAGGIO_VISTA_ALTRO") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, 0),
        Boid({55.0, 0.0}, {0.0, 0.0}, ID_PREDATORE),  // oltre 50
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – il predatore insegue la preda entro RAGGIO_VISTA_PRED") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, ID_PREDATORE),  // predatore osservatore
        Boid({30.0, 0.0}, {0.0, 0.0}, 1),             // preda a 30px (≤ 40)
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // coesione verso la preda: 30 * pesoCoesione(0.01)
    CHECK(dv.x == doctest::Approx(0.3).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – il predatore NON insegue un altro predatore") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, ID_PREDATORE),
        Boid({30.0, 0.0}, {0.0, 0.0}, ID_PREDATORE),  // altro predatore, NON è una preda
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // niente caccia (30 > 25: nemmeno separazione fisica)
    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – predatori vicini si respingono soltanto, senza attrarsi") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, ID_PREDATORE),
        Boid({20.0, 0.0}, {0.0, 0.0}, ID_PREDATORE),  // entro SOGLIA_SEPARAZIONE
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    // solo separazione fisica: (0-20)/20 * pesoSeparazione(0.5)
    CHECK(dv.x == doctest::Approx(-0.5).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – predatore oltre RAGGIO_VISTA_PRED, nessuna caccia") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 0.0};

    std::vector<Boid> stormo = {
        Boid({0.0,  0.0}, {0.0, 0.0}, ID_PREDATORE),
        Boid({45.0, 0.0}, {0.0, 0.0}, 1),  // oltre 40
    };

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, {});

    CHECK(dv.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("calcolaDeltaV – l'ostacolo contribuisce alla fuga") {
    const ParametriSimulazione p{0.01, 0.1, 0.5, 2.0, 1.5};

    std::vector<Boid> stormo = {
        Boid({0.0,   0.0},   {0.0, 0.0}, 0),  // osservatore
        Boid({500.0, 500.0}, {0.0, 0.0}, 1),  // fuori da ogni raggio
    };
    const std::vector<sf::Vector2i> ostacoli = {sf::Vector2i(10, 0)};

    const auto dv = stormo[0].calcolaDeltaV(stormo, p, ostacoli);

    // repulsione dall'ostacolo: (0-10)/10 * pesoOstacoli(1.5)
    CHECK(dv.x == doctest::Approx(-1.5).epsilon(1e-5));
    CHECK(dv.y == doctest::Approx(0.0).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Aggiorna
// ---------------------------------------------------------------------------

TEST_CASE("aggiorna – la posizione cambia dopo un tick") {
    Boid b1({200.0, 200.0}, {0.1, 0.0}, 0);
    Boid b2({220.0, 200.0}, {0.1, 0.0}, 0);
    std::vector<Boid> stormo = {b1, b2};

    const double px_prima = stormo[0].getPos().x;
    const ParametriSimulazione p{0.001, 0.1, 0.4, 1.0, 0.09};
    aggiorna(stormo, p, {});

    CHECK(stormo[0].getPos().x != doctest::Approx(px_prima).epsilon(1e-10));
}

TEST_CASE("aggiorna – stormo con un solo boid, si muove per inerzia") {
    Boid b1({200.0, 200.0}, {0.5, 0.3}, 0);
    std::vector<Boid> stormo = {b1};

    const ParametriSimulazione p{0.001, 0.1, 0.4, 1.0, 0.09};
    aggiorna(stormo, p, {});

    CHECK(stormo[0].getPos().x == doctest::Approx(200.5).epsilon(1e-8));
    CHECK(stormo[0].getPos().y == doctest::Approx(200.3).epsilon(1e-8));
}

TEST_CASE("aggiorna – stormo vuoto, nessun crash") {
    std::vector<Boid> stormo;
    const ParametriSimulazione p{0.001, 0.1, 0.4, 1.0, 0.09};
    CHECK_NOTHROW(aggiorna(stormo, p, {}));
}

TEST_CASE("aggiorna – simultaneità: posizioni pre-tick usate per tutti i calcoli") {
    Boid b1({0.0, 0.0}, { 0.1, 0.0}, 0);
    Boid b2({5.0, 0.0}, {-0.1, 0.0}, 0);
    std::vector<Boid> stormo_ab = {b1, b2};
    std::vector<Boid> stormo_ba = {b2, b1};

    const ParametriSimulazione p{0.001, 0.1, 0.4, 1.0, 0.09};
    aggiorna(stormo_ab, p, {});
    aggiorna(stormo_ba, p, {});

    CHECK(stormo_ab[0].getPos().x ==
          doctest::Approx(stormo_ba[1].getPos().x).epsilon(1e-8));
    CHECK(stormo_ab[1].getPos().x ==
          doctest::Approx(stormo_ba[0].getPos().x).epsilon(1e-8));
}

TEST_CASE("aggiorna – predatore non modifica il proprio ID dopo il tick") {
    Boid pred ({300.0, 300.0}, {0.1, 0.0}, ID_PREDATORE);
    Boid preda({320.0, 300.0}, {0.0, 0.0}, 1);
    std::vector<Boid> stormo = {pred, preda};

    const ParametriSimulazione p{0.001, 0.1, 0.4, 1.0, 0.09};
    aggiorna(stormo, p, {});

    CHECK(stormo[0].getId() == ID_PREDATORE);
    CHECK(stormo[1].getId() == 1);
}

// ---------------------------------------------------------------------------
// Boid (costruttori, applica, ruolo)
// ---------------------------------------------------------------------------

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
    CHECK(pred.isPredatore());
    CHECK_FALSE(norm.isPredatore());
}

// ---------------------------------------------------------------------------
// ParametriSimulazione
// ---------------------------------------------------------------------------

TEST_CASE("ParametriSimulazione – valori di default") {
    const ParametriSimulazione p;
    CHECK(p.pesoCoesione     == doctest::Approx(0.005).epsilon(1e-10));
    CHECK(p.pesoAllineamento == doctest::Approx(0.5).epsilon(1e-10));
    CHECK(p.pesoSeparazione  == doctest::Approx(0.7).epsilon(1e-10));
    CHECK(p.pesoPredatore    == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(p.pesoOstacoli     == doctest::Approx(0.09).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// Statistiche
// ---------------------------------------------------------------------------

TEST_CASE("Statistiche – stormo vuoto") {
    const std::vector<Boid> vuoto;
    const auto stats = calcolaStatistiche(vuoto);

    CHECK(stats.velocitaMedia       == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.distanzaMedia       == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Statistiche – stormo con 1 boid") {
    const std::vector<Boid> singolo = { Boid({10.0, 20.0}, {3.0, 4.0}, 0) };
    const auto stats = calcolaStatistiche(singolo);

    CHECK(stats.velocitaMedia       == doctest::Approx(5.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.distanzaMedia       == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Statistiche – i predatori sono esclusi dai calcoli") {
    const std::vector<Boid> stormo = {
        Boid({  0.0,   0.0}, { 1.0, 0.0}, 0),
        Boid({  3.0,   0.0}, { 1.0, 0.0}, 0),
        Boid({100.0, 100.0}, {50.0, 0.0}, ID_PREDATORE)  // non deve influire
    };

    const auto stats = calcolaStatistiche(stormo);

    CHECK(stats.velocitaMedia       == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(stats.distanzaMedia       == doctest::Approx(3.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Statistiche – calcolo analitico esatto per velocità media e deviazione standard") {
    const std::vector<Boid> stormo = {
        Boid({0.0, 0.0}, {3.0, 4.0}, 0),  // norma 5
        Boid({0.0, 0.0}, {0.0, 3.0}, 0),  // norma 3
        Boid({0.0, 0.0}, {4.0, 0.0}, 0)   // norma 4
    };

    const double media  = calcolaVelocitaMedia(stormo);
    const double devStd = calcolaDeviazioneStdVelocita(stormo, media);

    CHECK(media  == doctest::Approx(4.0).epsilon(1e-10));
    CHECK(devStd == doctest::Approx(std::sqrt(2.0 / 3.0)).epsilon(1e-5));
}

TEST_CASE("Statistiche – calcolo analitico esatto per distanza media e deviazione standard") {
    const std::vector<Boid> stormo = {
        Boid({0.0, 0.0}, {1.0, 0.0}, 0),
        Boid({3.0, 0.0}, {1.0, 0.0}, 0),
        Boid({7.0, 0.0}, {1.0, 0.0}, 0)
    };

    const auto stats = calcolaStatistiche(stormo);

    CHECK(stats.distanzaMedia       == doctest::Approx(14.0 / 3.0).epsilon(1e-5));
    CHECK(stats.deviazioneStdDistanza == doctest::Approx(std::sqrt(26.0) / 3.0).epsilon(1e-5));
    CHECK(stats.velocitaMedia       == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(stats.deviazioneStdVelocita == doctest::Approx(0.0).epsilon(1e-10));
}

}  // namespace brd
