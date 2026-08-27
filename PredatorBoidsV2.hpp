#ifndef PREDATORBOIDS_HPP
#define PREDATORBOIDS_HPP

#include <SFML/Graphics.hpp>
#include <vector>

namespace brd {

// ---------------------------------------------------------------------------
// Costanti della simulazione
// ---------------------------------------------------------------------------

// Un Boid con questo id non appartiene a nessuno stormo: ha il ruolo di
// predatore e caccia i boid di tutti gli stormi.
inline constexpr int ID_PREDATORE = 999;

// Raggio entro cui un boid percepisce i compagni del PROPRIO stormo
// (coesione e allineamento).
inline constexpr double RAGGIO_VISTA_STORMO = 30.0;

// Raggio entro cui un boid percepisce individui di ALTRI stormi e, in
// particolare, i predatori: la fuga dal predatore scatta già a questa
// distanza (con peso amplificato), non solo al contatto ravvicinato.
inline constexpr double RAGGIO_VISTA_ALTRO = 50.0;

// Raggio di caccia del predatore: più corto di RAGGIO_VISTA_ALTRO, così la
// preda avverte il predatore prima di essere vista.
inline constexpr double RAGGIO_VISTA_PRED = 40.0;

// Distanza sotto la quale due boid si respingono fisicamente.
inline constexpr double SOGLIA_SEPARAZIONE = 25.0;

// Raggio d'azione degli ostacoli aggiunti dall'utente con il click.
inline constexpr double RAGGIO_OSTACOLO = 50.0;

// ---------------------------------------------------------------------------
// Funzioni geometriche di supporto
// ---------------------------------------------------------------------------

[[nodiscard]] inline double normaQuadrata(const sf::Vector2<double>& v) noexcept {
    return v.x * v.x + v.y * v.y;
}

[[nodiscard]] inline double distanzaQuadrata(const sf::Vector2<double>& a,
                                             const sf::Vector2<double>& b) noexcept {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return dx * dx + dy * dy;
}

[[nodiscard]] double norma(sf::Vector2<double> v);
[[nodiscard]] double distanza(const sf::Vector2<double>& a, const sf::Vector2<double>& b);

// ---------------------------------------------------------------------------
// Parametri e statistiche
// ---------------------------------------------------------------------------

class Boid;

// Pesi della simulazione, riuniti in un'unica struttura per evitare firme
// con molti parametri scalari. I valori di default sono quelli usati da main.
struct ParametriSimulazione {
    double pesoCoesione     = 0.005;
    double pesoAllineamento = 0.5;
    double pesoSeparazione  = 0.7;
    double pesoPredatore    = 1.0;  // peso (amplificato) della fuga dal predatore
    double pesoOstacoli     = 0.09;
};

struct StatisticheStormo {
    double velocitaMedia{0.0};
    double deviazioneStdVelocita{0.0};
    double distanzaMedia{0.0};
    double deviazioneStdDistanza{0.0};
};

// Le statistiche considerano SOLO i boid non predatori: i predatori non fanno
// parte dello stormo e, avendo velocità e distanze molto diverse, ne
// falserebbero media e deviazione standard.
[[nodiscard]] StatisticheStormo calcolaStatistiche(const std::vector<Boid>& stormo);
[[nodiscard]] double calcolaVelocitaMedia(const std::vector<Boid>& stormo);
[[nodiscard]] double calcolaDeviazioneStdVelocita(const std::vector<Boid>& stormo,
                                                  double media);
[[nodiscard]] double calcolaDistanzaMedia(const std::vector<Boid>& stormo);
[[nodiscard]] double calcolaDeviazioneStdDistanza(const std::vector<Boid>& stormo,
                                                  double media);

// ---------------------------------------------------------------------------
// Boid
// ---------------------------------------------------------------------------

class Boid {
public:
    Boid(sf::Vector2<double> p = {0.0, 0.0},
         sf::Vector2<double> v = {0.0, 0.0},
         int id = 0)
        : pos_{p}, vel_{v}, idStormo_{id} {}

    [[nodiscard]] sf::Vector2<double> getPos() const noexcept { return pos_; }
    [[nodiscard]] sf::Vector2<double> getVel() const noexcept { return vel_; }
    [[nodiscard]] int getId() const noexcept { return idStormo_; }

    // Un boid è un predatore se il suo id vale ID_PREDATORE.
    [[nodiscard]] bool isPredatore() const noexcept { return idStormo_ == ID_PREDATORE; }

    // Delta-velocità risultante dalle regole dello stormo, dalla presenza di
    // predatori e dagli ostacoli, calcolata sulle posizioni attuali.
    [[nodiscard]] sf::Vector2<double> calcolaDeltaV(
        const std::vector<Boid>& stormo,
        const ParametriSimulazione& parametri,
        const std::vector<sf::Vector2i>& posizioni) const;

    void applica(sf::Vector2<double> deltaV);

    // Limita la norma della velocità all'intervallo [minV, maxV] (per i
    // predatori il limite superiore è maxVPred), preservandone la direzione.
    void controlloV(double maxV, double minV, double maxVPred);

    // Fa riaffiorare il boid dal lato opposto quando supera un bordo.
    void bordiToroidali(int lunghezzaFinestra, int altezzaFinestra);

    void disegna(sf::RenderWindow& window, sf::CircleShape& c) const;

private:
    sf::Vector2<double> pos_;
    sf::Vector2<double> vel_;
    int idStormo_;
};

// ---------------------------------------------------------------------------
// Regole dello stormo
// ---------------------------------------------------------------------------
// Le tre regole ricevono i vicini come vettore di PUNTATORI const: in questo
// modo, a ogni frame, non vengono copiati i Boid ma solo i loro indirizzi.

// Sterza verso il centro di massa dei vicini.
[[nodiscard]] sf::Vector2<double> coesione(const std::vector<const Boid*>& vicini,
                                           const Boid& boid,
                                           double pesoCoesione);

// Sterza per allineare la propria velocità a quella media dei vicini.
[[nodiscard]] sf::Vector2<double> allineamento(const std::vector<const Boid*>& vicini,
                                               const Boid& boid,
                                               double pesoAllineamento);

// Si allontana dai vicini troppo vicini. La preda avverte il predatore già
// entro RAGGIO_VISTA_ALTRO e fugge con peso amplificato; tra boid ordinari
// (stesso stormo o stormi diversi) la repulsione agisce solo entro
// SOGLIA_SEPARAZIONE.
[[nodiscard]] sf::Vector2<double> separazione(const std::vector<const Boid*>& vicini,
                                              const Boid& boid,
                                              double pesoSeparazione,
                                              double pesoPredatore);

// Si allontana dagli ostacoli entro RAGGIO_OSTACOLO.
[[nodiscard]] sf::Vector2<double> evitareOstacoli(const Boid& boid,
                                                  const std::vector<sf::Vector2i>& posizioni,
                                                  double pesoOstacoli);

// Aggiorna lo stormo in modo SIMULTANEO: tutti i delta-velocità vengono
// calcolati a partire dalle posizioni e velocità di inizio tick e solo
// dopo vengono applicati.
void aggiorna(std::vector<Boid>& stormo,
              const ParametriSimulazione& parametri,
              const std::vector<sf::Vector2i>& posizioni);

void disegnaOstacoli(sf::RenderWindow& window,
                     const std::vector<sf::Vector2i>& posizioni);

}  // namespace brd

#endif  // PREDATORBOIDS_HPP
