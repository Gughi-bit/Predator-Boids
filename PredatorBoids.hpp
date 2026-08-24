#ifndef PREDATORBOIDS_HPP
#define PREDATORBOIDS_HPP

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace brd {

// ─── Costanti fisiche della simulazione ─────────────────────────────────────
inline constexpr int    ID_PREDATORE        = 999;
inline constexpr double RAGGIO_VISTA_STORMO = 30.0;  // px – coesione/allineamento stormo
inline constexpr double RAGGIO_VISTA_ALTRO  = 50.0;  // px – separazione inter-stormo
inline constexpr double RAGGIO_VISTA_PRED   = 40.0;  // px – campo visivo predatore
inline constexpr double SOGLIA_SEPARAZIONE  = 25.0;  // px – distanza minima prima della repulsione
inline constexpr double RAGGIO_OSTACOLO     = 50.0;  // px – raggio di evitamento ostacolo

// ─── Utilità matematiche ─────────────────────────────────────────────────────
inline double normaQuadrata(const sf::Vector2<double>& v) {
    return v.x * v.x + v.y * v.y;
}

inline double distanzaQuadrata(const sf::Vector2<double>& a, const sf::Vector2<double>& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return dx * dx + dy * dy;
}

double norma(sf::Vector2<double> v);
double distanza(const sf::Vector2<double>& a, const sf::Vector2<double>& b);

class Boid;

// ─── Statistiche collettive dello stormo ─────────────────────────────────────
struct StatisticheStormo {
    double velocitaMedia{0.0};
    double deviazioneStdVelocita{0.0};
    double distanzaMedia{0.0};
    double deviazioneStdDistanza{0.0};
};

StatisticheStormo calcolaStatistiche(const std::vector<Boid>& stormo);
double            calcolaVelocitaMedia(const std::vector<Boid>& stormo);
double            calcolaDeviazioneStdVelocita(const std::vector<Boid>& stormo, double media);
double            calcolaDistanzaMedia(const std::vector<Boid>& stormo);
double            calcolaDeviazioneStdDistanza(const std::vector<Boid>& stormo, double media);

// ─── Classe Boid ─────────────────────────────────────────────────────────────
class Boid {
 public:
  Boid(sf::Vector2<double> p = {0.0, 0.0},
       sf::Vector2<double> v = {0.0, 0.0},
       int id = 0)
      : pos_{p}, vel_{v}, idStormo_{id} {}

  // Accessori (const-correct)
  sf::Vector2<double> getPos() const { return pos_; }
  sf::Vector2<double> getVel() const { return vel_; }
  int                 getId()  const { return idStormo_; }

  // Calcola il delta-velocità per questo boid senza modificare lo stato
  // (operazione const: legge solo la snapshot corrente dello stormo).
  sf::Vector2<double> calcolaDeltaV(const std::vector<Boid>& stormo,
                                     double pesoCoesione,
                                     double pesoAllineamento,
                                     double pesoSeparazione,
                                     double pesoPredatore,
                                     double pesoOstacoli,
                                     const std::vector<sf::Vector2i>& posizioni) const;

  // Applica delta-v e avanza la posizione di un passo.
  void applica(sf::Vector2<double> deltaV);

  // Limita la velocità entro [minV, maxV] (o maxVPred per i predatori).
  void controlloV(double maxV,
                  const sf::Vector2<double>& minV,
                  double maxVPred);

  // Repulsione morbida dai bordi della finestra.
  void bordi(double repulsioneBordi,
             int lunghezzaFinestra,
             int altezzaFinestra,
             int margineX,
             int margineY);

  // Rendering.
  void disegno(sf::RenderWindow& window, sf::CircleShape& c) const;

 private:
  sf::Vector2<double> pos_;
  sf::Vector2<double> vel_;
  int                 idStormo_;
};

// ─── Regole di volo (funzioni libere, operano su snapshot immutabili) ─────────
sf::Vector2<double> coesione(const std::vector<Boid>& vicini,
                              const Boid& boid,
                              double pesoCoesione);

sf::Vector2<double> allineamento(const std::vector<Boid>& vicini,
                                  const Boid& boid,
                                  double pesoAllineamento);

sf::Vector2<double> separazione(const std::vector<Boid>& vicini,
                                 const Boid& boid,
                                 double pesoSeparazione,
                                 double pesoPredatore);

sf::Vector2<double> evitareOstacoli(const Boid& boid,
                                     const std::vector<sf::Vector2i>& posizioni,
                                     double pesoOstacoli);

// ─── Aggiornamento simultaneo dell'intero stormo ─────────────────────────────
// Prima calcola tutti i delta-v su una copia frozen dello stato corrente,
// poi li applica: nessun boid influenza un altro durante lo stesso tick.
void aggiorna(std::vector<Boid>& stormo,
              double pesoCoesione,
              double pesoAllineamento,
              double pesoSeparazione,
              double pesoPredatore,
              double pesoOstacoli,
              const std::vector<sf::Vector2i>& posizioni);

// ─── Classe Ostacolo ─────────────────────────────────────────────────────────
class Ostacolo {
 public:
  void         setPosizione(sf::Vector2i pos) { posizione_ = pos; }
  sf::Vector2i getPosizione() const           { return posizione_; }

  void creazione(sf::RenderWindow& window,
                 const std::vector<sf::Vector2i>& posizioni);

 private:
  sf::Vector2i posizione_{};
};

}  // namespace brd

#endif  // PREDATORBOIDS_HPP
