#include "PredatorBoids.hpp"

#include <algorithm>
#include <numeric>

namespace brd {


double norma(sf::Vector2<double> v) {
    return std::sqrt(normaQuadrata(v));
}

double distanza(const sf::Vector2<double>& a, const sf::Vector2<double>& b) {
    return std::sqrt(distanzaQuadrata(a, b));
}


sf::Vector2<double> coesione(const std::vector<Boid>& vicini,
                              const Boid& boid,
                              double pesoCoesione) {
    if (vicini.empty()) return {0.0, 0.0};

    const auto centroMassa = std::accumulate(
        vicini.begin(), vicini.end(),
        sf::Vector2<double>{0.0, 0.0},
        [](sf::Vector2<double> acc, const Boid& b) {
            return acc + b.getPos();
        });

    const double invN = 1.0 / static_cast<double>(vicini.size());
    const sf::Vector2<double> centroide{centroMassa.x * invN, centroMassa.y * invN};

    return (centroide - boid.getPos()) * pesoCoesione;
}

sf::Vector2<double> allineamento(const std::vector<Boid>& vicini,
                                  const Boid& boid,
                                  double pesoAllineamento) {
    if (vicini.empty()) return {0.0, 0.0};

    const auto velSomma = std::accumulate(
        vicini.begin(), vicini.end(),
        sf::Vector2<double>{0.0, 0.0},
        [](sf::Vector2<double> acc, const Boid& b) {
            return acc + b.getVel();
        });

    const double invN = 1.0 / static_cast<double>(vicini.size());
    const sf::Vector2<double> velMedia{velSomma.x * invN, velSomma.y * invN};

    return (velMedia - boid.getVel()) * pesoAllineamento;
}

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


sf::Vector2<double> Boid::calcolaDeltaV(
    const std::vector<Boid>& stormo,
    double pesoCoesione,
    double pesoAllineamento,
    double pesoSeparazione,
    double pesoPredatore,
    double pesoOstacoli,
    const std::vector<sf::Vector2i>& posizioni) const {

    sf::Vector2<double> dv{0.0, 0.0};

    if (idStormo_ == ID_PREDATORE) {
        constexpr double raggioPred2 = RAGGIO_VISTA_PRED * RAGGIO_VISTA_PRED;
        std::vector<Boid> prede;
        prede.reserve(stormo.size());

        std::copy_if(stormo.begin(), stormo.end(), std::back_inserter(prede),
                     [&](const Boid& altro) {
                         return &altro != this &&
                                distanzaQuadrata(pos_, altro.getPos()) <= raggioPred2;
                     });

        if (!prede.empty()) {
            dv += coesione(prede, *this, pesoCoesione);
            dv += allineamento(prede, *this, pesoAllineamento);
        }
        dv += separazione(prede, *this, pesoSeparazione, pesoPredatore);

    } else {
        constexpr double raggioStormo2 = RAGGIO_VISTA_STORMO * RAGGIO_VISTA_STORMO;
        constexpr double raggioAltro2  = RAGGIO_VISTA_ALTRO * RAGGIO_VISTA_ALTRO;

        std::vector<Boid> vicini_stormo;
        std::vector<Boid> vicini_altro_stormo;
        vicini_stormo.reserve(stormo.size());
        vicini_altro_stormo.reserve(stormo.size());

        std::for_each(stormo.begin(), stormo.end(), [&](const Boid& altro) {
            if (&altro == this) return;
            const double d2 = distanzaQuadrata(pos_, altro.getPos());

            if (d2 <= raggioStormo2 && altro.getId() == idStormo_) {
                vicini_stormo.push_back(altro);
            } else if (d2 <= raggioAltro2 && altro.getId() != idStormo_) {
                vicini_altro_stormo.push_back(altro);
            }
        });

        if (!vicini_stormo.empty()) {
            dv += coesione(vicini_stormo, *this, pesoCoesione);
            dv += allineamento(vicini_stormo, *this, pesoAllineamento);
            dv += separazione(vicini_stormo, *this, pesoSeparazione, pesoPredatore);
        }

        if (!vicini_altro_stormo.empty()) {
            dv += separazione(vicini_altro_stormo, *this, pesoSeparazione, pesoPredatore);
        }
    }

    dv += evitareOstacoli(*this, posizioni, pesoOstacoli);
    return dv;
}

void Boid::applica(sf::Vector2<double> deltaV) {
    vel_ += deltaV;
    pos_ += vel_;
}

void Boid::controlloV(double maxV,
                       const sf::Vector2<double>& minV,
                       double maxVPred) {
    const double n2 = normaQuadrata(vel_);
    if (n2 < 1e-24) return;  

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
}

void Boid::bordi(double repulsioneBordi,
                  int lunghezzaFinestra,
                  int altezzaFinestra,
                  int margineX,
                  int margineY) {
    const double invMargineX = repulsioneBordi / static_cast<double>(margineX);
    const double invMargineY = repulsioneBordi / static_cast<double>(margineY);
    const double limiteDestro = static_cast<double>(lunghezzaFinestra - margineX);
    const double limiteInferiore = static_cast<double>(altezzaFinestra - margineY);

    if (pos_.x < margineX) {
        vel_.x += (margineX - pos_.x) * invMargineX;
    }
    if (pos_.x > limiteDestro) {
        vel_.x -= (pos_.x - limiteDestro) * invMargineX;
    }
    if (pos_.y < margineY) {
        vel_.y += (margineY - pos_.y) * invMargineY;
    }
    if (pos_.y > limiteInferiore) {
        vel_.y -= (pos_.y - limiteInferiore) * invMargineY;
    }
}

void Boid::disegno(sf::RenderWindow& window, sf::CircleShape& c) const {
    static const sf::Color colori[] = {
        sf::Color::Blue,   
        sf::Color::White,  
        sf::Color::Green,
    };

    if (idStormo_ == ID_PREDATORE) {
        c.setFillColor(sf::Color::Red);
    } else {
        const int idx = idStormo_ % 3;
        c.setFillColor(colori[idx]);
    }

    c.setRadius(idStormo_ == ID_PREDATORE ? 5.f : 3.f);  
    c.setPosition(sf::Vector2f(static_cast<float>(pos_.x),
                               static_cast<float>(pos_.y)));
    window.draw(c);
}


void aggiorna(std::vector<Boid>& stormo,
              double pesoCoesione,
              double pesoAllineamento,
              double pesoSeparazione,
              double pesoPredatore,
              double pesoOstacoli,
              const std::vector<sf::Vector2i>& posizioni) {
    std::vector<sf::Vector2<double>> deltaVs;
    deltaVs.reserve(stormo.size());

    std::transform(stormo.begin(), stormo.end(), std::back_inserter(deltaVs),
                   [&](const Boid& b) {
                       return b.calcolaDeltaV(stormo, pesoCoesione, pesoAllineamento,
                                              pesoSeparazione, pesoPredatore,
                                              pesoOstacoli, posizioni);
                   });

    auto dvIt = deltaVs.begin();
    std::for_each(stormo.begin(), stormo.end(), [&](Boid& b) {
        b.applica(*dvIt++);
    });
}


void Ostacolo::creazione(sf::RenderWindow& window,
                          const std::vector<sf::Vector2i>& posizioni) {
    sf::CircleShape o(10.f);
    o.setFillColor(sf::Color(200, 100, 50));  

    std::for_each(posizioni.begin(), posizioni.end(), [&](const sf::Vector2i& p) {
        o.setPosition(sf::Vector2f(static_cast<float>(p.x),
                                   static_cast<float>(p.y)));
        window.draw(o);
    });
}


double calcolaVelocitaMedia(const std::vector<Boid>& stormo) {
    if (stormo.empty()) return 0.0;

    const double sommaVel = std::accumulate(
        stormo.begin(), stormo.end(), 0.0,
        [](double acc, const Boid& b) {
            return acc + norma(b.getVel());
        });

    return sommaVel / static_cast<double>(stormo.size());
}

double calcolaDeviazioneStdVelocita(const std::vector<Boid>& stormo, double media) {
    if (stormo.empty()) return 0.0;

    const double sommaSqDiff = std::accumulate(
        stormo.begin(), stormo.end(), 0.0,
        [media](double acc, const Boid& b) {
            const double diff = norma(b.getVel()) - media;
            return acc + diff * diff;
        });

    return std::sqrt(sommaSqDiff / static_cast<double>(stormo.size()));
}

double calcolaDistanzaMedia(const std::vector<Boid>& stormo) {
    const std::size_t n = stormo.size();
    if (n < 2) return 0.0;

    double sommaDistanze = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto posI = stormo[i].getPos();
        sommaDistanze = std::accumulate(
            stormo.begin() + static_cast<std::ptrdiff_t>(i + 1), stormo.end(),
            sommaDistanze,
            [&](double acc, const Boid& b) {
                return acc + distanza(posI, b.getPos());
            });
    }

    const double numCoppie = static_cast<double>(n * (n - 1) / 2);
    return sommaDistanze / numCoppie;
}

double calcolaDeviazioneStdDistanza(const std::vector<Boid>& stormo, double media) {
    const std::size_t n = stormo.size();
    if (n < 2) return 0.0;

    double sommaSqDiff = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto posI = stormo[i].getPos();
        sommaSqDiff = std::accumulate(
            stormo.begin() + static_cast<std::ptrdiff_t>(i + 1), stormo.end(),
            sommaSqDiff,
            [&](double acc, const Boid& b) {
                const double diff = distanza(posI, b.getPos()) - media;
                return acc + diff * diff;
            });
    }

    const double numCoppie = static_cast<double>(n * (n - 1) / 2);
    return std::sqrt(sommaSqDiff / numCoppie);
}

StatisticheStormo calcolaStatistiche(const std::vector<Boid>& stormo) {
    StatisticheStormo stats;
    stats.velocitaMedia = calcolaVelocitaMedia(stormo);
    stats.deviazioneStdVelocita = calcolaDeviazioneStdVelocita(stormo, stats.velocitaMedia);
    stats.distanzaMedia = calcolaDistanzaMedia(stormo);
    stats.deviazioneStdDistanza = calcolaDeviazioneStdDistanza(stormo, stats.distanzaMedia);
    return stats;
}

}  

