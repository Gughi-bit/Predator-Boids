#include "PredatorBoids.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <numeric>

namespace brd {

namespace {

// Un boid "ordinario" (non predatore): è lui che forma lo stormo.
// Usato dalle statistiche per escludere i predatori dai calcoli.
bool nonPredatore(const Boid& b) {
    return !b.isPredatore();
}

}  // namespace

// ---------------------------------------------------------------------------
// Funzioni geometriche
// ---------------------------------------------------------------------------

double norma(sf::Vector2<double> v) {
    return std::sqrt(normaQuadrata(v));
}

double distanza(const sf::Vector2<double>& a, const sf::Vector2<double>& b) {
    return std::sqrt(distanzaQuadrata(a, b));
}

// ---------------------------------------------------------------------------
// Regole dello stormo
// ---------------------------------------------------------------------------

sf::Vector2<double> coesione(const std::vector<const Boid*>& vicini,
                             const Boid& boid,
                             double pesoCoesione) {
    if (vicini.empty()) {
        return {0.0, 0.0};
    }

    const auto centroMassa = std::accumulate(
        vicini.begin(), vicini.end(), sf::Vector2<double>{0.0, 0.0},
        [](sf::Vector2<double> acc, const Boid* b) { return acc + b->getPos(); });

    const double invN = 1.0 / static_cast<double>(vicini.size());
    const sf::Vector2<double> centro{centroMassa.x * invN, centroMassa.y * invN};

    return (centro - boid.getPos()) * pesoCoesione;
}

sf::Vector2<double> allineamento(const std::vector<const Boid*>& vicini,
                                 const Boid& boid,
                                 double pesoAllineamento) {
    if (vicini.empty()) {
        return {0.0, 0.0};
    }

    const auto velSomma = std::accumulate(
        vicini.begin(), vicini.end(), sf::Vector2<double>{0.0, 0.0},
        [](sf::Vector2<double> acc, const Boid* b) { return acc + b->getVel(); });

    const double invN = 1.0 / static_cast<double>(vicini.size());
    const sf::Vector2<double> velMedia{velSomma.x * invN, velSomma.y * invN};

    return (velMedia - boid.getVel()) * pesoAllineamento;
}

sf::Vector2<double> separazione(const std::vector<const Boid*>& vicini,
                                const Boid& boid,
                                double pesoSeparazione,
                                double pesoPredatore) {
    const auto& pThis = boid.getPos();
    const bool thisPredatore = boid.isPredatore();

    constexpr double sogliaSep2   = SOGLIA_SEPARAZIONE * SOGLIA_SEPARAZIONE;
    constexpr double raggioAltro2 = RAGGIO_VISTA_ALTRO * RAGGIO_VISTA_ALTRO;

    return std::accumulate(
        vicini.begin(), vicini.end(), sf::Vector2<double>{0.0, 0.0},
        [&](sf::Vector2<double> acc, const Boid* altroPtr) {
            const Boid& altro = *altroPtr;
            const auto& pAltro = altro.getPos();
            const double d2 = distanzaQuadrata(pThis, pAltro);

            // La preda avverte il predatore già entro RAGGIO_VISTA_ALTRO; per
            // tutti gli altri accoppiamenti vale la sola SOGLIA_SEPARAZIONE.
            const bool coppiaPredaPredatore = !thisPredatore && altro.isPredatore();
            const double soglia2 = coppiaPredaPredatore ? raggioAltro2 : sogliaSep2;

            if (d2 > 0.0 && d2 <= soglia2) {
                const double d = std::sqrt(d2);
                const double fattore = coppiaPredaPredatore ? pesoPredatore : pesoSeparazione;
                return acc + (pThis - pAltro) / d * fattore;
            }
            return acc;
        });
}

sf::Vector2<double> evitareOstacoli(const Boid& boid,
                                    const std::vector<sf::Vector2i>& posizioni,
                                    double pesoOstacoli) {
    const auto& pThis = boid.getPos();
    constexpr double raggio2 = RAGGIO_OSTACOLO * RAGGIO_OSTACOLO;

    return std::accumulate(
        posizioni.begin(), posizioni.end(), sf::Vector2<double>{0.0, 0.0},
        [&](sf::Vector2<double> acc, const sf::Vector2i& ostacolo) {
            const sf::Vector2<double> posOst{static_cast<double>(ostacolo.x),
                                             static_cast<double>(ostacolo.y)};
            const double d2 = distanzaQuadrata(pThis, posOst);

            if (d2 > raggio2) {
                return acc;  // ostacolo fuori dal raggio d'azione
            }
            if (d2 > 0.0) {
                const double invD = 1.0 / std::sqrt(d2);
                return acc + (pThis - posOst) * (invD * pesoOstacoli);
            }
            // Caso degenere: boid esattamente sull'ostacolo →
            // spinta deterministica verso l'alto.
            return acc + sf::Vector2<double>{0.0, -pesoOstacoli};
        });
}

// ---------------------------------------------------------------------------
// Boid
// ---------------------------------------------------------------------------

sf::Vector2<double> Boid::calcolaDeltaV(const std::vector<Boid>& stormo,
                                        const ParametriSimulazione& parametri,
                                        const std::vector<sf::Vector2i>& posizioni) const {
    sf::Vector2<double> dv{0.0, 0.0};

    constexpr double raggioStormo2 = RAGGIO_VISTA_STORMO * RAGGIO_VISTA_STORMO;
    constexpr double raggioAltro2  = RAGGIO_VISTA_ALTRO * RAGGIO_VISTA_ALTRO;
    constexpr double raggioPred2   = RAGGIO_VISTA_PRED * RAGGIO_VISTA_PRED;
    constexpr double sogliaSep2    = SOGLIA_SEPARAZIONE * SOGLIA_SEPARAZIONE;

    // I vicini vengono collezionati come PUNTATORI (niente copie di Boid).
    if (isPredatore()) {
        std::vector<const Boid*> prede;        // sole prede entro il raggio di caccia
        std::vector<const Boid*> ingombranti;  // boid qualsiasi fisicamente troppo vicini
        prede.reserve(stormo.size());
        ingombranti.reserve(stormo.size());

        std::for_each(stormo.begin(), stormo.end(), [&](const Boid& altro) {
            if (&altro == this) {
                return;
            }
            const double d2 = distanzaQuadrata(pos_, altro.getPos());

            // Il predatore caccia SOLO i boid: gli altri predatori non sono prede.
            if (!altro.isPredatore() && d2 <= raggioPred2) {
                prede.push_back(&altro);
            }
            // Distanza di sicurezza da chiunque sia troppo vicino.
            if (d2 <= sogliaSep2) {
                ingombranti.push_back(&altro);
            }
        });

        dv += coesione(prede, *this, parametri.pesoCoesione);
        dv += allineamento(prede, *this, parametri.pesoAllineamento);
        dv += separazione(ingombranti, *this, parametri.pesoSeparazione,
                          parametri.pesoPredatore);
    } else {
        std::vector<const Boid*> viciniStessoStormo;
        std::vector<const Boid*> viciniAltro;  // predatori e stormi diversi
        viciniStessoStormo.reserve(stormo.size());
        viciniAltro.reserve(stormo.size());

        std::for_each(stormo.begin(), stormo.end(), [&](const Boid& altro) {
            if (&altro == this) {
                return;
            }
            const double d2 = distanzaQuadrata(pos_, altro.getPos());

            if (altro.isPredatore() || altro.getId() != idStormo_) {
                // Predatori e stormi "altri": percepiti entro RAGGIO_VISTA_ALTRO.
                if (d2 <= raggioAltro2) {
                    viciniAltro.push_back(&altro);
                }
            } else if (d2 <= raggioStormo2) {
                // Compagni di stormo: percepiti entro RAGGIO_VISTA_STORMO.
                viciniStessoStormo.push_back(&altro);
            }
        });

        dv += coesione(viciniStessoStormo, *this, parametri.pesoCoesione);
        dv += allineamento(viciniStessoStormo, *this, parametri.pesoAllineamento);
        dv += separazione(viciniStessoStormo, *this, parametri.pesoSeparazione,
                          parametri.pesoPredatore);
        dv += separazione(viciniAltro, *this, parametri.pesoSeparazione,
                          parametri.pesoPredatore);
    }

    dv += evitareOstacoli(*this, posizioni, parametri.pesoOstacoli);
    return dv;
}

void Boid::applica(sf::Vector2<double> deltaV) {
    vel_ += deltaV;
    pos_ += vel_;
}

void Boid::controlloV(double maxV, double minV, double maxVPred) {
    const double n2 = normaQuadrata(vel_);
    if (n2 < 1e-24) {
        return;  // velocità nulla: nessuna direzione da preservare
    }

    const double n = std::sqrt(n2);
    const double invN = 1.0 / n;
    const double limiteSup = isPredatore() ? maxVPred : maxV;

    if (n > limiteSup) {
        vel_ = vel_ * (limiteSup * invN);
    } else if (n < minV) {
        vel_ = vel_ * (minV * invN);
    }
}

void Boid::bordiToroidali(int lunghezzaFinestra, int altezzaFinestra) {
    const double w = static_cast<double>(lunghezzaFinestra);
    const double h = static_cast<double>(altezzaFinestra);

    if (pos_.x < 0.0) {
        pos_.x += w;
    } else if (pos_.x >= w) {
        pos_.x -= w;
    }

    if (pos_.y < 0.0) {
        pos_.y += h;
    } else if (pos_.y >= h) {
        pos_.y -= h;
    }
}

void Boid::disegna(sf::RenderWindow& window, sf::CircleShape& c) const {
    static const sf::Color coloriStormo[] = {
        sf::Color::Blue,
        sf::Color::White,
        sf::Color::Green,
    };

    const float raggio = isPredatore() ? 5.f : 3.f;

    c.setRadius(raggio);
    c.setOrigin(raggio, raggio);  // il centro della figura coincide con pos_
    c.setFillColor(isPredatore()
                       ? sf::Color::Red
                       : coloriStormo[static_cast<std::size_t>(idStormo_) % 3]);
    c.setPosition(
        sf::Vector2f(static_cast<float>(pos_.x), static_cast<float>(pos_.y)));
    window.draw(c);
}

// ---------------------------------------------------------------------------
// Aggiornamento e disegno dello stormo
// ---------------------------------------------------------------------------

void aggiorna(std::vector<Boid>& stormo,
              const ParametriSimulazione& parametri,
              const std::vector<sf::Vector2i>& posizioni) {
    std::vector<sf::Vector2<double>> deltaVs;
    deltaVs.reserve(stormo.size());

    // Prima fase: tutti i delta calcolati dalle posizioni di inizio tick.
    std::transform(stormo.begin(), stormo.end(), std::back_inserter(deltaVs),
                   [&](const Boid& b) {
                       return b.calcolaDeltaV(stormo, parametri, posizioni);
                   });

    // Seconda fase: applicazione simultanea dei delta.
    auto dvIt = deltaVs.begin();
    std::for_each(stormo.begin(), stormo.end(),
                  [&](Boid& b) { b.applica(*dvIt++); });
}

void disegnaOstacoli(sf::RenderWindow& window,
                     const std::vector<sf::Vector2i>& posizioni) {
    if (posizioni.empty()) {
        return;
    }

    sf::CircleShape o(10.f);
    o.setOrigin(10.f, 10.f);  // il centro del cerchio coincide con il click
    o.setFillColor(sf::Color(200, 100, 50));

    std::for_each(posizioni.begin(), posizioni.end(), [&](const sf::Vector2i& p) {
        o.setPosition(sf::Vector2f(static_cast<float>(p.x),
                                   static_cast<float>(p.y)));
        window.draw(o);
    });
}

// ---------------------------------------------------------------------------
// Statistiche (solo boid non predatori)
// ---------------------------------------------------------------------------

double calcolaVelocitaMedia(const std::vector<Boid>& stormo) {
    const std::size_t nBoid = static_cast<std::size_t>(
        std::count_if(stormo.begin(), stormo.end(), nonPredatore));
    if (nBoid == 0) {
        return 0.0;
    }

    const double sommaVel = std::accumulate(
        stormo.begin(), stormo.end(), 0.0,
        [](double acc, const Boid& b) {
            return nonPredatore(b) ? acc + norma(b.getVel()) : acc;
        });

    return sommaVel / static_cast<double>(nBoid);
}

double calcolaDeviazioneStdVelocita(const std::vector<Boid>& stormo, double media) {
    const std::size_t nBoid = static_cast<std::size_t>(
        std::count_if(stormo.begin(), stormo.end(), nonPredatore));
    if (nBoid == 0) {
        return 0.0;
    }

    const double sommaSqDiff = std::accumulate(
        stormo.begin(), stormo.end(), 0.0, [media](double acc, const Boid& b) {
            if (nonPredatore(b)) {
                const double diff = norma(b.getVel()) - media;
                return acc + diff * diff;
            }
            return acc;
        });

    return std::sqrt(sommaSqDiff / static_cast<double>(nBoid));
}

double calcolaDistanzaMedia(const std::vector<Boid>& stormo) {
    const std::size_t nBoid = static_cast<std::size_t>(
        std::count_if(stormo.begin(), stormo.end(), nonPredatore));
    if (nBoid < 2) {
        return 0.0;
    }

    // Distanza media su tutte le coppie non ordinate di boid: per sua natura
    // richiede una visita a due livelli; il livello interno usa accumulate.
    double sommaDistanze = 0.0;
    for (auto it = stormo.begin(); it != stormo.end(); ++it) {
        if (!nonPredatore(*it)) {
            continue;
        }
        const auto& posI = it->getPos();
        sommaDistanze = std::accumulate(
            std::next(it), stormo.end(), sommaDistanze,
            [&posI](double acc, const Boid& b) {
                return nonPredatore(b) ? acc + distanza(posI, b.getPos()) : acc;
            });
    }

    const double numCoppie = static_cast<double>(nBoid * (nBoid - 1) / 2);
    return sommaDistanze / numCoppie;
}

double calcolaDeviazioneStdDistanza(const std::vector<Boid>& stormo, double media) {
    const std::size_t nBoid = static_cast<std::size_t>(
        std::count_if(stormo.begin(), stormo.end(), nonPredatore));
    if (nBoid < 2) {
        return 0.0;
    }

    double sommaSqDiff = 0.0;
    for (auto it = stormo.begin(); it != stormo.end(); ++it) {
        if (!nonPredatore(*it)) {
            continue;
        }
        const auto& posI = it->getPos();
        sommaSqDiff = std::accumulate(
            std::next(it), stormo.end(), sommaSqDiff,
            [&posI, media](double acc, const Boid& b) {
                if (nonPredatore(b)) {
                    const double diff = distanza(posI, b.getPos()) - media;
                    return acc + diff * diff;
                }
                return acc;
            });
    }

    const double numCoppie = static_cast<double>(nBoid * (nBoid - 1) / 2);
    return std::sqrt(sommaSqDiff / numCoppie);
}

StatisticheStormo calcolaStatistiche(const std::vector<Boid>& stormo) {
    StatisticheStormo stats;
    stats.velocitaMedia = calcolaVelocitaMedia(stormo);
    stats.deviazioneStdVelocita =
        calcolaDeviazioneStdVelocita(stormo, stats.velocitaMedia);
    stats.distanzaMedia = calcolaDistanzaMedia(stormo);
    stats.deviazioneStdDistanza =
        calcolaDeviazioneStdDistanza(stormo, stats.distanzaMedia);
    return stats;
}

}  // namespace brd
