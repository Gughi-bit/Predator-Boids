#include "PredatorBoids.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

// ─── Parametri della finestra ────────────────────────────────────────────────
static constexpr int LUNGHEZZA_FINESTRA = 1200;
static constexpr int ALTEZZA_FINESTRA   = 800;
static constexpr int MARGINE_X          = 100;
static constexpr int MARGINE_Y          = 100;

// ─── Parametri della popolazione di default ──────────────────────────────────
static constexpr int DEFAULT_N_BOIDS        = 300;
static constexpr int N_STORMI               = 3;   // ID stormo: 0, 1, 2
static constexpr int FREQ_PREDATORI         = 10;  // 1 predatore ogni N boid

// ─── Parametri di volo di default ───────────────────────────────────────────
static constexpr double MAX_VELOCITY        = 0.5;
static constexpr double MIN_VELOCITY        = 0.08;
static constexpr double MAX_V_PRED          = 0.25;

static constexpr double DEFAULT_PESO_COESIONE    = 0.001;
static constexpr double DEFAULT_PESO_ALLINEAMENTO = 0.1;
static constexpr double DEFAULT_PESO_SEPARAZIONE  = 0.4;
static constexpr double PESO_PREDATORE            = 1.0;
static constexpr double PESO_OSTACOLI             = 0.09;
static constexpr double REPULSIONE_BORDI          = 1.5;

int main(int argc, char* argv[]) {
    int    nBoids           = DEFAULT_N_BOIDS;
    double pesoCoesione     = DEFAULT_PESO_COESIONE;
    double pesoAllineamento = DEFAULT_PESO_ALLINEAMENTO;
    double pesoSeparazione  = DEFAULT_PESO_SEPARAZIONE;

    // Validazione rigorosa degli argomenti da riga di comando (se forniti)
    // Uso opzionale: ./boids [n_boids] [peso_separazione] [peso_allineamento] [peso_coesione]
    if (argc > 1) {
        try {
            if (argc >= 2) nBoids           = std::stoi(argv[1]);
            if (argc >= 3) pesoSeparazione  = std::stod(argv[2]);
            if (argc >= 4) pesoAllineamento = std::stod(argv[3]);
            if (argc >= 5) pesoCoesione     = std::stod(argv[4]);

            if (nBoids <= 0 || pesoSeparazione < 0.0 || pesoAllineamento < 0.0 || pesoCoesione < 0.0) {
                std::cerr << "Errore: parametri non validi. I valori devono essere strettamente positivi o nulli per i pesi e n_boids > 0.\n"
                          << "Uso: " << argv[0] << " [n_boids] [peso_separazione] [peso_allineamento] [peso_coesione]\n";
                return 1;
            }
        } catch (const std::exception& e) {
            std::cerr << "Errore nel parsing dei parametri di input: " << e.what() << "\n"
                      << "Uso: " << argv[0] << " [n_boids] [peso_separazione] [peso_allineamento] [peso_coesione]\n";
            return 1;
        }
    }

    std::cout << "Avvio simulazione Boids con:\n"
              << " - Numero Boids:      " << nBoids << "\n"
              << " - Peso Separazione:  " << pesoSeparazione << "\n"
              << " - Peso Allineamento: " << pesoAllineamento << "\n"
              << " - Peso Coesione:     " << pesoCoesione << "\n\n";

    sf::RenderWindow window(
        sf::VideoMode(LUNGHEZZA_FINESTRA, ALTEZZA_FINESTRA),
        "Simulazione Boids");
    window.setFramerateLimit(60);

    // ── Generazione casuale dello stormo ──────────────────────────────────────
    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> distX(200.0, 1000.0);
    std::uniform_real_distribution<double> distY(200.0,  500.0);
    std::uniform_real_distribution<double> distVx(-0.4, 0.4);
    std::uniform_real_distribution<double> distVy(-0.4, 0.4);
    std::uniform_int_distribution<int>     distId(0, N_STORMI - 1);

    std::vector<brd::Boid> stormo;
    stormo.reserve(static_cast<std::size_t>(nBoids));

    for (int i = 0; i < nBoids; ++i) {
        const sf::Vector2<double> p{distX(gen), distY(gen)};
        const sf::Vector2<double> v{distVx(gen), distVy(gen)};
        // Ogni FREQ_PREDATORI boid è un predatore (ID = 999)
        const int id = (i % FREQ_PREDATORI == 0) ? brd::ID_PREDATORE : distId(gen);
        stormo.emplace_back(p, v, id);
    }

    // ── Variabili di stato ───────────────────────────────────────────────────
    std::vector<sf::Vector2i> posizioniOstacoli;
    brd::Ostacolo              ostacolo;
    sf::CircleShape            cerchio;
    const sf::Vector2<double>  minV{MIN_VELOCITY, MIN_VELOCITY};
    std::size_t                frameCounter = 0;

    // ── Loop principale ───────────────────────────────────────────────────────
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // Click sinistro: aggiungi un ostacolo
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Left) {
                posizioniOstacoli.push_back(sf::Mouse::getPosition(window));
            }
            // Click destro: rimuovi l'ultimo ostacolo
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Right &&
                !posizioniOstacoli.empty()) {
                posizioniOstacoli.pop_back();
            }
        }

        // ── Aggiornamento simultaneo ──────────────────────────────────────────
        brd::aggiorna(stormo, pesoCoesione, pesoAllineamento,
                      pesoSeparazione, PESO_PREDATORE,
                      PESO_OSTACOLI, posizioniOstacoli);

        std::for_each(stormo.begin(), stormo.end(), [&](brd::Boid& boid) {
            boid.bordi(REPULSIONE_BORDI, LUNGHEZZA_FINESTRA, ALTEZZA_FINESTRA,
                       MARGINE_X, MARGINE_Y);
            boid.controlloV(MAX_VELOCITY, minV, MAX_V_PRED);
        });

        // ── Calcolo e visualizzazione periodica statistiche collettive ────────
        ++frameCounter;
        if (frameCounter % 60 == 0) {
            const auto stats = brd::calcolaStatistiche(stormo);
            std::cout << std::fixed << std::setprecision(3)
                      << "[Frame " << std::setw(5) << frameCounter << "] "
                      << "Vel. media: " << stats.velocitaMedia
                      << " (std: " << stats.deviazioneStdVelocita << ") | "
                      << "Dist. media: " << stats.distanzaMedia
                      << " (std: " << stats.deviazioneStdDistanza << ")\n";
        }

        // ── Rendering ─────────────────────────────────────────────────────────
        window.clear(sf::Color(20, 20, 30));  // sfondo scuro

        ostacolo.creazione(window, posizioniOstacoli);

        std::for_each(stormo.begin(), stormo.end(), [&](const brd::Boid& boid) {
            boid.disegno(window, cerchio);
        });

        window.display();
    }

    return 0;
}

