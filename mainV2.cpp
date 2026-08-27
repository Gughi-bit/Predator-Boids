#include "PredatorBoids.hpp"

#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

namespace {

constexpr int LUNGHEZZA_FINESTRA = 1200;
constexpr int ALTEZZA_FINESTRA   = 800;

constexpr int DEFAULT_N_BOIDS = 200;
constexpr int N_STORMI        = 3;   // stormi "regolari" (i predatori a parte)
constexpr int FREQ_PREDATORI  = 10;  // un predatore ogni FREQ_PREDATORI boid generati

constexpr double MAX_VELOCITY = 2.0;  // limite di velocità dei boid
constexpr double MIN_VELOCITY = 0.2;  // velocità minima dei boid
constexpr double MAX_V_PRED   = 0.4;  // limite di velocità dei predatori

constexpr std::size_t FRAME_STATISTICHE_HUD = 15;  // aggiornamento HUD (frame)
constexpr std::size_t FRAME_STATISTICHE_LOG = 60;  // statistiche su console (frame)

}  // namespace

int main() {
    const brd::ParametriSimulazione parametri;  // pesi di default

    std::cout << "Avvio simulazione Boids con:\n"
              << " - Numero Boids:      " << DEFAULT_N_BOIDS << "\n"
              << " - Peso Coesione:     " << parametri.pesoCoesione << "\n"
              << " - Peso Allineamento: " << parametri.pesoAllineamento << "\n"
              << " - Peso Separazione:  " << parametri.pesoSeparazione << "\n"
              << " - Peso Predatore:    " << parametri.pesoPredatore << "\n"
              << " - Peso Ostacoli:     " << parametri.pesoOstacoli << "\n\n";

    sf::RenderWindow window(
        sf::VideoMode(LUNGHEZZA_FINESTRA, ALTEZZA_FINESTRA),
        "Simulazione Boids");
    window.setFramerateLimit(60);

    // Generazione (pseudo)casuale dello stormo iniziale.
    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> distX(200.0, 1000.0);
    std::uniform_real_distribution<double> distY(200.0, 500.0);
    std::uniform_real_distribution<double> distVx(-0.4, 0.4);
    std::uniform_real_distribution<double> distVy(-0.4, 0.4);
    std::uniform_int_distribution<int>     distId(0, N_STORMI - 1);

    std::vector<brd::Boid> stormo;
    stormo.reserve(static_cast<std::size_t>(DEFAULT_N_BOIDS));

    for (int i = 0; i < DEFAULT_N_BOIDS; ++i) {
        const sf::Vector2<double> p{distX(gen), distY(gen)};
        const sf::Vector2<double> v{distVx(gen), distVy(gen)};
        const int id = (i % FREQ_PREDATORI == 0) ? brd::ID_PREDATORE : distId(gen);
        stormo.emplace_back(p, v, id);
    }

    // Font per l'HUD: prima un file locale, poi i font di sistema più comuni.
    sf::Font font;
    const bool fontCaricato =
        font.loadFromFile("font.ttf") ||
        font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf") ||
        font.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf") ||
        font.loadFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
    if (!fontCaricato) {
        std::cerr << "Attenzione: impossibile caricare il font per le statistiche a schermo.\n";
    }

    sf::Text textStats;
    if (fontCaricato) {
        textStats.setFont(font);
        textStats.setCharacterSize(14);
        textStats.setFillColor(sf::Color(240, 240, 240));
        textStats.setPosition(20.f, 20.f);
    }

    sf::RectangleShape hudBackground(sf::Vector2f(340.f, 150.f));
    hudBackground.setPosition(10.f, 10.f);
    hudBackground.setFillColor(sf::Color(15, 15, 25, 200));
    hudBackground.setOutlineThickness(1.f);
    hudBackground.setOutlineColor(sf::Color(80, 80, 120, 180));

    std::vector<sf::Vector2i> posizioniOstacoli;
    sf::CircleShape           cerchio;
    std::size_t               frameCounter = 0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Left) {
                posizioniOstacoli.push_back(sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Right &&
                !posizioniOstacoli.empty()) {
                posizioniOstacoli.pop_back();
            }
        }

        brd::aggiorna(stormo, parametri, posizioniOstacoli);

        std::for_each(stormo.begin(), stormo.end(), [](brd::Boid& boid) {
            boid.bordiToroidali(LUNGHEZZA_FINESTRA, ALTEZZA_FINESTRA);
            boid.controlloV(MAX_VELOCITY, MIN_VELOCITY, MAX_V_PRED);
        });

        ++frameCounter;
        if (frameCounter % FRAME_STATISTICHE_HUD == 0) {
            const auto stats = brd::calcolaStatistiche(stormo);
            const auto nPredatori = static_cast<std::size_t>(std::count_if(
                stormo.begin(), stormo.end(),
                [](const brd::Boid& b) { return b.isPredatore(); }));

            if (frameCounter % FRAME_STATISTICHE_LOG == 0) {
                std::cout << std::fixed << std::setprecision(3)
                          << "[Frame " << std::setw(5) << frameCounter << "] "
                          << "Vel. media: " << stats.velocitaMedia
                          << " (std: " << stats.deviazioneStdVelocita << ") | "
                          << "Dist. media: " << stats.distanzaMedia
                          << " (std: " << stats.deviazioneStdDistanza << ")\n";
            }

            if (fontCaricato) {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2);
                ss << "=== STATISTICHE STORMO ===\n"
                   << "Boid: " << (stormo.size() - nPredatori)
                   << " | Predatori: " << nPredatori << "\n"
                   << "Velocita media: " << stats.velocitaMedia
                   << " (+/- " << stats.deviazioneStdVelocita << ")\n"
                   << "Distanza media: " << stats.distanzaMedia
                   << " (+/- " << stats.deviazioneStdDistanza << ")\n"
                   << "Ostacoli attivi: " << posizioniOstacoli.size() << "\n"
                   << "Click SX: +ostacolo | DX: -ostacolo";
                textStats.setString(ss.str());
            }
        }

        window.clear(sf::Color(20, 20, 30));

        brd::disegnaOstacoli(window, posizioniOstacoli);

        std::for_each(stormo.begin(), stormo.end(), [&](const brd::Boid& boid) {
            boid.disegna(window, cerchio);
        });

        if (fontCaricato) {
            window.draw(hudBackground);
            window.draw(textStats);
        }

        window.display();
    }

    return 0;
}
