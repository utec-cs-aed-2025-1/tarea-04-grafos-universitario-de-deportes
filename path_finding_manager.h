//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <unordered_set>
#include <set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    BestFirstSearch
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    Graph *current_graph = nullptr;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;
    int render_counter = 0;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;

        // mapa de distancias
        std::unordered_map<Node*, double> dist;

        // nodos a visitar ordenados por distancia
        std::set<Entry> pq;

        // distancias como infinito
        for (auto& [id, node] : graph.nodes) {
            dist[node] = std::numeric_limits<double>::max();
        }

        dist[src] = 0.0;
        pq.insert({src, 0.0});
        parent[src] = nullptr;

        int iterations = 0;
        while (!pq.empty()) {
            Entry current_entry = *pq.begin();
            pq.erase(pq.begin());
            Node* current = current_entry.node;

            iterations++;

            /*
            if (iterations % 1000 == 0) {
                std::cout << "Dijkstra iteration " << iterations << ", queue size: " << pq.size() << std::endl;
            }            
            */

            if (current == dest) {
                std::cout << "Dijkstra llego al destino despues de " << iterations << " iteraciones" << std::endl;
                break;
            }

            // explorar todas las aristas del nodo actual
            for (Edge* edge : current->edges) {
                Node* neighbor = nullptr;
                if (edge->src == current) {
                    neighbor = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    neighbor = edge->src;
                }

                if (neighbor == nullptr) continue;

                // nueva distancia al vecino
                double new_dist = dist[current] + edge->length;

                // si hay un camino mas corto
                if (new_dist < dist[neighbor]) {
                    // se quita la entrada anterior del set
                    pq.erase({neighbor, dist[neighbor]});

                    dist[neighbor] = new_dist;
                    parent[neighbor] = current;

                    pq.insert({neighbor, new_dist});

                    visited_edges.push_back(sfLine(
                        current->coord,
                        neighbor->coord,
                        sf::Color(100, 100, 255, 100),
                        1.0f
                    ));

                    render();
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        
        // g: distancia desde el origen
        std::unordered_map<Node*, double> g_score;
        
        // f: g + heurística
        std::unordered_map<Node*, double> f_score;
        
        // Set de nodos a visitar, ordenados por f_score
        std::set<Entry> open_set;
        
        auto heuristic = [this](Node* node) -> double {
            float dx = node->coord.x - dest->coord.x;
            float dy = node->coord.y - dest->coord.y;
            return std::sqrt(dx * dx + dy * dy);
        };
        
        for (auto& [id, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<double>::max();
            f_score[node] = std::numeric_limits<double>::max();
        }
        
        g_score[src] = 0.0;
        f_score[src] = heuristic(src);
        open_set.insert({src, f_score[src]});
        parent[src] = nullptr;
        
        int iterations = 0;
        while (!open_set.empty()) {
            Entry current_entry = *open_set.begin();
            open_set.erase(open_set.begin());
            Node* current = current_entry.node;
            
            iterations++;
            /*
            if (iterations % 1000 == 0) {
                std::cout << "A* iteration " << iterations << ", open set size: " << open_set.size() << std::endl;
            }            
            */

            if (current == dest) {
                std::cout << "A* llego a su destino despues de " << iterations << " iteraciones" << std::endl;
                break;
            }
            
            for (Edge* edge : current->edges) {
                Node* neighbor = nullptr;
                if (edge->src == current) {
                    neighbor = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    neighbor = edge->src;
                }
                
                if (neighbor == nullptr) continue;
                
                double tentative_g_score = g_score[current] + edge->length;
                
                if (tentative_g_score < g_score[neighbor]) {
                    open_set.erase({neighbor, f_score[neighbor]});
                    
                    parent[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor);
                    
                    open_set.insert({neighbor, f_score[neighbor]});
                    
                    visited_edges.push_back(sfLine(
                        current->coord,
                        neighbor->coord,
                        sf::Color(100, 255, 100, 100),
                        1.0f
                    ));
                    
                    render();
                }
            }
        }

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {};

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        render_counter++;
        if (render_counter % 5000 != 0) {
            return;
        }

        window_manager->clear();

        // grafo base
        if (current_graph != nullptr) {
            current_graph->draw();
        }

        // dibuja todas las aristas visitadas hasta ahora
        for (sfLine& line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // origen
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // destino
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }

        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {};

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }
        
        current_graph = &graph;
        path.clear();
        visited_edges.clear();
        render_counter = 0;

        // ejecutar algoritmo
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            default:
                break;
        }
        
        current_graph = nullptr;
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
