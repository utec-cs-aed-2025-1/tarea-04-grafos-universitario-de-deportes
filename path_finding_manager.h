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
#include <queue>


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

        // Operador para std::set
        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
        
        // Operador para std::priority_queue
        bool operator > (const Entry& other) const {
            return dist > other.dist;
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
        
        // f: g + heuristica
        std::unordered_map<Node*, double> f_score;
        
        // min-heap de nodos a visitar, ordenados por f_score 
        //(greater se usa para que el menor este arriba)
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_set;
        
        // Set de nodos ya procesados
        std::unordered_set<Node*> closed_set;
        
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
        open_set.push({src, f_score[src]});
        parent[src] = nullptr;
        
        int iterations = 0;
        while (!open_set.empty()) {
            Entry current_entry = open_set.top();
            open_set.pop();
            Node* current = current_entry.node;
            
            // si el nodo ya fue procesado, saltarlo (puede haber duplicados en el heap)
            if (closed_set.find(current) != closed_set.end()) {
                continue;
            }
            
            // marcar el nodo como procesado
            closed_set.insert(current);
            
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
                
                // no procesar vecinos que ya estan en closed_set
                if (closed_set.find(neighbor) != closed_set.end()) {
                    continue;
                }
                
                double tentative_g_score = g_score[current] + edge->length;
                
                if (tentative_g_score < g_score[neighbor]) {
                    parent[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor);
                    
                    open_set.push({neighbor, f_score[neighbor]});
                    
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

    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        
        // Set de nodos a visitar ordenados por heurística
        std::set<Entry> open_set;
        
        // conjunto de nodos visitados
        std::unordered_set<Node*> visited;
        
        auto heuristic = [this](Node* node) -> double {
            float dx = node->coord.x - dest->coord.x;
            float dy = node->coord.y - dest->coord.y;
            return std::sqrt(dx * dx + dy * dy);
        };
        
        // inicializar el nodo origen
        open_set.insert({src, heuristic(src)});
        parent[src] = nullptr;
        
        // mientras haya nodos por visitar
        int iterations = 0;
        while (!open_set.empty()) {
            // nodo con menor euristica
            Entry current_entry = *open_set.begin();
            open_set.erase(open_set.begin());
            Node* current = current_entry.node;
            
            // marcar como visitado
            visited.insert(current);
            
            iterations++;
            /*
            if (iterations % 1000 == 0) {
                std::cout << "Best-First Search iteration " << iterations << ", open set size: " << open_set.size() << std::endl;
            }            
            */

            
            // si se llega al destino, break
            if (current == dest) {
                std::cout << "Best-First Search llego a su destino despues de " << iterations << " iteraciones" << std::endl;
                break;
            }
            
            // se exploran todas las aristas del nodo actual
            for (Edge* edge : current->edges) {
                // determinar el nodo vecino segun la direccion de la arista
                Node* neighbor = nullptr;
                if (edge->src == current) {
                    neighbor = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    neighbor = edge->src;
                }
                
                if (neighbor == nullptr) continue;
                
                // si el vecino no ha sido visitado.
                if (visited.find(neighbor) == visited.end()) {
                    // calcular heuristica del vecino
                    double h = heuristic(neighbor);
                    
                    // actualizar el padre
                    parent[neighbor] = current;
                    
                    // insertar en el set de los abiertos
                    open_set.insert({neighbor, h});
                    
                    // marcar en los visitados
                    visited.insert(neighbor);
                    
                    visited_edges.push_back(sfLine(
                        current->coord,
                        neighbor->coord,
                        sf::Color(255, 100, 255, 100),  // magenta
                        1.0f
                    ));
                    
                    render();
                }
            }
        }

        set_final_path(parent);
    }

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
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        // ¿el nodo es alcanzable?
        if (parent.find(dest) == parent.end()) {
            std::cout << "No se encontro un camino al destino" << std::endl;
            return;
        }

        Node* current = dest;
        double total_distance = 0.0;

        // reconstruccion del camino desde destino a source con el mapa de padres

        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* prev = parent[current];

            if (prev != nullptr) {
                // distancia euclidiana
                float dx = current->coord.x - prev->coord.x;
                float dy = current->coord.y - prev->coord.y;
                total_distance += std::sqrt(dx * dx + dy * dy);
                
                // agregar la linea color amarillo
                path.push_back(sfLine(
                    prev->coord,
                    current->coord,
                    sf::Color::Yellow,
                    2.0f
                ));
            }

            current = prev;
        }
        
        std::cout << "Path length (Euclidean): " << total_distance << " units" << std::endl;
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        std::cout << "Iniciando algoritmo desde el nodo " << src->id << " hasta el nodo " << dest->id << std::endl;
        
        current_graph = &graph;
        path.clear();
        visited_edges.clear();
        render_counter = 0;

        // ejecutar algoritmo
        switch (algorithm) {
            case Dijkstra:
                std::cout << "Ejecutando algoritmo Dijkstra..." << std::endl;
                dijkstra(graph);
                std::cout << "Dijkstra encontro camino con " << path.size() << " segmentos" << std::endl;
                break;
            case AStar:
                std::cout << "Ejecutando algoritmo A*..." << std::endl;
                a_star(graph);
                std::cout << "A* encontro camino con " << path.size() << " segmentos" << std::endl;
                break;
            case BestFirstSearch:
                std::cout << "Ejecutando algoritmo Best-First Search..." << std::endl;
                best_first_search(graph);
                std::cout << "Best-First Search encontro camino con " << path.size() << " segmentos" << std::endl;
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