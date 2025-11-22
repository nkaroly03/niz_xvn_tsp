#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Solver : public rclcpp::Node{
    private:
    struct Vec2{ double x, y; };
    struct Population{
        std::unique_ptr<size_t[]> ids;
        double cost;
    };

    static constexpr double M_MUTATION_RATIO = 0.2;

    std::string m_file_name;
    std::mt19937_64 m_rand;
    std::vector<Vec2> m_coors;
    std::unique_ptr<double[]> m_distances;
    size_t m_population_size;
    std::unique_ptr<Population[]> m_pops;
    Population m_pop_buf;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_publisher;

    void read_file(){
        std::ifstream file{m_file_name};
        if (!file.is_open()){
            RCLCPP_INFO(this->get_logger(), "Failed to open <%s>\n", m_file_name.data());
            exit(1);
        }

        std::string first_line;
        std::getline(file, first_line);

        double x, y;
        char c;

        while ((file >> x).get(c) >> y)
            m_coors.push_back(Vec2{x, y});
    }
    void calculate_distances(){
        m_distances = std::make_unique<double[]>(m_coors.size() * m_coors.size());

        double *it = m_distances.get();
        for (const auto &v1 : m_coors)
            for (const auto &v2 : m_coors)
                *it++ = std::hypot(v2.x - v1.x, v2.y - v1.y);
    }

    double calculate_cost(std::unique_ptr<size_t[]> &ids) noexcept{
        double cost = 0.0;
        for (size_t i = 0; i < m_coors.size() - 1; ++i)
            cost += m_distances[ids[i] * m_coors.size() + ids[i + 1]];
        return cost;
    }
    void create_population(){
        m_pops = std::make_unique<Population[]>(m_population_size);
        for (auto it = &m_pops[0]; it != &m_pops[m_population_size]; ++it){
            it->ids = std::make_unique<size_t[]>(m_coors.size());
            for (size_t i = 0; i < m_coors.size(); ++i)
                it->ids[i] = i;
            std::shuffle(&it->ids[0], &it->ids[m_coors.size()], m_rand);
            it->cost = calculate_cost(it->ids);
        }
        std::sort(&m_pops[0], &m_pops[m_population_size], [](const Population &p1, const Population &p2){ return p1.cost < p2.cost; });
    }

    void crossover() noexcept{
        size_t population_size_half = m_population_size / 2;
        size_t pop_idx1 = m_rand() % population_size_half, pop_idx2 = population_size_half + m_rand() % population_size_half;
        if (m_rand() % 2 == 0)
            std::swap(pop_idx1, pop_idx2);
        
        const Population &p1 = m_pops[pop_idx1], &p2 = m_pops[pop_idx2];

        size_t crossover_point = m_rand() % (m_coors.size() + 1);
        std::copy(&p1.ids[0], &p1.ids[crossover_point], &m_pop_buf.ids[0]);

        size_t i = 0;
        while (crossover_point < m_coors.size()){
            if (std::find(&m_pop_buf.ids[0], &m_pop_buf.ids[crossover_point], p2.ids[i]) == &m_pop_buf.ids[crossover_point])
                m_pop_buf.ids[crossover_point++] = p2.ids[i];
            ++i;
        }

        m_pop_buf.cost = calculate_cost(m_pop_buf.ids);
    }
    void mutate() noexcept{
        if (static_cast<double>(m_rand() % 100) < M_MUTATION_RATIO * 100.0){
            std::swap(m_pop_buf.ids[m_rand() % m_coors.size()], m_pop_buf.ids[m_rand() % m_coors.size()]);
            m_pop_buf.cost = calculate_cost(m_pop_buf.ids);
        }
    }

    void timer_callback(){
        crossover();
        mutate();

        if (m_pop_buf.cost < m_pops[0].cost){
            std::copy(&m_pop_buf.ids[0], &m_pop_buf.ids[m_coors.size()], &m_pops[0].ids[0]);
            m_pops[0].cost = m_pop_buf.cost;
        }

        size_t population_size_half = m_population_size / 2;
        std::swap(m_pop_buf, m_pops[population_size_half + m_rand() % population_size_half]);
        std::shuffle(&m_pop_buf.ids[0], &m_pop_buf.ids[m_coors.size()], m_rand);
        m_pop_buf.cost = calculate_cost(m_pop_buf.ids);

        if (m_pop_buf.cost < m_pops[0].cost){
            std::copy(&m_pop_buf.ids[0], &m_pop_buf.ids[m_coors.size()], &m_pops[0].ids[0]);
            m_pops[0].cost = m_pop_buf.cost;
        }

        std::swap(m_pop_buf, m_pops[population_size_half + m_rand() % population_size_half]);
        std::shuffle(&m_pop_buf.ids[0], &m_pop_buf.ids[m_coors.size()], m_rand);
        m_pop_buf.cost = calculate_cost(m_pop_buf.ids);

        std::sort(&m_pops[0], &m_pops[m_population_size], [](const Population &p1, const Population &p2){ return p1.cost < p2.cost; });

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker spheres, lines;

        spheres.header.frame_id = "map";
        spheres.ns = "spheres";
        spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        spheres.id = 0;
        spheres.action = visualization_msgs::msg::Marker::ADD;
        spheres.pose.orientation.x = 0.0;
        spheres.pose.orientation.y = 0.0;
        spheres.pose.orientation.z = 0.0;
        spheres.pose.orientation.w = 0.0;

        spheres.scale.x = 0.2;
        spheres.scale.y = 0.2;
        spheres.scale.z = 0.2;
        
        spheres.color.r = 1.0f;
        spheres.color.g = 0.0f;
        spheres.color.b = 0.0f;
        spheres.color.a = 1.0f;

        lines.header.frame_id = "map";
        lines.ns = "lines";
        lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lines.id = 1;
        lines.action = visualization_msgs::msg::Marker::ADD;
        lines.pose.orientation.x = 0.0;
        lines.pose.orientation.y = 0.0;
        lines.pose.orientation.z = 0.0;
        lines.pose.orientation.w = 0.0;

        lines.scale.x = 0.1;
        
        lines.color.r = 0.0f;
        lines.color.g = 0.0f;
        lines.color.b = 1.0f;
        lines.color.a = 1.0f;

        geometry_msgs::msg::Point p;
        p.z = 0.0;
        for (auto it = &m_pops[0].ids[0]; it != &m_pops[0].ids[m_coors.size()]; ++it){
            p.x = m_coors[*it].x;
            p.y = m_coors[*it].y;

            spheres.points.push_back(p);
            lines.points.push_back(p);
        }

        p.x = m_coors[m_pops[0].ids[0]].x;
        p.y = m_coors[m_pops[0].ids[0]].y;
        lines.points.push_back(p);

        markers.markers.push_back(spheres);
        markers.markers.push_back(lines);

        // for (size_t i = 0; i < m_population_size; ++i)
            // RCLCPP_INFO(this->get_logger(), "%lf", m_pops[0].cost);

        m_publisher->publish(markers);
    }

    public:
    Solver() : Node{"tsp_node"}{
        using namespace std::chrono_literals;

        this->declare_parameter("file_name", "");
        this->get_parameter("file_name", m_file_name);

        read_file();
        calculate_distances();

        this->declare_parameter("population_size", static_cast<int64_t>(m_coors.size()));
        this->get_parameter("population_size", m_population_size);

        std::random_device rd;
        m_rand.seed(rd());

        create_population();
        m_pop_buf.ids = std::make_unique<size_t[]>(m_coors.size());

        m_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array_topic", 10);
        m_timer = this->create_wall_timer(1ns, std::bind(&Solver::timer_callback, this));
    }
};

int main(const int argc, const char *const *const argv){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Solver>());
    rclcpp::shutdown();

    return 0;
}