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

    void timer_callback(){
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
        m_timer = this->create_wall_timer(500ms, std::bind(&Solver::timer_callback, this));
    }
};

int main(const int argc, const char *const *const argv){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Solver>());
    rclcpp::shutdown();

    return 0;
}