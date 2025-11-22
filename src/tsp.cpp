#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Solver : public rclcpp::Node{
    private:
    using Vec2 = std::pair<double, double>;
    using Population = std::pair<std::unique_ptr<size_t[]>, double>;

    std::string m_file_name;
    std::vector<Vec2> m_coors;
    std::unique_ptr<double[]> m_distances;
    size_t m_population_size;
    std::unique_ptr<Population[]> m_pops;
    Population m_buf;

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
            m_coors.emplace_back(x, y);
    }
    void calculate_distances(){
        m_distances = std::make_unique<double[]>(m_coors.size() * m_coors.size());

        double *it = m_distances.get();
        for (const Vec2 &coor1 : m_coors)
            for (const Vec2 &coor2 : m_coors)
                *it++ = std::hypot(coor2.first - coor1.first, coor2.second - coor1.second);
    }

    void timer_callback(){

    }

    public:
    Solver() : Node{"tsp_node"}{
        using namespace std::chrono_literals;

        this->declare_parameter("file_name", "/home/giganibba840/ros2_ws/src/niz_xvn_tsp/csv/test.csv");
        this->get_parameter("file_name", m_file_name);

        read_file();
        calculate_distances();
        
        this->declare_parameter("population_size", m_coors.size());
        this->get_parameter("population_size", m_population_size);

        m_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker_topic", 10);
        m_timer = this->create_wall_timer(500ms, std::bind(&Solver::timer_callback, this));
    }
};

int main(const int argc, const char *const *const argv){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Solver>());
    rclcpp::shutdown();

    return 0;
}