// Nodo propio de alto nivel para el G1: comprueba conexion, (opcional) Start,
// camina a velocidad constante durante N segundos y se detiene.
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include <g1/g1_loco_client.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_stop{false};
static void OnSigint(int) { g_stop = true; }

class WalkNode : public rclcpp::Node {
 public:
  WalkNode() : Node("g1_walk_node"), client_(this) {
    vx_ = declare_parameter<double>("vx", 0.2);        // m/s hacia delante
    vy_ = declare_parameter<double>("vy", 0.0);        // m/s lateral
    wz_ = declare_parameter<double>("wz", 0.0);        // rad/s giro
    seconds_ = declare_parameter<double>("seconds", 3.0);
    do_start_ = declare_parameter<bool>("do_start", false);
    worker_ = std::thread([this] { Run(); });
  }
  ~WalkNode() override {
    if (worker_.joinable()) worker_.join();
  }
  bool finished() const { return finished_; }

 private:
  bool Ok(int32_t ret, const char *what) {
    if (ret == 0) return true;
    RCLCPP_ERROR(get_logger(), "%s fallo, codigo %d", what, ret);
    return false;
  }

  void Run() {
    std::this_thread::sleep_for(500ms);  // tiempo para el descubrimiento DDS

    // 1. Comprobar que hay alguien al otro lado (espera hasta 5 s).
    int fsm_id = -1;
    if (!Ok(client_.GetFsmId(fsm_id), "GetFsmId")) {
      RCLCPP_ERROR(get_logger(),
                   "Sin respuesta de /api/sport. En simulacion es lo esperado; "
                   "en el robot real, revisa red, ROS_DOMAIN_ID y que este encendido.");
      finished_ = true;
      return;
    }
    RCLCPP_INFO(get_logger(), "FSM actual: %d", fsm_id);

    // 2. Start solo si se pide expresamente.
    if (do_start_) {
      if (!Ok(client_.Start(), "Start")) {
        finished_ = true;
        return;
      }
      std::this_thread::sleep_for(2s);  // dejar que el robot cambie de modo
    }

    // 3. Caminar: se reenvia la velocidad a ~10 Hz con duracion corta (0.5 s),
    //    asi, si este nodo muere, el robot se detiene solo.
    const auto t_end = std::chrono::steady_clock::now() +
                       std::chrono::duration<double>(seconds_);
    while (!g_stop && std::chrono::steady_clock::now() < t_end) {
      if (!Ok(client_.SetVelocity(static_cast<float>(vx_),
                                  static_cast<float>(vy_),
                                  static_cast<float>(wz_), 0.5F),
              "SetVelocity")) {
        break;
      }
      std::this_thread::sleep_for(100ms);
    }

    // 4. Parar siempre al terminar, falle o no el bucle (y tambien con Ctrl+C).
    Ok(client_.StopMove(), "StopMove");
    RCLCPP_INFO(get_logger(), "Terminado: StopMove enviado.");
    finished_ = true;
  }

  unitree::robot::g1::LocoClient client_;
  double vx_{}, vy_{}, wz_{}, seconds_{};
  bool do_start_{};
  std::atomic<bool> finished_{false};
  std::thread worker_;
};

int main(int argc, char **argv) {
  // Manejamos Ctrl+C nosotros para poder enviar StopMove antes de cerrar.
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, OnSigint);
  auto node = std::make_shared<WalkNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (!node->finished()) exec.spin_some(10ms);
  rclcpp::shutdown();
  return 0;
}
