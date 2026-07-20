#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/device/roboRIO.hpp"
#include <ncurses.h>
#include <chrono>
#include <csignal>
#include <fstream>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_running = 1;

constexpr uint8_t defaultCanId = 1;
constexpr uint8_t maxCanId = 63;
constexpr auto paramTimeout = std::chrono::milliseconds{150};
constexpr const char* statePath = "config/control_panel_state";

void onSignal(int) {
    g_running = 0;
}

} 

enum class Panel {
    Run = 0,
    Pidf = 1
};

struct RunState {
    int mode = 0;
    float setpoint = 0.0f;
    uint8_t slot = 0;
    int active_field = 0;
};

struct PidfState {
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float f = 0.0f;
    int active_field = 0;
};

struct ConfigState {
    int control_type = 0;
    int sensor_type = 0;
    int active_field = 0;
};

struct EncoderState {
    float position_factor = 1.0f;
    float velocity_factor = 1.0f;
    float zero_offset = 0.0f;
    int active_field = 0;
};

struct UIState {
    Panel active_panel = Panel::Run;
    uint8_t current_can_id = defaultCanId;
    bool is_running = true;
};

struct PanelState {
    RunState run;
    PidfState pidf;
    uint8_t can_id = defaultCanId;
};

static void retargetMotor(SparkMax& motor, UIState& ui, uint8_t new_id) {
    ui.current_can_id = new_id;
    motor.setInternalCANID(new_id);
}

// Writes the edited PIDF value to the corresponding SparkMAX parameter (P0/I0/D0/F0)
static void applyPidfEdit(SparkMax& motor, PidfState& pidf, int field) {
    if (field == 0) {
        motor.setP(pidf.p, paramTimeout);
    } else if (field == 1) {
        motor.setI(pidf.i, paramTimeout);
    } else if (field == 2) {
        motor.setD(pidf.d, paramTimeout);
    } else if (field == 3) {
        motor.setF(pidf.f, paramTimeout);
    }
}

static void applyAllPidf(SparkMax& motor, const PidfState& pidf) {
    motor.setP(pidf.p, paramTimeout);
    motor.setI(pidf.i, paramTimeout);
    motor.setD(pidf.d, paramTimeout);
    motor.setF(pidf.f, paramTimeout);
}

static bool loadPanelState(PanelState& state) {
    std::ifstream in(statePath);
    if (!in) {
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        const std::string key = line.substr(0, eq);
        const std::string val = line.substr(eq + 1);

        try {
            if (key == "can_id") {
                state.can_id = static_cast<uint8_t>(std::stoi(val));
            } else if (key == "mode") {
                state.run.mode = std::stoi(val);
            } else if (key == "setpoint") {
                state.run.setpoint = std::stof(val);
            } else if (key == "pid_slot") {
                state.run.slot = static_cast<uint8_t>(std::stoi(val));
            } else if (key == "p") {
                state.pidf.p = std::stof(val);
            } else if (key == "i") {
                state.pidf.i = std::stof(val);
            } else if (key == "d") {
                state.pidf.d = std::stof(val);
            } else if (key == "f") {
                state.pidf.f = std::stof(val);
            }
        } catch (...) {
            continue;
        }
    }

    return true;
}

static void savePanelState(const RunState& run, const PidfState& pidf, uint8_t can_id) {
    std::ofstream out(statePath);
    if (!out) {
        return;
    }

    out << "can_id=" << static_cast<int>(can_id) << '\n'
        << "mode=" << run.mode << '\n'
        << "setpoint=" << run.setpoint << '\n'
        << "pid_slot=" << static_cast<int>(run.slot) << '\n'
        << "p=" << pidf.p << '\n'
        << "i=" << pidf.i << '\n'
        << "d=" << pidf.d << '\n'
        << "f=" << pidf.f << '\n';
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    spark_mmrt::can::SocketCanTransport transport;
    std::string interface = (argc > 1) ? argv[1] : "vcan0";
    transport.open(interface);

    SparkMax motor(transport, defaultCanId);
    RoboRIO rio(transport);

    UIState ui;
    RunState run;
    PidfState pidf;
    ConfigState config;
    EncoderState encoder;

    (void)config;
    (void)encoder;

    PanelState saved;
    if (loadPanelState(saved)) {
        run = saved.run;
        pidf = saved.pidf;
        retargetMotor(motor, ui, saved.can_id);
        applyAllPidf(motor, pidf);
    }

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    if (has_colors() == TRUE) {
        start_color();
        init_pair(1, COLOR_CYAN, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    }

    while (ui.is_running && g_running) {
        int h, w;
        getmaxyx(stdscr, h, w);
        int mid_y = h / 2;
        int mid_x = w / 2;

        int ch = getch();
        
        // toggling between Run and PIDF
        switch (ch) {
            case 'q':
            case 'Q':
                ui.is_running = false;
                break;
            case KEY_RIGHT:
            case KEY_LEFT:
                ui.active_panel = (ui.active_panel == Panel::Run) ? Panel::Pidf : Panel::Run;
                break;
            case KEY_UP:
                if (ui.active_panel == Panel::Run) {
                    run.active_field = (run.active_field + 3) % 4;
                }
                else {
                    pidf.active_field = (pidf.active_field + 3) % 4;
                } break;
            case KEY_DOWN:
                if (ui.active_panel == Panel::Run) {
                    run.active_field = (run.active_field + 1) % 4;
                }
                else {
                    pidf.active_field = (pidf.active_field + 1) % 4;
                } break;
            case '+':
            case '=': // don't have to hold down shift
                if (ui.active_panel == Panel::Run) {
                    if (run.active_field == 0) {
                        run.mode = (run.mode + 1) % 3;
                    } else if (run.active_field == 1) {
                        run.setpoint += 0.05f;
                    } else if (run.active_field == 2) {
                        run.slot += 1;
                    } else if (run.active_field == 3 && ui.current_can_id < maxCanId) {
                        retargetMotor(motor, ui, static_cast<uint8_t>(ui.current_can_id + 1));
                    }
                } else {
                    if (pidf.active_field == 0) pidf.p += 0.001f;
                    else if (pidf.active_field == 1) pidf.i += 0.001f;
                    else if (pidf.active_field == 2) pidf.d += 0.001f;
                    else if (pidf.active_field == 3) pidf.f += 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field);
                } break;
            case '-':
            case '_':
                if (ui.active_panel == Panel::Run) {
                    if (run.active_field == 0) {
                        run.mode = (run.mode + 2) % 3;
                    } else if (run.active_field == 1) {
                        run.setpoint -= 0.05f;
                    } else if (run.active_field == 2 && run.slot > 0) {
                        run.slot -= 1;
                    } else if (run.active_field == 3 && ui.current_can_id > 0) {
                        retargetMotor(motor, ui, static_cast<uint8_t>(ui.current_can_id - 1));
                    }
                } else {
                    if (pidf.active_field == 0) pidf.p -= 0.001f;
                    else if (pidf.active_field == 1) pidf.i -= 0.001f;
                    else if (pidf.active_field == 2) pidf.d -= 0.001f;
                    else if (pidf.active_field == 3) pidf.f -= 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field);
                } break;
        }

        auto f = transport.recv(std::chrono::microseconds{20000});
        if (f) {
            auto& frame = *f;
            uint8_t device = uint8_t(frame.arbId & 0x03F);
            if (device == motor.getID()) {
                motor.processFrame(frame);
            }
        }

        rio.heartbeat();

        if (run.mode == 0) {
            motor.setDutyCycle(run.setpoint);
        } else if (run.mode == 1) {
            motor.setVelocity(run.setpoint);
        } else if (run.mode == 2) {
            motor.setVoltage(run.setpoint);
        }

        werase(stdscr);
        box(stdscr, 0, 0);

        mvhline(4, 1, ACS_HLINE, w - 2);
        mvhline(mid_y, 1, ACS_HLINE, w - 2);
        mvvline(5, mid_x, ACS_VLINE, mid_y - 5);

        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(0, 2, " SparkMAX control panel ");
        attroff(COLOR_PAIR(1) | A_BOLD);
        mvprintw(2, 2, "CAN ID: %d | Press 'q' to quit | arrows to navigate | +/- edit", motor.getID());

        int run_center = (mid_x/2) - 7;
        int pidf_center = mid_x + ((w - mid_x)/2) - 8; // '-8' is ~ half the the length of the string

        bool run_focus = (ui.active_panel == Panel::Run);

        if (run_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(5, run_center, " - Run panel - ");
        if (run_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        if (run_focus && run.active_field == 0) attron(A_REVERSE);
        mvprintw(7, 4, "Control Mode: %d", run.mode);
        if (run_focus && run.active_field == 0) attroff(A_REVERSE);

        if (run_focus && run.active_field == 1) attron(A_REVERSE);
        mvprintw(8, 4, "Setpoint:     %.2f", run.setpoint);
        if (run_focus && run.active_field == 1) attroff(A_REVERSE);

        if (run_focus && run.active_field == 2) attron(A_REVERSE);
        mvprintw(9, 4, "PID Slot:     %d", run.slot);
        if (run_focus && run.active_field == 2) attroff(A_REVERSE);

        if (run_focus && run.active_field == 3) attron(A_REVERSE);
        mvprintw(10, 4, "Device CAN ID: %d", ui.current_can_id);
        if (run_focus && run.active_field == 3) attroff(A_REVERSE);

        bool pidf_focus = (ui.active_panel == Panel::Pidf);

        if (pidf_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(5, pidf_center, " - PIDF tuning - ");
        if (pidf_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        if (pidf_focus && pidf.active_field == 0) attron(A_REVERSE);
        mvprintw(7, mid_x + 4, "P: %.4f", pidf.p);
        if (pidf_focus && pidf.active_field == 0) attroff(A_REVERSE);

        if (pidf_focus && pidf.active_field == 1) attron(A_REVERSE);
        mvprintw(8, mid_x + 4, "I: %.4f", pidf.i);
        if (pidf_focus && pidf.active_field == 1) attroff(A_REVERSE);

        if (pidf_focus && pidf.active_field == 2) attron(A_REVERSE);
        mvprintw(9, mid_x + 4, "D: %.4f", pidf.d);
        if (pidf_focus && pidf.active_field == 2) attroff(A_REVERSE);

        if (pidf_focus && pidf.active_field == 3) attron(A_REVERSE);
        mvprintw(10, mid_x + 4, "F: %.4f", pidf.f);
        if (pidf_focus && pidf.active_field == 3) attroff(A_REVERSE);

        const auto& s0 = motor.getStatus0();
        const auto& s2 = motor.getStatus2();

        attron(COLOR_PAIR(3) | A_BOLD);
        mvprintw(mid_y + 1, 2, " - Live feedback - ");
        attroff(COLOR_PAIR(3) | A_BOLD);

        attron(COLOR_PAIR(2));
        mvprintw(mid_y + 3, 4, "Position: %.3f", s2.primaryEncoderPosition);
        mvprintw(mid_y + 4, 4, "Velocity: %.3f RPM", s2.primaryEncoderVelocity);
        mvprintw(mid_y + 5, 4, "Current:  %.3f A", s0.current);
        mvprintw(mid_y + 6, 4, "Voltage:  %.3f V", s0.voltage);
        attroff(COLOR_PAIR(2));

        refresh();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }
    endwin();
    savePanelState(run, pidf, ui.current_can_id);
    return 0;
}