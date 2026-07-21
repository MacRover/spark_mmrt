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
constexpr int controlTypeCount = 8;
constexpr int sensorTypeCount = 5;
constexpr auto paramTimeout = std::chrono::milliseconds{150};
constexpr const char* statePath = "config/control_panel_state";

void onSignal(int) {
    g_running = 0;
}

} 

enum class Panel {
    Run = 0,
    Pidf = 1,
    Config = 2,
    Encoder = 3
};

namespace {

Panel cyclePanel(Panel panel, int direction) {
    int next = (static_cast<int>(panel) + direction + 4) % 4;
    return static_cast<Panel>(next);
}

int fieldCount(Panel panel) {
    switch (panel) {
        case Panel::Run: return 4;
        case Panel::Pidf: return 4;
        case Panel::Config: return 2;
        case Panel::Encoder: return 3;
    }
    return 0;
}

} 

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
    ConfigState config;
    EncoderState encoder;
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

static void applyConfigEdit(SparkMax& motor, ConfigState& config, int field) {
    if (field == 0) {
        motor.setControlType(static_cast<ControlType>(config.control_type), paramTimeout);
    } else if (field == 1) {
        motor.setSensorType(static_cast<SensorType>(config.sensor_type), paramTimeout);
    }
}

static void applyAllConfig(SparkMax& motor, const ConfigState& config) {
    motor.setControlType(static_cast<ControlType>(config.control_type), paramTimeout);
    motor.setSensorType(static_cast<SensorType>(config.sensor_type), paramTimeout);
}

static void applyEncoderEdit(SparkMax& motor, EncoderState& encoder, int field) {
    if (field == 0) {
        motor.setDutyCyclePosConversionFactor(encoder.position_factor, paramTimeout);
    } else if (field == 1) {
        motor.setDutyCycleVelConversionFactor(encoder.velocity_factor, paramTimeout);
    } else if (field == 2) {
        motor.setEncoderPosition(encoder.zero_offset);
    }
}

static void applyAllEncoder(SparkMax& motor, const EncoderState& encoder) {
    motor.setDutyCyclePosConversionFactor(encoder.position_factor, paramTimeout);
    motor.setDutyCycleVelConversionFactor(encoder.velocity_factor, paramTimeout);
    motor.setEncoderPosition(encoder.zero_offset);
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
            } else if (key == "control_type") {
                state.config.control_type = std::stoi(val);
            } else if (key == "sensor_type") {
                state.config.sensor_type = std::stoi(val);
            } else if (key == "position_factor") {
                state.encoder.position_factor = std::stof(val);
            } else if (key == "velocity_factor") {
                state.encoder.velocity_factor = std::stof(val);
            } else if (key == "zero_offset") {
                state.encoder.zero_offset = std::stof(val);
            }
        } catch (...) {
            continue;
        }
    }

    return true;
}

static void savePanelState(const PanelState& state) {
    std::ofstream out(statePath);
    if (!out) {
        return;
    }

    out << "can_id=" << static_cast<int>(state.can_id) << '\n'
        << "mode=" << state.run.mode << '\n'
        << "setpoint=" << state.run.setpoint << '\n'
        << "pid_slot=" << static_cast<int>(state.run.slot) << '\n'
        << "p=" << state.pidf.p << '\n'
        << "i=" << state.pidf.i << '\n'
        << "d=" << state.pidf.d << '\n'
        << "f=" << state.pidf.f << '\n'
        << "control_type=" << state.config.control_type << '\n'
        << "sensor_type=" << state.config.sensor_type << '\n'
        << "position_factor=" << state.encoder.position_factor << '\n'
        << "velocity_factor=" << state.encoder.velocity_factor << '\n'
        << "zero_offset=" << state.encoder.zero_offset << '\n';
}

static void bumpField(int& field, int delta, Panel panel) {
    const int count = fieldCount(panel);
    field = (field + delta + count) % count;
}

// Helper to get a reference for the active field index for the current panel
static int& getActiveField(Panel panel, RunState& run, PidfState& pidf, ConfigState& config, EncoderState& encoder) {
    switch (panel) {
        case Panel::Run:     return run.active_field;
        case Panel::Pidf:    return pidf.active_field;
        case Panel::Config:  return config.active_field;
        case Panel::Encoder: return encoder.active_field;
    }
    return run.active_field;
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

    PanelState saved;
    if (loadPanelState(saved)) {
        run = saved.run;
        pidf = saved.pidf;
        config = saved.config;
        encoder = saved.encoder;
        retargetMotor(motor, ui, saved.can_id);
        applyAllPidf(motor, pidf);
        applyAllConfig(motor, config);
        applyAllEncoder(motor, encoder);
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

        // toggling between different menus
        switch (ch) {
            case 'q':
            case 'Q':
                ui.is_running = false;
                break;
            case KEY_RIGHT:
                ui.active_panel = cyclePanel(ui.active_panel, 1);
                break;
            case KEY_LEFT:
                ui.active_panel = cyclePanel(ui.active_panel, -1);
                break;
            case KEY_UP:
                bumpField(getActiveField(ui.active_panel, run, pidf, config, encoder), -1, ui.active_panel);
                break;
            case KEY_DOWN:
                bumpField(getActiveField(ui.active_panel, run, pidf, config, encoder), 1, ui.active_panel);
                break;
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
                } else if (ui.active_panel == Panel::Pidf) {
                    if (pidf.active_field == 0) pidf.p += 0.001f;
                    else if (pidf.active_field == 1) pidf.i += 0.001f;
                    else if (pidf.active_field == 2) pidf.d += 0.001f;
                    else if (pidf.active_field == 3) pidf.f += 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field);
                } else if (ui.active_panel == Panel::Config) {
                    if (config.active_field == 0) {
                        config.control_type = (config.control_type + 1) % controlTypeCount;
                    } else {
                        config.sensor_type = (config.sensor_type + 1) % sensorTypeCount;
                    }
                    applyConfigEdit(motor, config, config.active_field);
                } else {
                    if (encoder.active_field == 0) encoder.position_factor += 1.0f;
                    else if (encoder.active_field == 1) encoder.velocity_factor += 1.0f;
                    else encoder.zero_offset += 0.05f;
                    applyEncoderEdit(motor, encoder, encoder.active_field);
                }
                break;
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
                } else if (ui.active_panel == Panel::Pidf) {
                    if (pidf.active_field == 0) pidf.p -= 0.001f;
                    else if (pidf.active_field == 1) pidf.i -= 0.001f;
                    else if (pidf.active_field == 2) pidf.d -= 0.001f;
                    else if (pidf.active_field == 3) pidf.f -= 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field);
                } else if (ui.active_panel == Panel::Config) {
                    if (config.active_field == 0) {
                        config.control_type = (config.control_type + controlTypeCount - 1) % controlTypeCount;
                    } else {
                        config.sensor_type = (config.sensor_type + sensorTypeCount - 1) % sensorTypeCount;
                    }
                    applyConfigEdit(motor, config, config.active_field);
                } else {
                    if (encoder.active_field == 0 && encoder.position_factor > 0.0f) {
                        encoder.position_factor -= 1.0f;
                    } else if (encoder.active_field == 1 && encoder.velocity_factor > 0.0f) {
                        encoder.velocity_factor -= 1.0f;
                    } else {
                        encoder.zero_offset -= 0.05f;
                    }
                    applyEncoderEdit(motor, encoder, encoder.active_field);
                }
                break;
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
            motor.setDutyCycle(run.setpoint, run.slot);
        } else if (run.mode == 1) {
            motor.setVelocity(run.setpoint, run.slot);
        } else if (run.mode == 2) {
            motor.setVoltage(run.setpoint, run.slot);
        }

        const int feedback_y = (h > 18) ? (h - 4) : (h - 3);
        mid_y = 4 + (feedback_y - 4) / 2;
        if (mid_y < 10) {
            mid_y = 10;
        }
        if (mid_y >= feedback_y - 3) {
            mid_y = feedback_y - 4;
        }

        const int top_title_y = 5;
        const int top_field_y = 7;
        const int bot_title_y = mid_y + 1;
        const int bot_field_y = mid_y + 3;

        werase(stdscr);
        box(stdscr, 0, 0);

        mvhline(4, 1, ACS_HLINE, w - 2);
        mvhline(mid_y, 1, ACS_HLINE, w - 2);
        mvhline(feedback_y, 1, ACS_HLINE, w - 2);
        mvvline(5, mid_x, ACS_VLINE, feedback_y - 5);

        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(0, 2, " SparkMAX control panel ");
        attroff(COLOR_PAIR(1) | A_BOLD);
        mvprintw(2, 2, "CAN ID: %d | Press 'q' to quit | arrows to navigate | +/- edit", motor.getID());

        const bool run_focus = (ui.active_panel == Panel::Run);
        const bool pidf_focus = (ui.active_panel == Panel::Pidf);
        const bool config_focus = (ui.active_panel == Panel::Config);
        const bool encoder_focus = (ui.active_panel == Panel::Encoder);

        int left_center = (mid_x / 2) - 7;
        int right_center = mid_x + ((w - mid_x) / 2) - 8;

        // Top-left: Run
        auto drawField = [](bool panel_focused, int active_field, int target_field, int y, int x, const char* format, auto value) {
            bool highlight = panel_focused && (active_field == target_field);
            if (highlight) attron(A_REVERSE);
            mvprintw(y, x, format, value);
            if (highlight) attroff(A_REVERSE);
        };

        // Top-left: Run
        if (run_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(top_title_y, left_center, " - Run panel - ");
        if (run_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(run_focus, run.active_field, 0, top_field_y,     4, "Control Mode: %d", run.mode);
        drawField(run_focus, run.active_field, 1, top_field_y + 1, 4, "Setpoint:     %.2f", run.setpoint);
        drawField(run_focus, run.active_field, 2, top_field_y + 2, 4, "PID Slot:     %d", run.slot);
        drawField(run_focus, run.active_field, 3, top_field_y + 3, 4, "Device CAN ID: %d", ui.current_can_id);

        // Top-right: PIDF
        if (pidf_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(top_title_y, right_center, " - PIDF tuning - ");
        if (pidf_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(pidf_focus, pidf.active_field, 0, top_field_y,     mid_x + 4, "P: %.4f", pidf.p);
        drawField(pidf_focus, pidf.active_field, 1, top_field_y + 1, mid_x + 4, "I: %.4f", pidf.i);
        drawField(pidf_focus, pidf.active_field, 2, top_field_y + 2, mid_x + 4, "D: %.4f", pidf.d);
        drawField(pidf_focus, pidf.active_field, 3, top_field_y + 3, mid_x + 4, "F: %.4f", pidf.f);

        // Bottom-left: Config
        if (config_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(bot_title_y, left_center, " - Config panel - ");
        if (config_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(config_focus, config.active_field, 0, bot_field_y,     4, "Control Type: %d", config.control_type);
        drawField(config_focus, config.active_field, 1, bot_field_y + 1, 4, "Sensor Type:  %d", config.sensor_type);

        // Bottom-right: Encoder
        if (encoder_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(bot_title_y, right_center, " - Encoder panel - ");
        if (encoder_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(encoder_focus, encoder.active_field, 0, bot_field_y,     mid_x + 4, "Abs Pos Factor: %.2f", encoder.position_factor);
        drawField(encoder_focus, encoder.active_field, 1, bot_field_y + 1, mid_x + 4, "Abs Vel Factor: %.2f", encoder.velocity_factor);
        drawField(encoder_focus, encoder.active_field, 2, bot_field_y + 2, mid_x + 4, "Zero Offset:    %.3f", encoder.zero_offset);

        const auto& s0 = motor.getStatus0();
        const auto& s2 = motor.getStatus2();

        attron(COLOR_PAIR(3) | A_BOLD);
        mvprintw(feedback_y + 1, 2, " Live feedback ");
        attroff(COLOR_PAIR(3) | A_BOLD);

        attron(COLOR_PAIR(2));
        mvprintw(feedback_y + 1, 18, "Pos %.3f | Vel %.3f RPM | Curr %.3f A | Volt %.3f V", s2.primaryEncoderPosition, s2.primaryEncoderVelocity, s0.current, s0.voltage);
        attroff(COLOR_PAIR(2));

        refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }
    endwin();
    PanelState final_state{run, pidf, config, encoder, ui.current_can_id};
    savePanelState(final_state);
    return 0;
}