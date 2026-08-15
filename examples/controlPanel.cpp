#include "spark_mmrt/device/SparkMax.hpp"
#include <ncurses.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <string>

#define GREEN_FOREGROUND_PAIR 2
#define RED_FOREGROUND_PAIR 4

#define HIGHLIGHT_VALIDITY(updated, x) if (has_colors())\
    {\
        if (updated) attron(COLOR_PAIR(GREEN_FOREGROUND_PAIR));\
        else         attron(COLOR_PAIR(RED_FOREGROUND_PAIR));\
    } (x);\

namespace {

volatile std::sig_atomic_t g_running = 1;

constexpr uint8_t defaultCanId = 1;
constexpr uint8_t maxCanId = 63;
constexpr int controlTypeCount = 8;
constexpr int sensorTypeCount = 5;
constexpr auto paramTimeout = std::chrono::milliseconds{150};
constexpr auto feedbackStaleAfter = std::chrono::milliseconds{500};
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

constexpr float unitRotations = 1.0f;
constexpr float unitRadians = 6.28318530718f; 
constexpr float unitDegrees = 360.0f;

const char* runModeLabel(int mode) {
    switch (mode) {
        case 0: return "duty";
        case 1: return "velocity";
        case 2: return "voltage";
        case 3: return "position";
        default: return "?";
    }
}

const char* controlTypeLabel(int type) {
    switch (type) {
        case 0: return "duty";
        case 1: return "velocity";
        case 2: return "voltage";
        case 3: return "position";
        case 4: return "smartmotion";
        case 5: return "smartvel";
        case 6: return "mm_pos";
        case 7: return "mm_vel";
        default: return "?";
    }
}

const char* sensorTypeLabel(int type) {
    switch (type) {
        case 0: return "none";
        case 1: return "main enc";
        case 2: return "analog";
        case 3: return "alt enc";
        case 4: return "duty enc";
        default: return "?";
    }
}

const char* unitFactorLabel(float factor) {
    if (std::fabs(factor - unitRotations) < 0.01f) return "rotations";
    if (std::fabs(factor - unitRadians) < 0.01f) return "radians";
    if (std::fabs(factor - unitDegrees) < 0.01f) return "degrees";
    return "rotations";
}

float cycleUnitFactor(float current, int direction) {
    const float presets[] = {unitRotations, unitRadians, unitDegrees};
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(current - presets[i]) < 0.01f) {
            index = i;
            break;
        }
    }
    index = (index + direction + 3) % 3;
    return presets[index];
}

} 

struct RunState {
    int mode = 0;
    float setpoint = 0.0f;
    uint8_t slot = 0;
    int active_field = 0;
};

struct PidfState {
    float p[4] = {};
    float i[4] = {};
    float d[4] = {};
    float f[4] = {};
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
static void applyPidfEdit(SparkMax& motor, PidfState& pidf, int field, uint8_t pidSlot) {
    if (field == 0) {
        motor.setP(pidf.p[pidSlot], pidSlot, paramTimeout);
    } else if (field == 1) {
        motor.setI(pidf.i[pidSlot], pidSlot, paramTimeout);
    } else if (field == 2) {
        motor.setD(pidf.d[pidSlot], pidSlot, paramTimeout);
    } else if (field == 3) {
        motor.setF(pidf.f[pidSlot], pidSlot, paramTimeout);
    }
}

static void applyAllPidf(SparkMax& motor, const PidfState& pidf, uint8_t pidSlot) {
    motor.setP(pidf.p[pidSlot], pidSlot, paramTimeout);
    motor.setI(pidf.i[pidSlot], pidSlot, paramTimeout);
    motor.setD(pidf.d[pidSlot], pidSlot, paramTimeout);
    motor.setF(pidf.f[pidSlot], pidSlot, paramTimeout);
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
            } else if (key.size() == 2 && key[1] >= '0' && key[1] <= '3') {
                const int s = key[1] - '0';
                if (key[0] == 'p') state.pidf.p[s] = std::stof(val);
                else if (key[0] == 'i') state.pidf.i[s] = std::stof(val);
                else if (key[0] == 'd') state.pidf.d[s] = std::stof(val);
                else if (key[0] == 'f') state.pidf.f[s] = std::stof(val);
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
        << "pid_slot=" << static_cast<int>(state.run.slot) << '\n';
    for (int s = 0; s < 4; ++s) {
        out << "p" << s << "=" << state.pidf.p[s] << '\n'
            << "i" << s << "=" << state.pidf.i[s] << '\n'
            << "d" << s << "=" << state.pidf.d[s] << '\n'
            << "f" << s << "=" << state.pidf.f[s] << '\n';
    }
    out << "control_type=" << state.config.control_type << '\n'
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

    UIState ui;
    RunState run;
    PidfState pidf;
    ConfigState config;
    EncoderState encoder;
    auto last_feedback_at = std::chrono::steady_clock::time_point{};
    bool have_feedback = false;
    bool heartbeat_on = false;

    PanelState saved;
        if (loadPanelState(saved)) {
        run = saved.run;
        pidf = saved.pidf;
        config = saved.config;
        encoder = saved.encoder;
        retargetMotor(motor, ui, saved.can_id);
            applyAllPidf(motor, pidf, run.slot);
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
        init_pair(4, COLOR_RED, COLOR_BLACK);
    }

    auto drawField = [](bool panel_focused, int active_field, int target_field, int y, int x, const char* format, auto value) {
        const bool highlight = panel_focused && (active_field == target_field);
        if (highlight) attron(A_REVERSE);
        mvprintw(y, x, format, value);
        if (highlight) attroff(A_REVERSE);
    };

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
            case ' ':
                heartbeat_on = !heartbeat_on;
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
                        run.mode = (run.mode + 1) % 4;
                    } else if (run.active_field == 1) {
                        run.setpoint += 0.05f;
                    } else if (run.active_field == 2) {
                        run.slot = (run.slot + 1) % 4;
                    } else if (run.active_field == 3 && ui.current_can_id < maxCanId) {
                        retargetMotor(motor, ui, static_cast<uint8_t>(ui.current_can_id + 1));
                        have_feedback = false; 
                    }
                } else if (ui.active_panel == Panel::Pidf) {
                    if (pidf.active_field == 0) pidf.p[run.slot] += 0.001f;
                    else if (pidf.active_field == 1) pidf.i[run.slot] += 0.001f;
                    else if (pidf.active_field == 2) pidf.d[run.slot] += 0.001f;
                    else if (pidf.active_field == 3) pidf.f[run.slot] += 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field, run.slot);
                } else if (ui.active_panel == Panel::Config) {
                    if (config.active_field == 0) {
                        config.control_type = (config.control_type + 1) % controlTypeCount;
                    } else {
                        config.sensor_type = (config.sensor_type + 1) % sensorTypeCount;
                    }
                    applyConfigEdit(motor, config, config.active_field);
                } else {
                    if (encoder.active_field == 0) {
                        encoder.position_factor = cycleUnitFactor(encoder.position_factor, 1);
                    } else if (encoder.active_field == 1) {
                        encoder.velocity_factor = cycleUnitFactor(encoder.velocity_factor, 1);
                    } else {
                        encoder.zero_offset += 0.05f;
                    }
                    applyEncoderEdit(motor, encoder, encoder.active_field);
                }
                break;
            case '-':
            case '_':
                if (ui.active_panel == Panel::Run) {
                    if (run.active_field == 0) {
                        run.mode = (run.mode + 3) % 4;
                    } else if (run.active_field == 1) {
                        run.setpoint -= 0.05f;
                    } else if (run.active_field) {
                        run.slot = (run.slot + 3) % 4;
                    } else if (run.active_field == 3 && ui.current_can_id > 0) {
                        retargetMotor(motor, ui, static_cast<uint8_t>(ui.current_can_id - 1));
                        have_feedback = false;
                    }
                } else if (ui.active_panel == Panel::Pidf) {
                    if (pidf.active_field == 0) pidf.p[run.slot] -= 0.001f;
                    else if (pidf.active_field == 1) pidf.i[run.slot] -= 0.001f;
                    else if (pidf.active_field == 2) pidf.d[run.slot] -= 0.001f;
                    else if (pidf.active_field == 3) pidf.f[run.slot] -= 0.001f;
                    applyPidfEdit(motor, pidf, pidf.active_field, run.slot);
                } else if (ui.active_panel == Panel::Config) {
                    if (config.active_field == 0) {
                        config.control_type = (config.control_type + controlTypeCount - 1) % controlTypeCount;
                    } else {
                        config.sensor_type = (config.sensor_type + sensorTypeCount - 1) % sensorTypeCount;
                    }
                    applyConfigEdit(motor, config, config.active_field);
                } else {
                    if (encoder.active_field == 0) {
                        encoder.position_factor = cycleUnitFactor(encoder.position_factor, -1);
                    } else if (encoder.active_field == 1) {
                        encoder.velocity_factor = cycleUnitFactor(encoder.velocity_factor, -1);
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
                if (motor.processFrame(frame) >= 0) {
                    last_feedback_at = std::chrono::steady_clock::now();
                    have_feedback = true;
                }
            }
        }
        if (heartbeat_on) {
            motor.heartbeat();
        }

        if (run.mode == 0) {
            motor.setDutyCycle(run.setpoint, run.slot);
        } else if (run.mode == 1) {
            motor.setVelocity(run.setpoint, run.slot);
        } else if (run.mode == 2) {
            motor.setVoltage(run.setpoint, run.slot);
        } else if (run.mode == 3) {
            motor.setPosition(run.setpoint, run.slot);
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
        mvprintw(2, 2, "CAN ID: %d | Press space to %s | Press 'q' to quit | arrows to navigate | +/- edit", motor.getID(), heartbeat_on ? "stall" : "run");

        const bool run_focus = (ui.active_panel == Panel::Run);
        const bool pidf_focus = (ui.active_panel == Panel::Pidf);
        const bool config_focus = (ui.active_panel == Panel::Config);
        const bool encoder_focus = (ui.active_panel == Panel::Encoder);

        int left_center = (mid_x / 2) - 7;
        int right_center = mid_x + ((w - mid_x) / 2) - 8;

        // Top-left: Run
        if (run_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(top_title_y, left_center, " - Run panel - ");
        if (run_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        char modeBuf[32];
        char controlTypeBuf[32];
        char sensorTypeBuf[32];
        char posFactorBuf[40];
        char velFactorBuf[40];

        std::snprintf(modeBuf, sizeof(modeBuf), "%s (%d)", runModeLabel(run.mode), run.mode);
        std::snprintf(controlTypeBuf, sizeof(controlTypeBuf), "%s (%d)", controlTypeLabel(config.control_type), config.control_type);
        std::snprintf(sensorTypeBuf, sizeof(sensorTypeBuf), "%s (%d)", sensorTypeLabel(config.sensor_type), config.sensor_type);
        std::snprintf(posFactorBuf, sizeof(posFactorBuf), "%s (%.2f)", unitFactorLabel(encoder.position_factor), encoder.position_factor);
        std::snprintf(velFactorBuf, sizeof(velFactorBuf), "%s (%.2f)", unitFactorLabel(encoder.velocity_factor), encoder.velocity_factor);

        drawField(run_focus, run.active_field, 0, top_field_y,     4, "Control Mode: %s", modeBuf);
        drawField(run_focus, run.active_field, 1, top_field_y + 1, 4, "Setpoint:     %.2f", run.setpoint);
        drawField(run_focus, run.active_field, 2, top_field_y + 2, 4, "PID Slot:     %d", run.slot);
        drawField(run_focus, run.active_field, 3, top_field_y + 3, 4, "Device CAN ID: %d", ui.current_can_id);

        // Top-right: PIDF
        if (pidf_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(top_title_y, right_center, " - PIDF tuning - ");
        if (pidf_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(pidf_focus, pidf.active_field, 0, top_field_y,     mid_x + 4, "P: %.4f", pidf.p[run.slot]);
        drawField(pidf_focus, pidf.active_field, 1, top_field_y + 1, mid_x + 4, "I: %.4f", pidf.i[run.slot]);
        drawField(pidf_focus, pidf.active_field, 2, top_field_y + 2, mid_x + 4, "D: %.4f", pidf.d[run.slot]);
        drawField(pidf_focus, pidf.active_field, 3, top_field_y + 3, mid_x + 4, "F: %.4f", pidf.f[run.slot]);

        // Bottom-left: Config
        if (config_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(bot_title_y, left_center, " - Config panel - ");
        if (config_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(config_focus, config.active_field, 0, bot_field_y,     4, "Control Type: %s", controlTypeBuf);
        drawField(config_focus, config.active_field, 1, bot_field_y + 1, 4, "Sensor Type:  %s", sensorTypeBuf);

        // Bottom-right: Encoder
        if (encoder_focus) attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(bot_title_y, right_center, " - Encoder panel - ");
        if (encoder_focus) attroff(COLOR_PAIR(1) | A_BOLD);

        drawField(encoder_focus, encoder.active_field, 0, bot_field_y,     mid_x + 4, "Abs Pos Factor: %s", posFactorBuf);
        drawField(encoder_focus, encoder.active_field, 1, bot_field_y + 1, mid_x + 4, "Abs Vel Factor: %s", velFactorBuf);
        drawField(encoder_focus, encoder.active_field, 2, bot_field_y + 2, mid_x + 4, "Zero Offset:    %.3f", encoder.zero_offset);

        const auto& s0 = motor.getStatus0();
        const auto& s2 = motor.getStatus2();
        const auto& s5 = motor.getStatus5();
        const bool feedback_fresh = have_feedback && ((std::chrono::steady_clock::now() - last_feedback_at) < feedbackStaleAfter);

        attron(COLOR_PAIR(3) | A_BOLD);
        mvprintw(feedback_y + 1, 2, " Live feedback ");
        attroff(COLOR_PAIR(3) | A_BOLD);

        HIGHLIGHT_VALIDITY(feedback_fresh,
            mvprintw(feedback_y + 1, 18, "Pos %.3f | Vel %.3f RPM | Curr %.3f A | Volt %.3f V  [%s]", s2.primaryEncoderPosition, s2.primaryEncoderVelocity, s0.current, s0.voltage, feedback_fresh ? "live" : "stale"));
        HIGHLIGHT_VALIDITY(feedback_fresh,
            mvprintw(feedback_y + 2, 18, "DC Pos %.3f | DC Vel %.3f", s5.dutyCycleEncPosition, s5.dutyCycleEncVelocity));
        if (has_colors()) {
            attroff(COLOR_PAIR(RED_FOREGROUND_PAIR));
            attroff(COLOR_PAIR(GREEN_FOREGROUND_PAIR));
        }

        refresh();

    }
    endwin();
    PanelState final_state{run, pidf, config, encoder, ui.current_can_id};
    savePanelState(final_state);
    return 0;
}