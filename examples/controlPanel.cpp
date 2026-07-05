#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/device/roboRIO.hpp"
#include <ncurses.h>
#include <chrono>
#include <thread>
#include <string>

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
    uint8_t slot = 0;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float f = 0.0f;
    int active_field = 0; 
};

struct UIState {
    Panel active_panel = Panel::Run;
    uint8_t current_can_id = 0;
    bool is_running = true;
};

int main(int argc, char* argv[]) {
    spark_mmrt::can::SocketCanTransport transport;
    std::string interface = (argc > 1) ? argv[1] : "vcan0";
    transport.open(interface);
    SparkMax motor(transport, 0); 
    RoboRIO rio(transport);

    UIState ui;
    RunState run;
    PidfState pidf;

    initscr();
    cbreak();              
    noecho();              
    keypad(stdscr, TRUE);  
    nodelay(stdscr, TRUE); 

    while (ui.is_running) {
        int h, w;
        getmaxyx(stdscr, h, w);
        int mid_y = h / 2;
        int mid_x = w / 2;

        int ch = getch();
        
        // toggling between Run and Pidf
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
                    run.active_field = (run.active_field + 2) % 3;
                }
                else {
                    pidf.active_field = (pidf.active_field + 3) % 4;
                } break;
            case KEY_DOWN:
                if (ui.active_panel == Panel::Run) {
                    run.active_field = (run.active_field + 1) % 3;
                }
                else {
                    pidf.active_field = (pidf.active_field + 1) % 4;
                } break;

            case '+':
            case '=':
                if (ui.active_panel == Panel::Run) {
                    if (run.active_field == 0) run.mode = (run.mode + 2) % 3;
                    else if (run.active_field == 1) run.setpoint +=0.05f;
                    else if (run.active_field == 2 && run.slot > 0) run.slot += 1;
                } else {
                    if (pidf.active_field == 0) pidf.p += 0.001f;
                    else if (pidf.active_field == 1) pidf.i += 0.001f;
                    else if (pidf.active_field == 2) pidf.d += 0.001f;
                    else if (pidf.active_field == 3) pidf.d += 0.001f;
                } break;

            case '-':
            case '_':
                if (ui.active_panel == Panel::Run) {
                    if (run.active_field == 0) run.mode = (run.mode + 2) % 3;
                    else if (run.active_field == 1) run.setpoint -= 0.05f;
                    else if (run.active_field == 2 && run.slot > 0) run.slot -= 1;
                } else {
                    if (pidf.active_field == 0) pidf.p -= 0.001f;
                    else if (pidf.active_field == 1) pidf.i -= 0.001f;
                    else if (pidf.active_field == 2) pidf.d -= 0.001f;
                    else if (pidf.active_field == 3) pidf.f -= 0.001f;
                } break;
            
        }

        werase(stdscr); 
        box(stdscr, 0, 0); 
        
        mvhline(4, 1, ACS_HLINE, w - 2); 
        mvhline(mid_y, 1, ACS_HLINE, w - 2);
        mvvline(5, mid_x, ACS_VLINE, mid_y - 5); 
    
        mvprintw(0, 2, " SparkMAX control panel ");
        mvprintw(2, 2, "CAN ID: %d | Press 'q' to quit | Left/Right arrows change tabs", ui.current_can_id);

        bool run_focus = (ui.active_panel == Panel::Run);

        if (run_focus) attron(A_REVERSE); 
        mvprintw(5, 2, " - Run panel - ");
        if (run_focus) attroff(A_REVERSE);

        if (run_focus && run.active_field == 0) attron(A_REVERSE);
        mvprintw(7, 4, "Control Mode: %d", run.mode);
        if (run_focus && run.active_field == 0) attroff(A_REVERSE);

        if (run_focus && run.active_field == 1) attron(A_REVERSE);
        mvprintw(8, 4, "Setpoint:     %.2f", run.setpoint);
        if (run_focus && run.active_field == 1) attroff(A_REVERSE);

        if (run_focus && run.active_field == 2) attron(A_REVERSE);
        mvprintw(9, 4, "PID Slot:     %d", run.slot);
        if (run_focus && run.active_field == 2) attroff(A_REVERSE);

        bool pidf_focus = (ui.active_panel == Panel::Pidf);

        if (pidf_focus) attron(A_REVERSE); 
        mvprintw(5, mid_x + 2, " - PIDF tuning - ");
        if (pidf_focus) attroff(A_REVERSE);

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

        mvprintw(mid_y + 1, 2, " - Live feedback - ");
        mvprintw(mid_y + 3, 4, "Position: 0.000");
        mvprintw(mid_y + 4, 4, "Velocity: 0.000 RPM");
        mvprintw(mid_y + 5, 4, "Current:  0.000 A");
        mvprintw(mid_y + 6, 4, "Voltage:  0.000 V");

        refresh();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    }
    endwin();
    return 0;
}