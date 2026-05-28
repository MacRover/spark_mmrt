#include <iostream>
#include <vector>
#include <ncurses.h>
#include "spark_mmrt/device/SparkMax.hpp"

#ifndef CAN_INTERFACE
    #define CAN_INTERFACE "vcan0"
#endif

#define WIN_LENGTH 80
#define WIN_WIDTH 28
#define SUBWIN_LENGTH 10

#define RED_PAIR 1
#define GREEN_PAIR 2

#define HIGHLIGHT_VALIDITY(updated, x) if (has_colors())\
    {\
        if (updated) wattron(main_win, COLOR_PAIR(GREEN_PAIR));\
        else         wattron(main_win, COLOR_PAIR(RED_PAIR));\
    } (x);\

void drawWindow(WINDOW *win, WINDOW *sub)
{
    box(win, 0, 0);
    box(sub, 0, 0);
    wattron(win, A_BOLD);
    mvwprintw(win, 0, WIN_LENGTH / 2 - 11, "SparkMAX Monitor Panel");
    wattroff(win, A_BOLD);
    mvwprintw(sub, 0, 2, "CAN-ID");
}

void printStatus0(WINDOW *win, const Status0& s0, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Applied Output: %.4f ", s0.appliedOutput);
    mvwprintw(win, ridx++, cidx, "Input Voltage: %.2f V", s0.voltage);
    mvwprintw(win, ridx++, cidx, "Current: %.2f A", s0.current);
    mvwprintw(win, ridx++, cidx, "Motor Temp: %d C", s0.motorTempC);
    mvwprintw(win, ridx++, cidx, "Motor Inverted?: %s", s0.inverted ? "YES" : "NO");
    ridx++;
}

void printStatus1(WINDOW *win, const Status1& s1, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Other Fault: %s", s1.otherFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "Motor Type Fault: %s", s1.motorTypeFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "Sensor Fault: %s", s1.sensorFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "CAN Fault: %s", s1.canFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "Temperature Fault: %s", s1.temperatureFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "DRV Fault: %s", s1.drvFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "ESC EEPROM Fault: %s", s1.escEepromFault ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "Firmware Fault: %s", s1.firmwareFault ? "YES" : "NO");
    ridx++;
}

void printStatus2(WINDOW *win, const Status2& s2, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Primary Encoder Pos: %.4f  ", s2.primaryEncoderPosition);
    mvwprintw(win, ridx++, cidx, "Primary Encoder Vel: %.4f ", s2.primaryEncoderVelocity);
    ridx++;
}

void printStatus3(WINDOW *win, const Status3& s3, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Analog Voltage: %.4f V", s3.analogVoltage);
    mvwprintw(win, ridx++, cidx, "Analog Position: %.4f ", s3.analogPosition);
    mvwprintw(win, ridx++, cidx, "Analog Velocity: %.4f ", s3.analogVelocity);
    ridx++;
}

void printStatus4(WINDOW *win, const Status4& s4, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Alt Encoder Pos: %.4f  ", s4.altEncoderPosition);
    mvwprintw(win, ridx++, cidx, "Alt Encoder Vel: %.4f ", s4.altEncoderVelocity);
    ridx++;
}

void printStatus5(WINDOW *win, const Status5& s5, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Duty Cycle Encoder Pos: %.4f ", s5.dutyCycleEncPosition);
    mvwprintw(win, ridx++, cidx, "Duty Cycle Encoder Vel: %.4f ", s5.dutyCycleEncVelocity);
    ridx++;
}

void printStatus8(WINDOW *win, const Status8& s8, uint8_t& ridx, uint8_t cidx)
{
    mvwprintw(win, ridx++, cidx, "Set Point: %.4f ", s8.setPoint);
    mvwprintw(win, ridx++, cidx, "At Set Point?: %s ", s8.isAtSetpoint ? "YES" : "NO");
    mvwprintw(win, ridx++, cidx, "Selected PID Slot: %d ", s8.selectedPIDSlot);
    ridx++;
}

int main(int argc, char *argv[]) {
    spark_mmrt::can::SocketCanTransport transport;
    if (argc > 1)
    {
        std::string interface = argv[1];
        transport.open(interface);
    }
    else
    {
        transport.open(CAN_INTERFACE);
    }

    uint8_t id_digits = 0;
    uint8_t can_id = 0;
    SparkMax sparkmax(transport, 0);

    initscr();
    cbreak();
    noecho();
    WINDOW * main_win = newwin(WIN_WIDTH, WIN_LENGTH, 1, 1);
    WINDOW * id_win = derwin(main_win, 3, SUBWIN_LENGTH, 1, 1);
    keypad(id_win, TRUE);
    nodelay(main_win, TRUE);
    nodelay(id_win, TRUE);
    mvprintw(0, 0, "Press 'q' to quit");
    refresh();

    if (has_colors())
    {
        start_color();
        use_default_colors();
        init_pair(RED_PAIR, COLOR_RED, -1);
        init_pair(GREEN_PAIR, COLOR_GREEN, -1);
    }

    std::vector<bool> updated(10, false);

    while (transport.isOpen())
    {
        werase(main_win);
        drawWindow(main_win, id_win);
        mvwprintw(id_win, 1, 4, "%d", can_id);
        int ch = wgetch(id_win);
        if (ch != ERR)
        {
            if (ch == 'q' || ch == 'Q')
            {
                break;
            }
            else if (id_digits < 2 && '0' <= ch && ch <= '9')
            {
                can_id = can_id * 10 + (ch - '0');
                updated.assign(updated.size(), false);
                id_digits++;
            }
            else if (id_digits > 0 && ch == KEY_BACKSPACE)
            {
                can_id = can_id / 10;
                updated.assign(updated.size(), false);
                id_digits--;
            }
        }

        auto f = transport.recv(std::chrono::microseconds{20000});
        if (f)
        {
            auto &frame = *f;
            uint8_t device = uint8_t(frame.arbId & 0x03F);
            if (device == can_id)
            {
                int ret = sparkmax.processFrame(frame);
                if (ret >= 0)
                {
                    updated[ret] = true;
                }
            }
        }

        Status0 s0 = sparkmax.getStatus0();
        Status1 s1 = sparkmax.getStatus1();
        Status2 s2 = sparkmax.getStatus2();
        Status3 s3 = sparkmax.getStatus3();
        Status4 s4 = sparkmax.getStatus4();
        Status5 s5 = sparkmax.getStatus5();
        Status8 s8 = sparkmax.getStatus8();

        uint8_t rowIdx = 4;
        uint8_t colIdx = 2;
        HIGHLIGHT_VALIDITY(updated[0], printStatus0(main_win, s0, rowIdx, colIdx));
        HIGHLIGHT_VALIDITY(updated[1], printStatus1(main_win, s1, rowIdx, colIdx));
        HIGHLIGHT_VALIDITY(updated[2], printStatus2(main_win, s2, rowIdx, colIdx));
        HIGHLIGHT_VALIDITY(updated[3], printStatus3(main_win, s3, rowIdx, colIdx));
        rowIdx = 4;
        colIdx = WIN_LENGTH / 2;
        HIGHLIGHT_VALIDITY(updated[4], printStatus4(main_win, s4, rowIdx, colIdx));
        HIGHLIGHT_VALIDITY(updated[5], printStatus5(main_win, s5, rowIdx, colIdx));
        HIGHLIGHT_VALIDITY(updated[8], printStatus8(main_win, s8, rowIdx, colIdx));

        if (has_colors())
        {
            wattroff(main_win, COLOR_PAIR(RED_PAIR));
            wattroff(main_win, COLOR_PAIR(GREEN_PAIR));
        }
        wrefresh(main_win);
    }

    endwin();
    return 0;
}