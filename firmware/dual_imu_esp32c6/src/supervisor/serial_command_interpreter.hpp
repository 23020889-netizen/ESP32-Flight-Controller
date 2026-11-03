#pragma once

#include "flight_controller_manager.hpp"

/**
 * @brief Non-blocking command interpreter parsing serial console instructions.
 */
class SerialCommandInterpreter {
public:
    /**
     * @brief Checks and parses any pending serial command.
     * @param manager Reference to flight manager.
     */
    void processIncomingCommands(FlightControllerManager& manager);

private:
    void displayHelpInformation() const;
    void displayStatusInformation() const;
};

extern SerialCommandInterpreter commandInterpreter;

