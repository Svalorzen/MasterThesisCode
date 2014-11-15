#ifndef MASTER_THESIS_SIGNALS_HEADER_FILE
#define MASTER_THESIS_SIGNALS_HEADER_FILE

#include <csignal>
#include <cstdio>
#include <iostream>

bool processInterrupted = false;

void interruptSetter(int) {
    std::cout << "Received SIGINT...\n";
    processInterrupted = true;
}

void registerSigInt() {
     struct sigaction sigIntHandler;

     sigIntHandler.sa_handler = interruptSetter;
     sigemptyset(&sigIntHandler.sa_mask);
     sigIntHandler.sa_flags = 0;

     sigaction(SIGINT, &sigIntHandler, NULL);
}

#endif
