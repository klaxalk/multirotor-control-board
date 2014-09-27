#ifndef COMMANDS_H
#define COMMANDS_H

/*
    TODO wtf? kde...
    #if PX4FLOW_DATA_RECEIVE == ENABLED
        //px4flow values
        extern volatile float groundDistance;
        extern volatile float elevatorSpeed;
        extern volatile float aileronSpeed;
        char groundDistanceTelemetry(char *address64, char *address16,char options);
    #endif // PX4FLOW_DATA_RECEIVE
*/
char groundDistanceTelemetry(char *address64, char *address16,char options);
char groundDistanceRequest(char *address64, char *address16);

#endif /*COMMANDS_H*/
