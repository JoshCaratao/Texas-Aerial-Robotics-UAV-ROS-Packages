
#ifndef PID_H
#define PID_H

class PID {
    
    public:
        PID(double p, double i, double d); //Constructor
        double calculate(double error, double time);

    private:

        double P;
        double I;
        double D;

        double totalPID;
        double lastError;
        double lastTime;

};

#endif