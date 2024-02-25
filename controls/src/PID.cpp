#include "controls/PID.h"

PID::PID(double p, double i, double d) : P(p), I(i), D(d), totalPID(0), lastTime(0){

    P = p;
    I = i;
    D = d;

    
    totalPID = 0;
    lastTime = 0;
    totalError = 0;

}

double PID::calculate(double error, double time){

    double elapsedTime = time - lastTime;
    double pTerm = P * error;
    double iTerm = 0;
    double dTerm = 0;
    
    if(elapsedTime > 0){
        dTerm = D*((error-lastError)/(elapsedTime));
    }

    if(error < 0.5 && error > 0.05){
        totalError += (((error + lastError)/2)*(elapsedTime));
        iTerm = I * totalError;
    }
    else{
        totalError = 0;
    }

    totalPID = pTerm + iTerm + dTerm;

    lastError = error;
    lastTime = time;

    return totalPID;

}