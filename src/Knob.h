#ifndef KNOB_H
#define KNOB_H

class Knob{
    public:
        int rotation;
        int max; 
        int min; 
        int lastIncrement = 0;
        int previousStateA;
        int previousStateB;
        int no;

        Knob(int _no, int _minVal, int _maxVal, int _start);
        int clamp(int r);
        void storeValue(int r);        
        void updateValues(int currentStateA, int currentStateB);
};

#endif