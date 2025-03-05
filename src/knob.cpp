#include "Knob.h"
#include <algorithm>
    
Knob::Knob(int _no, int _minVal, int _maxVal, int _start) 
    : no(_no), min(_minVal), max(_maxVal), rotation(_start) {  // Member initializer list
    previousStateA = 0;
    previousStateB = 0;
}

int Knob::clamp(int r){
    return std::max(std::min(r, max), min);
}


void Knob::storeValue(int r){
    __atomic_store_n(&rotation, r, __ATOMIC_RELEASE);
}

void Knob::updateValues(int currentStateA, int currentStateB){
    if(previousStateB == currentStateB && previousStateA != currentStateA){ //if B stays the same and A flips
        if(previousStateB == previousStateA){ //if prev state B = A
            storeValue(clamp(rotation + 1));
            lastIncrement = 1;
        }
        else{
            storeValue(clamp(rotation - 1));
            lastIncrement = -1;
        }
    }
    else{   //If B flips
        if(previousStateA != currentStateA){ //if A flips
            storeValue(clamp(rotation + lastIncrement));  
        }
    }
    previousStateA = currentStateA;
    previousStateB = currentStateB;
}