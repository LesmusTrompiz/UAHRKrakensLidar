#ifndef POLAR_CONST

#define POLAR_CONST
#include <vector>


struct polar
{
    float mod;
    float alfa;
    //polar(void);
    //polar(float _mod, float _alfa);
    polar(void) : mod{0}, alfa{0} {}
    polar(float _mod,float _alfa) : mod{_mod}, alfa{_alfa} {}
};

//polar::polar(void) : mod{0}, alfa{0} {}
//polar::polar(float _mod,float _alfa) : mod{_mod}, alfa{_alfa} {}


typedef std::vector<polar> cluster;


#endif