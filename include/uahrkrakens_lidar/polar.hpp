#ifndef POLAR

#define POLAR


struct polar
{
    float mod;
    float degrees;
    polar(void);
    polar(float _mod, float _degrees);
};

polar::polar(void) : mod{0}, degrees{0} {}
polar::polar(float _mod,float _degrees) : mod{_mod}, degrees{_degrees} {}


#endif