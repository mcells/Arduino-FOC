#ifndef MULTI_FILTER_H
#define MULTI_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  Multi filter class
 *  Implements a state variable filter to compute low-, high-, and bandpass filters 
 *  with variable Q (resonance) in one operation, as well as seamless switching between them
 */
class MultiFilter
{
public:
    /**
     * @param Tf - Multi filter time constant
     * @param q - Filter resonance
     */
    MultiFilter(float Tf, float q=1.0f);
    ~MultiFilter() = default;

    enum returnType {
        MULTI_FILTER_LOWPASS,
        MULTI_FILTER_HIGHPASS,
        MULTI_FILTER_BANDPASS
    };

    float operator() (float x);
    float Tf; //!< Multi filter time constant

    void setQ(float newQ);  //!< Set filter resonance
    void setfrequency(float newfrequency); //!< set filter frequency instead of time constant
    void setReturnType(returnType type); //!< change between low-, high-, or bandpass output as default return value

    //!< Get different filter outputs
    float getLp();
    float getHp();
    float getBp();

    //!< Get different filter outputs while updating the filter state
    float getLp(float x);
    float getHp(float x);
    float getBp(float x);

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp

    float yl_prev; //!< filtered  lowpass value in previous execution step 
    float yh_prev; //!< filtered highpass value in previous execution step 
    float yb_prev; //!< filtered bandpass value in previous execution step 

    float q = 1.0f;       //!< filter resonance.

    float alpha1;
    float alpha2 = 1.0f/q;

    returnType defaultFilter = MULTI_FILTER_LOWPASS;
};

#endif // MULTI_FILTER_H