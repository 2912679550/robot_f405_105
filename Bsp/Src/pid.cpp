#include "pid.h"
#include "math.h"

#define USEI
#define USED
// #define USED

Pid::Pid(float p, float i, float d, float n, float ts, float outIMin, float outIMax, float Iband, float outMin, float outMax)
{
    this->p = p;
    this->i = i;
    this->d = d;
    this->n = n;
    this->ts = ts;
    this->accI = .0f;
    this->accD = .0f;
    this->accIMin = outIMin;
    this->accIMax = outIMax;
    this->Iband = Iband; // integral separated
    this->accDMin = outMin / n;
    this->accDMax = outMax / n;
    this->outMax = outMax;
    this->outMin = outMin;
}

Pid::Pid(PID_PARAM *param)
{
    this->p = param->p;
    this->i = param->i;
    this->d = param->d;
    this->n = param->n;
    this->ts = param->ts;
    this->accI = .0f;
    this->accD = .0f;
    this->accIMin = param->outIMin;
    this->accIMax = param->outIMax;
    this->Iband = param->Iband; // integral separated
    this->accDMin = param->outMin / n;
    this->accDMax = param->outMax / n;
    this->outMax = param->outMax;
    this->outMin = param->outMin;
}

void Pid::SetParam(float p, float i)
{
    this->p = p;
    this->i = i;
    this->accI = .0f;
    this->accD = .0f;
}

void Pid::Reset()
{
    this->accI = .0f;
    this->accD = .0f;
}

float Pid::Tick(float diff)
{
    diff_ = diff;
    float pout;
#ifdef USED
    float dout;
#endif

    pout = diff * p;

#ifdef USEI
    if (abs(diff) < Iband)
    {
        accI += diff * i * ts;
        if (accI > accIMax)
            accI = accIMax;
        else if (accI < accIMin)
            accI = accIMin;
    }
    else
    {
        accI = 0;
    }
#endif

#ifdef USED
    dout = (diff * d - accD) * n;
    accD += dout * ts;
    if (accD > accDMax)
        accD = accDMax;
    else if (accD < accDMin)
        accD = accDMin;
#endif

#ifdef USEI
    pout += accI;
#endif
#ifdef USED
    pout += dout;
#endif

    if (pout > outMax)
        pout = outMax;
    else if (pout < outMin)
        pout = outMin;

    return pout;
}
