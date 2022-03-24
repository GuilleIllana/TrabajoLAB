#ifndef _CASILLA_DISFUNCIONAL
#define _CASILLA_DISFUNCIONAL

#include "Arduino.h"
#include "robot.h"


class Casilla
{
  public:
    Casilla(int row, int col);
    int getRow();
    int getCol();
    bool getAvail();
    float getDist();
    bool getExpl();
    void setAvail(bool avail);
    void setDist(float dist);
    void setExpl(bool expl);
  private:
    int _row;
    int _col;
    float _dist;
    bool _avail;
    bool _expl;
};

#endif
