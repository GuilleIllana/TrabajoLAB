#include "casilla.h"

Casilla::Casilla(int row, int col){
    _row = row;
    _col = col;
    _avail = true;
    _dist = 0;
    _expl = false;
}


int Casilla::getRow() {
  return _row;
}


int Casilla::getCol() {
  return _col;
}


bool Casilla::getAvail() {
  return _avail;
}


float Casilla::getDist() {
  return _dist;
}


bool Casilla::getExpl() {
  return _expl;
}


void Casilla::setAvail(bool avail) {
  _avail = avail;
}


void Casilla::setDist(float dist) {
  _dist = dist;
}


void Casilla::setExpl(bool expl) {
  _expl = expl;
}
