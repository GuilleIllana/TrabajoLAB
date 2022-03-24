#include "cuadricula.h"
#include <QTRSensors.h>
#include "Arduino.h"

Cuadricula::Cuadricula(int rows, int cols, int obs_row[], int obs_col[], int nobs) {
    _rows = rows;
    _cols = cols;
    _nobs = nobs;
    Tablero = (Casilla*)malloc(rows * cols * sizeof(Casilla));
        
    // Construcción del tablero
    for (int i = 0; i < _rows; i++){
        for (int j = 0; j < _cols; j++){
          Tablero[i*_rows + j] = Casilla(i,j);
          Mat_ady[i][j] = 1;
        }
    }
    // Adición de obstáculos
    for (int i = 0; i < nobs; i++){
      Tablero[rows*obs_row[i]+obs_col[i]].setAvail(false);
      int hola=rows*obs_row[i]+obs_col[i];
      Serial.println(hola);
    }  
}


// A utility function to find the vertex with minimum distance value
int Cuadricula::minDistance(int n)  {
  // Inicialización de las posibles casillas
  int idx[] = {n-_rows, n+_rows, n+1, n-1};
  int idx_g[4];
  int count = 0;

  // Comprobación de cada casilla
  for (int i = 0; i < 4; i++) {
    int dif_r = abs(Tablero[idx[i]].getRow() - Tablero[n].getRow());
    int dif_c = abs(Tablero[idx[i]].getCol() - Tablero[n].getCol());
    int sum = dif_r + dif_c;
    if ((idx[i] < (_rows*_cols)) && ( idx[i] >= 0 ) && (sum == 1) && (Tablero[idx[i]].getAvail() == true)) {
      idx_g[count] = idx[i];
      count++;
    }
    else {
//      Serial.print(idx[i]);
//      Serial.println();
    }
  }
  
  // Initialize min value
  float val_min = Tablero[idx_g[0]].getDist();
  int min_index = idx_g[0];
  
  // Obtención del índice de la casilla con las distancia mínima
  for (int v = 0; v < count; v++) {
    if ((Tablero[idx_g[v]].getExpl() == false) && (Tablero[idx_g[v]].getDist() <= val_min) && (Tablero[idx_g[v]].getAvail() == true)) {
      val_min = Tablero[idx_g[v]].getDist();
      min_index = idx_g[v];
    }
  }
  return min_index;
}

int Cuadricula::Planner(int ro, int co, int rf, int cf, int Recorrido[]) {
  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation 
  int count = 0, idx_ant = ro*_rows+co;
  int idx = 0;

  for (int i = 0; i < (_rows*_cols); i++) {
    Recorrido[i] = 0;
  }
  // Initialize all distances and explore as false
  for (int i = 0; i < _rows; i++) {
    for (int j = 0; j < _cols; j++) {
      Tablero[i*_rows + j].setDist(sqrt(sq(rf-i)+sq(cf-j)));
      Tablero[i*_rows + j].setExpl(false);
      if ((i == 0) || (j == 0) || (i == (_rows-1)) || (j == (_cols-1)))
        Tablero[i*_rows + j].setAvail(false);

    }
  }
  Tablero[ro*_rows + co].setAvail(true);
  Tablero[rf*_rows + cf].setAvail(true); 
  
  Recorrido[0] = ro*_rows + co;
  count++;
  
  // Find shortest path for all vertices
  while (idx != (rf*_rows + cf)) {
    // Pick the minimum distance vertex from the set of vertices not
    // yet processed. u is always equal to src in the first iteration.
    idx = minDistance(idx_ant);
 
    // Mark the picked vertex as processed
    Tablero[idx].setExpl(true);
 
    Recorrido[count] = idx;
    count++;

    idx_ant = idx;
 }

 return count;
}


void Cuadricula::printTablero() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          Serial.print(Tablero[i*_rows + j].getAvail());
          Serial.print('\t');
        }
    Serial.println();
    }
}


void Cuadricula::printDistancia() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          Serial.print(Tablero[i*_rows + j].getDist());
          Serial.print('\t');
        }
    Serial.println();
    }
}

void Cuadricula::printAvail() {  
    // Calling printer
    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
          Serial.print(Tablero[i*_rows + j].getAvail());
          Serial.print('\t');
        }
    Serial.println();
    }
}


void Cuadricula::MovGenerator(int nR, int* Recorrido, int* Movimientos, int* Orientacion, int ori_ini) { // Movimientos: 0-siguelineas, 1-giro izquierda, 2-giro derecha 
  int ori; // 0-derecha, 1-arriba, 2-izquierda, 3-abajo;

  for (int i = 0; i < (_rows*_cols); i++) {
    Movimientos[i] = 0;
  }
  
  if (Recorrido[0] < _cols) {
    ori = 3;
  }
  else if ((Recorrido[0] % _cols) == 0) {
    ori = 0;
  }
  else if ((Recorrido[0] > (_cols*(_rows-1))) && (Recorrido[0] < (_rows*_cols))) {
    ori = 1;
  }
  else if (((Recorrido[0]+1) % _cols) == 0) {
    ori = 2;
  }
  else {
    ori = ori_ini;
  }
  Orientacion[0] = ori;
  Movimientos[0] = 0;
  for (int i = 1; i < nR; i++) {
    int difr = Tablero[Recorrido[i+1]].getRow() - Tablero[Recorrido[i]].getRow();
    int difc = Tablero[Recorrido[i+1]].getCol() - Tablero[Recorrido[i]].getCol();
    int dif = abs(difr) - abs(difc); // 1 - aumenta fila, -1 - aumenta columna
    switch (dif)  {
      case 1:
          if (difr > 0) {
            switch (ori) {
              case 0:
                Movimientos[i] = 2;
                break;
              case 1:
                Movimientos[i] = 3;
                break;
              case 2:
                Movimientos[i] = 1;
                break;
              case 3:
                Movimientos[i] = 0;
                break;
              default:
                Movimientos[i] = 0;
            }
            ori = 3;
          }
          else if (difr < 0) {
            switch (ori) {
              case 0:
                Movimientos[i] = 1;
                break;
              case 1:
                Movimientos[i] = 0;
                break;
              case 2:
                Movimientos[i] = 2;
                break;
              case 3:
                Movimientos[i] = 3;
                break;
              default:
                Movimientos[i] = 0;
            }
            ori = 1;
          }
          break;
      case -1:
          if (difc > 0) {
            switch (ori) {
              case 0:
                Movimientos[i] = 0;
                break;
              case 1:
                Movimientos[i] = 2;
                break;
              case 2:
                Movimientos[i] = 3;
                break;
              case 3:
                Movimientos[i] = 1;
                break;
              default:
                Movimientos[i] = 0;
            }
            ori = 0;
          }
          else if (difc < 0) {
            switch (ori) {
              case 0:
                Movimientos[i] = 3;
                break;
              case 1:
                Movimientos[i] = 1;
                break;
              case 2:
                Movimientos[i] = 0;
                break;
              case 3:
                Movimientos[i] = 2;
                break;
              default:
                Movimientos[i] = 0;
            }
            ori = 2;
          }
          break;
      default:
        Movimientos[i] = 0;
        break;
    }
    Orientacion[i] = ori;    
  }
  Movimientos[nR-1] = 4;
}


void Cuadricula::setObs(int idx_o, int idx_f) {
  int ro = Tablero[idx_o].getRow();
  int co = Tablero[idx_o].getCol();
  int rf = Tablero[idx_f].getRow();
  int cf = Tablero[idx_f].getCol();
  Mat_ady[ro][cf] = 0;
  Mat_ady[rf][co] = 0;
}
  
