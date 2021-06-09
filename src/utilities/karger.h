//
// Created by colin on 02/06/2021.
//

#ifndef PIR_SUJE2_KARGER_H
#define PIR_SUJE2_KARGER_H

#endif //PIR_SUJE2_KARGER_H

#define N Phase->N

int lireFichier(FILE *fich, int mat[N][N]);
void affichage(int mat[N][N]);
int supprLigne(int mat[N][N], int ligne);
int merge(int mat[N][N], int ligne1, int ligne2);
int tailleMatrice(int mat[N][N]);
void choixRandom(int mat[N][N], int res[2]);
int coupeMin(int mat[N][N]);
int checkValide(int mat[N][N], int i);
int checkSym(int mat[N][N]);