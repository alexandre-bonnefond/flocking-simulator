//
// Created by colin on 02/06/2021.
//

#ifndef PIR_SUJE2_KARGER_H
#define PIR_SUJE2_KARGER_H

#endif //PIR_SUJE2_KARGER_H

#define N_karger 5

//int lireFichier(FILE *fich, int mat[N_karger][N_karger]);
void affichage(int mat[N_karger][N_karger]);
int supprLigne(int mat[N_karger][N_karger], int ligne);
int merge(int mat[N_karger][N_karger], int ligne1, int ligne2);
int tailleMatrice(int mat[N_karger][N_karger]);
void choixRandom(int mat[N_karger][N_karger], int res[2]);
int coupeMin(int mat[N_karger][N_karger]);
int checkValide(int mat[N_karger][N_karger], int i);
int checkSym(int mat[N_karger][N_karger]);