#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "karger.h"

int minCUT(int taille, int A[taille][taille]) {
    int couple[2];

    //fprintf(stdout, "A : \n");
    //affichage(taille, A);
    //fprintf(stdout, "Original : \n");
    //affichage(taille, original);

    if (checkSym(taille, A)) {
        while (tailleMatrice(taille, A) > 2) {
            choixRandom(taille, A, couple);
            merge(taille, A, couple[0], couple[1]);
        }
    } else {
        printf("Erreur matrice non symétrique\n");
        return 0;
    }

    return coupeMin(taille, A);
}

void affichage(int taille, int mat[taille][taille]) {
    for (int i = 0; i < taille; i++) {
        for (int j = 0; j < taille; j++) {
            fprintf(stdout, "%d ", mat[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void supprLigne(int taille, int mat[taille][taille], int ligne) {
    for (int i = 0; i < taille; i++) {
        mat[ligne][i] = 0;
        mat[i][ligne] = 0;
    }
}

void merge(int taille, int mat[taille][taille], int ligne1, int ligne2) {
    for (int i = 0; i < taille; i++) {
        if ((mat[ligne1][i] != 0) || (mat[ligne2][i] != 0)) {
            mat[ligne1][i] += mat[ligne2][i];
            mat[i][ligne1] += mat[i][ligne2];
        }
    }
    mat[ligne1][ligne1] = 0;
    mat[ligne1][ligne2] = 0;
    supprLigne(taille, mat, ligne2);
}

int tailleMatrice(int taille, int mat[taille][taille]) {
    int res = taille;
    for (int i = 0; i < taille; i++) {
        int tmp = 0;
        for (int j = 0; j < taille; j++) {
            if (mat[i][j] == 0) {
                tmp++;
            }
        }
        if (tmp == taille) {
            res--;
        }
    }
    return res;
}

int checkValide(int taille, int mat[taille][taille], int i) {
    int res = 0;
    for (int l = 0; l < taille; l++) {
        if (mat[i][l] == 0) {
            res++;
        }
    }
    if (res == taille) {
        return 0;
    }
    return 1;
}

void choixRandom(int taille, int mat[taille][taille], int res[2]) {
    int i, j;
    do {
        do {
            i = (rand() % taille);
        } while (!checkValide(taille, mat, i) );

        do {
            j = (rand() % taille);
        } while (!(i != j && checkValide(taille, mat, j)));
    } while (!mat[i][j] != 0);

    res[0] = i;
    res[1] = j;
}

int coupeMin(int taille, int mat[taille][taille]) {
    for (int i = 0; i < taille; i++) {
        for (int j = 0; j < taille; j++) {
            if (mat[i][j] != 0) {
                return mat[i][j];
            }
        }
    }
    return 0;
}

int checkSym(int taille, int mat[taille][taille]) {
    for (int i = 0; i < taille; i++) {
        for (int j = 0; j < taille; j++) {
            if (i != j && mat[i][j] != mat[j][i]) {
                return 0;
            }
        }
    }
    return 1;
}