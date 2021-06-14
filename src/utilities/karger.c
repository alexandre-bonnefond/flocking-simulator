#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "karger.h"

int minCUT(int A[N_karger][N_karger]) {
    int couple[2];

    //affichage(A);
    //printf("\n");
    //printf("Tailles %d\n", tailleMatrice(A));
    //printf("Symétrique : %d\n", checkSym(A));

    if (checkSym(A)) {
        while (tailleMatrice(A) > 2) {
            choixRandom(A, couple);
            //printf("Choix random %d et %d\n", (couple[0]+1), (couple[1]+1));

            merge(A, couple[0], couple[1]);
            //affichage(A);

            //printf("Tailles %d\n", tailleMatrice(A));
            //printf("Symétrique : %d\n", checkSym(A));
        }
    } else {
        printf("Erreur matrice non symétrique\n");
    }

    //printf("La coupe minimale est de %d\n", coupeMin(A));
    return coupeMin(A);
}

void affichage(int mat[N_karger][N_karger]) {
    int i, j;
    for (i = 0; i < N_karger; i++) {
        for (j = 0; j < N_karger; j++) {
            printf("%d ", mat[i][j]);
        }
        printf("\n");
    }
}

int supprLigne(int mat[N_karger][N_karger], int ligne) {
    int i;
    for (i = 0; i < N_karger; i++) {
        mat[ligne][i] = 0;
        mat[i][ligne] = 0;
    }
    return (0);
}

int merge(int mat[N_karger][N_karger], int ligne1, int ligne2) {
    int i;
    for (i = 0; i < N_karger; i++) {
        if ((mat[ligne1][i] != 0) || (mat[ligne2][i] != 0)) {
            mat[ligne1][i] += mat[ligne2][i];
            mat[i][ligne1] += mat[i][ligne2];
        }
    }
    mat[ligne1][ligne1] = 0;
    mat[ligne1][ligne2] = 0;
    supprLigne(mat, ligne2);
    return (0);
}

int tailleMatrice(int mat[N_karger][N_karger]) {
    int i, j;
    int res = N_karger;
    for (i = 0; i < N_karger; i++) {
        int tmp = 0;
        for (j = 0; j < N_karger; j++) {
            if (mat[i][j] == 0) {
                tmp++;
            }
        }
        if (tmp == N_karger) {
            res--;
        }
    }
    return res;
}

int checkValide(int mat[N_karger][N_karger], int i) {
    int res = 0;
    for (int l = 0; l < N_karger; l++) {
        if (mat[i][l] == 0) {
            res++;
        }
    }
    if (res == N_karger) {
        return 0;
    }
    return 1;
}

void choixRandom(int mat[N_karger][N_karger], int res[2]) {
    int i, j;
    do {
        do {
            i = (rand() % N_karger);
        } while (!checkValide(mat, i) );

        do {
            j = (rand() % N_karger);
        } while (!(i != j && checkValide(mat, j)));
    } while (!mat[i][j] != 0);

    res[0] = i;
    res[1] = j;
}

int coupeMin(int mat[N_karger][N_karger]) {
    int i, j;
    for (i = 0; i < N_karger; i++) {
        for (j = 0; j < N_karger; j++) {
            if (mat[i][j] != 0) {
                return mat[i][j];
            }
        }
    }
    return 0;
}

int checkSym(int mat[N_karger][N_karger]) {
    int i, j;
    for (i = 0; i < N_karger; i++) {
        for (j = 0; j < N_karger; j++) {
            if (i != j && mat[i][j] != mat[j][i]) {
                return 0;
            }
        }
    }
    return 1;
}