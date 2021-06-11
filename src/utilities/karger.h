#define N_karger 10

int minCUT(int A[N_karger][N_karger]);
void affichage(int mat[N_karger][N_karger]);
int supprLigne(int mat[N_karger][N_karger], int ligne);
int merge(int mat[N_karger][N_karger], int ligne1, int ligne2);
int tailleMatrice(int mat[N_karger][N_karger]);
void choixRandom(int mat[N_karger][N_karger], int res[2]);
int coupeMin(int mat[N_karger][N_karger]);
int checkValide(int mat[N_karger][N_karger], int i);
int checkSym(int mat[N_karger][N_karger]);