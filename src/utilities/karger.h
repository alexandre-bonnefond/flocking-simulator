int minCUT(int taille, int original[taille][taille]) ;
void affichage(int taille, int mat[taille][taille]) ;
void supprLigne(int taille, int mat[taille][taille], int ligne);
void merge(int taille, int mat[taille][taille], int ligne1, int ligne2);
int tailleMatrice(int taille, int mat[taille][taille]);
void choixRandom(int taille, int mat[taille][taille], int res[2]);
int coupeMin(int taille, int mat[taille][taille]);
int checkValide(int taille, int mat[taille][taille], int i);
int checkSym(int taille, int mat[taille][taille]);