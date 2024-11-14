#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>


using std::cout;
using std::cin;
using std::endl;
using std::ifstream;
using std::string;
using std::ofstream;

class Cesar{
 private:

 unsigned char ces_key;

   // Les tableaux sont dupliquer car la clé peux être grande mains comme on la maintiens basse grâce au modulo il faut juste le dupliquer par 2
 char maj[52] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};

 char min[52] = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};

 std::string c;  /// Notre chaine de caractère qui proviens de l'entrée

 std::string cesar_output = ""; /// Le fichier de sortis

 public:

  /// Constructeur par défaux
 /*
 Cesar() : ces_key(1), *c = "d" {}
 */
 /// Constructeur  qui prends le contenu du fichier d'entrée et le fichier et limite la clé en donnans le reste de la division euclidienne ainsi : 45 = 45 % 26 = 19

 Cesar(int ces_key,string entre){
  this->ces_key = ces_key % 26;
  c = entre;

 }

 ~Cesar(){}

 /// Les différentes méthodes qui chiffrent ou déchiffrent
 void encrypt(){
   bool found = 0;


   /// On regarde caractère par caractère
   for(int i = 0; i < c.size(); i++ ){

     /// Pour chaque caractère, on regarde si elle fais partie de notre alphabet et on la décale de la clée
    for(int y = 0; y < 26; y++){
      if(c[i] == maj[y]){
        cesar_output += maj[y + ces_key];
        found = 1;
      }
      if(c[i] == min[y]){
        cesar_output += min[y + ces_key];
        found = 1;
      }
    }



    /// Si le caractère n'étais pas une lettre, on la conserve, le code César ne modifie que les lettres
   if(!found){cesar_output += c[i];}

    /// Sinon on remet notre booléen en false
   else {found = 0;}
  }
 }



void decrypt(){ // Le déchiffrage est extrèmement similaire, en fait, on décale juste dans l'autre sens (on change le + en - pour le pointeur)
  bool found = 0;


  /// On regarde caractère par caractère
  for(int i = 0; i < c.size(); i++ ){

    /// Pour chaque caractère, on regarde si elle fais partie de notre alphabet et on la décale de la clée
    for(int y = 0; y < 26; y++){
      if(c[i] == maj[y]){
        cesar_output += maj[y - ces_key];
        found = 1;
      }
      if(c[i] == min[y]){
        cesar_output += min[y - ces_key];
        found = 1;
      }
    }



  /// Si le caractère n'étais pas une lettre, on la conserve, le code César ne modifie que les lettres
  if(!found){cesar_output += c[i];}

  /// Sinon on remet notre booléen en false
  else {found = 0;}
  }
}

std::string get_entre()const{return c;} /// à des fins de débug

std::string get_output()const{return cesar_output;} /// la sortie

};




class Xor{

private:
 unsigned char xor_key;
 
 std::string c;

 std::string xr_output = "";

 

public: 

 // Constructeur par défaux
/*
 Xor(){
  c = "d";
  xor_key = 1;  /// On donne la valeur 0000001 à notre clé

 }
*/
 /// Constructeur  qui prends le contenu du fichier d'entrée et le fichier et limite la clé en donnans le reste de la division euclidienne ainsi : 45 = 45 % 26 = 19

 Xor(int key,const string entre){

  // On prends la chaine de caractère
  c = entre;

 // on fait key modulo 256 pour obtenir notre xor key
 
  key %= 256;

  xor_key = key;}

  ~Xor(){} // Notre destructeur

 /*  Fonction pour convertir key en binaire, avant que je me rende compte que xor peux s'effectuer avec un entier
  for (int i = 7; i >= 0; --i) {
      xor_key[i] = (key >> (7 - i)) & 1;  // Décalage de key pour extraire le bit qui va du bit de point for au bit de point faible
    }

   }
  */

  /// fonction pour le encryptage et décryptage qui prends le fichier texte et le transforme en binaire
 void encrypt(){

   /// On parcour l'entrée caractère par caractère
   for(int i = 0; i < c.size(); i++){
   xr_output += (c[i] ^ xor_key);   /// Et oui, en une ligne, alors que j'étais partis pour le faire en 50 en passant par le binaire...
   }

 }

 string get_entre(){return c;}

 string get_output(){return xr_output;}
};













int main(){
int choice = 0;
int mode = 0;



while(choice > 3 || choice < 1 || mode < 1 || mode > 2){

    /// Entrée du choix de l'utilisateur

    cout << "Entrez l'opération voulue, 1 pour l'encryptage, 2 pour le décryptage" << std::endl;
    cin >> mode;
    cout << "Entrez le type voulu, 1 pour le César seul, 2 pour le Xor seul, 3 pour les deux" << std::endl;
    cin >> choice;

    if(choice > 3 || choice < 1 || mode < 1 || mode > 2) std::cout << "Entrée invalide, veuillez réessayer" << std::endl;
  
}

/// Entrée du nom du fichier

std::cout << "Entrez le nom du fichier sur lequel effectuer l'opération" << std::endl;
string filename;

std::cin >> filename;

/// Vérification que le fichier est dans la même dossier que l'executable
std::ifstream fichier(filename);  // Ouvrir le fichier en mode lecture
    if (!fichier) {
        std::cerr << "Erreur lors de l'ouverture du fichier, soit il n'existe pas, soit il n'est pas dans ce dossier." << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << fichier.rdbuf();  // Lire tout le contenu du fichier dans le flux

    std::string contenu = buffer.str();  // Récupérer le contenu du flux dans une string qui nous permettra ensuite de le convertir sans utiliser getline
    fichier.close();


    /// On montre le contenu du fichier
    std::cout << "Contenu du fichier d'entrée:" << std::endl;
    std::cout << contenu << std::endl;

/// Maintenant que le fichier est bien extrait il nous faut appliquer les
 std::string out = ""; // La variable qui stocke la sortie du cryptage, décryptage



 if(choice == 1){ // Si on utilise le système de chifrage Césard
   int ces_key;
   std::cout << "Entrez la cle Cesar" << std::endl;
   std::cin >> ces_key;
    Cesar cs(ces_key, contenu);
    if(mode == 1){cs.encrypt();}
    if(mode == 2){cs.decrypt();}
     out = cs.get_output(); // On crée une variable qui contient la sortie du décryptage/cryptage
   }

 if(choice == 2){// Si on utilise le système avec le xor
   int xor_key;
   std::cout << "Entrez la cle xor" << std::endl;
   std::cin >> xor_key;
   Xor xr(xor_key, contenu);
   xr.encrypt();
   
   std::cout << "Le code suite au cryptage/ décryptage est : \n \n" << xr.get_output() << std::endl;
   
   out = xr.get_output(); // On crée une variable qui contient la sortie du décryptage/cryptage
   
  }

 if(choice == 3){ // Si l'on utilise les deux méthodes
  int ces_key;
  int xor_key;

  std::cout << "Entrez la cle Cesar" << std::endl;
  std::cin >> ces_key;

  std::cout << "Entrez la cle xor" << std::endl;
  std::cin >> xor_key;

  // Maintenant, deux cas de figure, soit l'on encrypte, soit l'on décrypte

  if(mode == 1){ // Si l'on encrypte, on commence par Cesar, puis par xor 
  Cesar cs(ces_key, contenu);
  cs.encrypt();
  Xor xr(xor_key,cs.get_output());

  xr.encrypt();

  std::cout << "Le code suite au cryptage/ décryptage est : \n \n" << xr.get_output() << std::endl;

  out = xr.get_output(); // On crée une variable qui contient la sortie du décryptage/cryptage
  }

  if(mode == 2){// Si l'on décrypte, on commence par Cesar, puis par xor 
  Xor xr(xor_key, contenu);
  xr.encrypt();

  std::cout << "Après le décryptage xor :" << xr.get_output() << std::endl;
  Cesar cs(ces_key,xr.get_output());

  cs.decrypt();

  std::cout << "Le code suite au cryptage/ décryptage est : \n \n" << cs.get_output() << std::endl;

  out = cs.get_output(); // On crée une variable qui contient la sortie du décryptage/cryptage
  
  }


  }

 
 /// Maintenant que la variable out est contient le résultat on l'enregistre dans un fichier


 std::string filaname;

 std::cout << "Entrez le nom du fichier de sortie" << std::endl;

 std::cin >> filaname;

 std::fstream file_out;

 file_out.open(filaname, std::ios_base::out);
  file_out << out << std::endl;

  file_out.close();

  std::cout << out << "\n\n\n\n As been saved as : " << filaname << std::endl;
 
 
 }
