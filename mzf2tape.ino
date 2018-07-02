/*
   MZF2TAPE V1.0
   (c)Jean-François CAMPAN 2018 (pacman)

   Licence : GNU GPL v2
   
   Sur une idee de hlide (SD2MZCMT). Merci.

   Remplace le lecteur de Cassette du Sharp MZ-700
   par une version carte SD et diffusion de fichiers MZF/M12/MZT/BIN
   
   Utilise :
    - Afficheur graphique RepRapDiscount Full Graphic Smart Controller
    - ARDUINO ATMEGA 2560
    - CARTE de connexion

   Connexions :
    SHARP MZ-700              ARDUINO ATMEGA2560
       REMOTE                       14
       SENSE                        15
       WRITE                        18
       READ                         19
       E (GND)                      GND


  Utilisation :
   A l'initialisation demande le profil utiliser.
   Pour modifier un profil, faire un appui long (>5s) pendant le choix d'un fichier
   Les durées du profil personnalise sont modifiables.


  A Faire :
   - Avec 64 fichiers dans un dossier c'est encore trop lent !
   - Correction de bug de retour au dossier parent sur la bonne ligne
   - Interrompre la lecture avec le bouton
   - Afficher le % de lecture
   - Enregistrement sur carte SD des donnees issues de la borne WRITE directement en .MZF
   - Mettre en place les turbos
*/

#include "U8glib.h"
#include <RotaryEncoder.h>
#include <SdFat.h> 
#include <SPI.h>
#include <EEPROM.h>

#define NOM_PROGRAMME "MZF2TAPE v1.0"
#define     COPYRIGHT "(c)JFC 2018"

//#define INTERFACE_SERIE false
#define INTERFACE_SERIE_BAUD 115200

boolean transfert_rapide_actif = true ;

// ==================================
//  Interface Cassette SHARP MZ
// ==================================
#define MZ_CASSETTE_REMOTE   14     // Lecteur de cassette MZ Borne REMOTE (MOTOR ON : Entree) 
#define MZ_CASSETTE_SENSE    15     // Lecteur de cassette MZ Borne /SENSE (Detection mise en route moteur : Sortie) 

#define MZ_CASSETTE_WRITE    18     // Lecteur de cassette MZ Borne WRITE (Ecriture des donnees : Entree)
#define MZ_CASSETTE_READ     19     // Lecteur de cassette MZ Borne READ  (Lecture des donnees : Sortie)

#define LED                  13     // LED d'indication du fonctionnemnt de l'interface

// Definition des durees pour les differents type de machines
//  Duree Front Haut + Duree Periode en micro-secondes
unsigned long durees [12] = {
                             464, 958, // Sharp MZ-700/MZ-80K/MZ-80A : Bit 1
                             240, 504, // Sharp MZ-700/MZ-80K/MZ-80A : Bit 0
                             470, 954, // Sharp MZ-800 : Bit 1
                             240, 518, // Sharp MZ-800 : Bit 0
                             333, 667, // Sharp MZ-80B : Bit 1
                             167, 333, // Sharp MZ-80B : Bit 0
                            } ;
/*
   Profil Machine :
    Sharp MZ-700/MZ80K/MZ-80A : 0
    Sharp MZ-800              : 1
    Sharp MZ-80B              : 2
    Personnalise              : 3
 */
uint8_t profil_machine = 0 ;

// Valeurs par defaut du profil perso stockees en EEPROM
unsigned long profil_perso [4] = { 240, 504, 464, 958 } ;

/* 
  Calcul des multiples de 16us pour avoir la duree le plus proche
  des valeurs theoriques attendues
*/
unsigned long duree_haut_reelle_bit_1   = 0 ; 
unsigned long duree_periode_reelle_bit_1 = 0 ; 
unsigned long duree_haut_reelle_bit_0   = 0 ; 
unsigned long duree_periode_reelle_bit_0 = 0 ; 

// ==================================
//  Adresses de stockage EEPROM
// ==================================
#define EEPROM_CHOIX_PROFIL 0 // Permet de savoir si on a choisi un profil ou pas
#define EEPROM_NUM_PROFIL   2 // Numero du profil choisi
#define EEPROM_PERSO_BIT_0  4 // Largeur du bit 0 pour le profil personnalise (en us)
#define EEPROM_PERSO_BIT_1  8 // Largeur du bit 1 pour le profil personnalise (en us)


// ==================================
//  Interface SD
// ==================================
// Interface SD
#define SD0_MO        50     // SD MOSI
#define SD0_MI        51     // SD MISO
#define SD0_CK        52     // SD SCK
#define SD0_SS        53     // SS SS

// Structure des donnees
#define SD_REP_PROF            5    // Profondeur d'acces dans la structure maximale
#define SD_TAILLE_NOM_COURT   13    // Taille de fichier en 8.3
#define SD_TAILLE_NOM_LONG    25    // Taille du nom du fichier a utiliser

// Ensemble des types reconnus
#define TYPE_FORMAT_INCONNU        0
#define TYPE_FORMAT_RETOUR_DOSSIER 1
#define TYPE_FORMAT_REPERTOIRE     2
#define TYPE_FORMAT_MZF            3
#define TYPE_FORMAT_MZT            4
#define TYPE_FORMAT_M12            5
#define TYPE_FORMAT_BIN            6


// Variables SD
SdFat  sd ;
SdFile entree ;
SdFile repertoire [SD_REP_PROF] ;
char   nom_fichier_court [SD_TAILLE_NOM_COURT] ;
char   nom_fichier_long [SD_TAILLE_NOM_LONG+1] ;

uint8_t  entree_type = TYPE_FORMAT_INCONNU ; // Type par defaut
int16_t  entree_index = 0 ;
uint32_t entree_taille = 0 ;

bool    sd_pret = false ;
int8_t  sd_rep_profondeur = -1 ;
int16_t sd_rep_index [SD_REP_PROF] = { } ;

bool    annuler = false ;

// ==================================
//  LCD Graphique 128x64
// ==================================
// SPI Com: SCK = en = 23, MOSI = rw = 16, CS = di = 17
int LCD4 = 23 ; // D23
int LCDE = 17 ; // D17
int LCDRS = 16 ; // D16

// Icones
const uint8_t icone_dossier [] PROGMEM = {
                                          B00000,
                                          B00000,
                                          B00011,
                                          B11101,
                                          B10001,
                                          B10001,
                                          B11111,
                                          B00000
                                         } ;

const uint8_t icone_remonte [] PROGMEM = {
                                          B00000,
                                          B00100,
                                          B01110,
                                          B11111,
                                          B00100,
                                          B00100,
                                          B11100,
                                          B00000
                                         } ;



// ==================================
//  Encodeur rotatif integre
// ==================================
// BTN EN1=31 ; BTN EN2=33 ; SWITCH=35
int ENCODEUR_1 = 31 ;
int ENCODEUR_2 = 33 ;
int BOUTON = 35 ;
int encodeurDernierePosition = 0 ;
int encodeurNouvellePosition = 0 ;
int bouton_valeur = 0 ;
unsigned long date_debut = 0 ;
unsigned long date_fin = 0 ;

// Durees d'appuie minimal pour choisir ou modifier le profil machine
#define DUREE_BOOT 5           // Duree au BOOT 5s par defaut     
#define DUREE_INTERFACE 3      // Duree en utilisation 3s par defaut


// ==================================
//   Interface de menu
// ==================================
#define MENU_NOMBRE_MAX 500 // Nombre de fichiers maximum sur la carte SD
#define MENU_POS_Y 9 // Position de depart en Y du menu
#define MENU_NOMBRE_AFFICHAGE 6 // Nombre d'items a l'ecran maximum
#define MENU_SELECTION 2 // Position de la selection sur fichier

#define AFFICHE_PRESENTATION 0
#define AFFICHE_ERREUR_SD    1
#define AFFICHE_LECTURE_SD   2
#define AFFICHE_LC_SD        3
#define AFFICHE_LISTE_CHOIX  4
#define AFFICHE_PROPRIETES   5

uint16_t nombre_fichiers = 0 ; // Nombre de fichiers reel pour stockage
int16_t menu_nombre = 0 ; // Nombre de fichiers reel pour menu

int16_t menuNum [MENU_NOMBRE_MAX] ; // Contient la liste des numeros des fichiers dans l'ordre pour affichage

uint8_t menu_courant = 0 ;
uint8_t menu_position = 0 ;
int16_t menu_debut = -MENU_SELECTION ;
int16_t menu_debut_sauve = menu_debut ;
uint8_t menu_retrace = 1 ;
uint8_t menu_lecture = 0 ;


// ==================================
//   Type de fichier MZF/M12/MZT/BIN
// ==================================
#define MZF_INCONNU             0
#define MZF_BINAIRE             1
#define MZF_KUMA_INT            2
#define MZF_KUMA_COMPILER       3
#define MZF_BASIC_MZ700         4
#define MZF_BINAIRE_PURE        5
#define MZF_DATA_MZ700          6
#define MZF_BASIC_MZ80          7
#define MZF_DATA_MZ80           8
#define MZF_BINAIRE_MZF1        9

byte mzf_type = 0 ;
unsigned int mzf_taille = 0 ;
unsigned int mzf_adresse = 0 ;
unsigned int mzf_execution = 0 ;


// ==================================
//  Initialisation des peripheriques
// ==================================
U8GLIB_ST7920_128X64_1X lcd (LCD4, LCDE, LCDRS) ;
RotaryEncoder encoder (ENCODEUR_1, ENCODEUR_2) ;


/*
  =================================================================================
   Fonction de convertion d'un nombre en uint32_t vers chaine de 6 caracteres
   sans les '0' initiaux pour affichage de la taille d'un fichier
  =================================================================================
 */
const char *uint32_t2Char (uint32_t taille)
 {
  byte i = 0 ;
  static char buf [7] ;
  static char buf1 [2] ;
  uint32_t v1 = taille/10000 ;
  strcpy (buf1, u8g_u8toa ((uint8_t)(v1), 2)) ;
  buf [0] = buf1 [0] ;
  buf [1] = buf1 [1] ;
  uint32_t v2 = (taille-v1*10000)/100 ;
  strcpy (buf1, u8g_u8toa ((uint8_t)(v2), 2)) ;
  buf [2] = buf1 [0] ;
  buf [3] = buf1 [1] ;
  uint32_t v3 = taille-v1*10000-v2*100 ;
  strcpy (buf1, u8g_u8toa ((uint8_t)(v3), 2)) ;
  buf [4] = buf1 [0] ;
  buf [5] = buf1 [1] ;
  buf [6] = '\0' ;
  // Enleve les '0' a gauche
  while ((buf [i] == '0') && (i < 6)) { buf [i] = (char)' ' ; i++ ; }
  return buf ;
 }

/*
  =================================================================================
   Fonction de convertion d'un nombre en unsigned long vers chaine de 4 caracteres
   sans les '0' initiaux pour affichage de la taille d'un fichier
  =================================================================================
 */
const char *long2Char (uint32_t valeur)
 {
  byte i = 0 ;
  static char buf [5] ;
  static char buf1 [2] ;
  uint32_t v1 = valeur/100 ;
  strcpy (buf1, u8g_u8toa ((uint8_t)(v1), 2)) ;
  buf [0] = buf1 [0] ;
  buf [1] = buf1 [1] ;
  uint32_t v2 = valeur-v1*100 ;
  strcpy (buf1, u8g_u8toa ((uint8_t)(v2), 2)) ;
  buf [2] = buf1 [0] ;
  buf [3] = buf1 [1] ;
  buf [4] = '\0' ;
  // Enleve les '0' a gauche
  while ((buf [i] == '0') && (i < 4)) { buf [i] = (char)' ' ; i++ ; }
  if (i == 4) { buf [3] = (char)'0' ; }
  return buf ;
 }

/*
  =================================================================================
   Fonction de convertion d'un nombre en uint16_t vers chaine de 4 caracteres
   pour affichage hexadecimale
  =================================================================================
 */
const char *uint16_t2Hexa (uint16_t valeur)
 {
  char c [16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'} ;
  static char buf [5] ;
  uint8_t v1 = (byte)(valeur/4096) ;
  buf [0] = c [v1] ;
  uint8_t v2 = (byte)((valeur-v1*4096)/256) ;
  buf [1] = c [v2] ;
  uint8_t v3 = (byte)((valeur-v1*4096-v2*256)/16) ;
  buf [2] = c [v3] ;
  uint8_t v4 = (byte)(valeur-v1*4096-v2*256-v3*16) ;
  buf [3] = c [v4] ;
  buf [4] = '\0' ;
  return buf ;
 }

/*
 * Fonction de lecture de l'EEPROM pour retouver les valeurs personnalisees
 */
void lectureProfil ()
 {
  profil_machine = EEPROM.read (EEPROM_NUM_PROFIL) ;
  if (profil_machine == 3)
   {
    // On recupere le profil personnalise
    for (uint8_t i = 0 ; i < 2 ; i++)
     {
      profil_perso [i+0] = (uint16_t)(EEPROM.read (EEPROM_PERSO_BIT_0+i*2+0))+(uint16_t)(EEPROM.read (EEPROM_PERSO_BIT_0+i*2+1)<<8) ;
      profil_perso [i+2] = (uint16_t)(EEPROM.read (EEPROM_PERSO_BIT_1+i*2+0))+(uint16_t)(EEPROM.read (EEPROM_PERSO_BIT_1+i*2+1)<<8) ;
     }
    if (profil_perso [0] > 9999) { profil_perso [0] = 240 ; }
    if (profil_perso [1] > 9999) { profil_perso [1] = 504 ; }
    if (profil_perso [2] > 9999) { profil_perso [2] = 464 ; }
    if (profil_perso [3] > 9999) { profil_perso [3] = 958 ; }

    if (profil_perso [1] < profil_perso [0]) { profil_perso [1] = profil_perso [0] ; }
    if (profil_perso [3] < profil_perso [2]) { profil_perso [3] = profil_perso [2] ; }
   }
 }

/*
 * Fonction de sauvegarde en EEPROM du profil selectione
 */
void ecritureProfil ()
 {
  EEPROM.write (EEPROM_CHOIX_PROFIL, 1) ;
  EEPROM.write (EEPROM_NUM_PROFIL, profil_machine) ;
  if (profil_machine == 3)
   {
    // On ecrit le profil personnalise
    for (uint8_t i = 0 ; i < 2 ; i++)
     {
      EEPROM.write (EEPROM_PERSO_BIT_0+i*2+0, (byte)(profil_perso [i] & 0x00FF)) ;          // Bit 0 H
      EEPROM.write (EEPROM_PERSO_BIT_0+i*2+1, (byte)((profil_perso [i] >>8) & 0x00FF)) ;    // Bit 0 T
      
      EEPROM.write (EEPROM_PERSO_BIT_1+i*2+0, (byte)(profil_perso [i+2] & 0x00FF)) ;        // Bit 1 H
      EEPROM.write (EEPROM_PERSO_BIT_1+i*2+1, (byte)((profil_perso [i+2] >>8) & 0x00FF)) ;  // Bit 1 T
     }
   }
 }
 
/*
  =================================================================================
   Fonction de classement en ordre alphabetique des fichiers d'un dossier
  =================================================================================
*/
/*
   Fonction de comparaison caractere a caractere pour ordre alphabetique
   Ne tient pas compte de la casse.
   Renvoi 1 si Inversion a faire sinon 0
*/
byte nomPlusPetit (char *nom_1, char *nom_2)
 {
  char car1 ;
  char car2 ;
  uint16_t i = 0 ;
  while (i < strlen (nom_1))
   {
    car1 = (char)toLowerCase(nom_1 [i]) ;
    if (strlen (nom_2) < i) { return 1 ; }
    else
     {
      car2 = (char)toLowerCase (nom_2 [i]) ;
      if (car2 > car1) { return 1 ; }
      if (car2 < car1) { return 0 ; }
      i++ ;
     }
   }
  return 0 ; 
 }

/*
  Fonction de classement par ordre alphabetique en utilisant la methode
  du tri a bulle
*/
void ordreAlphabetique (int16_t premier)
 {
  int16_t i ;
  int16_t j ;
  int16_t k ;
  char nom1 [SD_TAILLE_NOM_LONG+1] ;
  char nom2 [SD_TAILLE_NOM_LONG+1] ;

  for (i = premier ; i < menu_nombre ; i++)
   {
    j = i ;
    while (j >= premier)
     {
      obtientEntree (menuNum [j]) ;
      for (k = 0 ; k <= SD_TAILLE_NOM_LONG ; k++) { nom1 [k] = nom_fichier_long [k] ; }
      obtientEntree (menuNum [j-1]) ;
      for (k = 0 ; k <= SD_TAILLE_NOM_LONG ; k++) { nom2 [k] = nom_fichier_long [k] ; }
      if (nomPlusPetit (nom1, nom2) == 1)
       {
        // Inversion
        k = menuNum [j] ;
        menuNum   [j] = menuNum [j-1] ;
        menuNum [j-1] = k ;
       }
      j-- ;
     }
   }
 }


/*
  =================================================================================
   Fonctions de base de reconnaissance des formats de fichier
  =================================================================================
*/
bool testExtmzf (char *nom_fichier)
 {
  return !!strstr (strlwr (nom_fichier + (strlen (nom_fichier)-4)), ".mzf") ;
 }

bool testExtmzt (char *nom_fichier)
 {
  return !!strstr (strlwr (nom_fichier + (strlen (nom_fichier)-4)), ".mzt") ;
 }

bool testExtm12 (char *nom_fichier)
 {
  return !!strstr (strlwr (nom_fichier + (strlen (nom_fichier)-4)), ".m12") ;
 }

bool testExtbin (char *nom_fichier)
 {
  return !!strstr (strlwr (nom_fichier + (strlen (nom_fichier)-4)), ".bin") ;
 }


/*
  =================================================================================
   Fonctions de reconnaissance de type de donnees
  =================================================================================
*/
void typeFichierMZ ()
 {
  unsigned char donnee ;
  
  entree.seekSet (0) ;
  
  // Lecture du type de fichier
  donnee = entree.read () ;

  switch (donnee)
   {
    // Binaire et Kuma Interpreter/Compiler
    case 0x01 : if (mzf_execution != 0x0000) { mzf_type = MZF_BINAIRE ; }
                                        else { mzf_type = MZF_KUMA_COMPILER ; }
                // Lecture du nom du fichier long
                for (int i = 0 ; i < 17 ; i++) { entree.read () ; }
                break ;
    // Basic MZ80
    case 0x02 : mzf_type = MZF_BASIC_MZ80 ;
                // Lecture du nom du fichier long
                for (int i = 0 ; i < 17 ; i++) { entree.read () ; }
                break ;
    // DATA MZ80
    case 0x03 : mzf_type = MZF_DATA_MZ80 ;
                // Lecture du nom du fichier long
                for (int i = 0 ; i < 17 ; i++) { entree.read () ; }
                break ;
    case 0x04 : mzf_type = MZF_DATA_MZ700 ;
                break ;
    // Sharp Basic 1Z-013B et Kuma Interpreter/Compiler
    case 0x05 :      if ((mzf_adresse == 0x6BCF) ||
                         (mzf_adresse == 0x0000))   { mzf_type = MZF_BASIC_MZ700 ; }
                else if  (mzf_adresse == 0x4000)    { mzf_type = MZF_KUMA_INT ; }
                // Lecture du nom du fichier long
                for (int i = 0 ; i < 17 ; i++) { entree.read () ; }
                break ;
    // MZF1
    case 0x4D : //(Buffer [0] = $4D) And (Buffer [1] = $5A) And (Buffer [2] = $46) And
                //(Buffer [3] = $31)
                mzf_type = MZF_BINAIRE_MZF1 ;
                // Saut de l'entete MZF1
                for (int i = 0 ; i < 4 ; i++) { entree.read () ; }
                // Lecture du nom du fichier long
                for (int i = 0 ; i < 17 ; i++) { entree.read () ; }
                break ;
    // Binaire BIN
    case 0xFE : mzf_type = MZF_BINAIRE_PURE ;
                break ;
   }

  if (mzf_type == MZF_BINAIRE_PURE)
   {
    // Adresse
    mzf_adresse    = (entree.read () & 0xFF) << 0 ;
    mzf_adresse   += (entree.read () & 0xFF) << 8 ;
    
    // Taille
    mzf_taille     = entree_taille-7 ;
    entree.read () ; 
    entree.read () ;
    
    // Execution
    mzf_execution  = (entree.read() & 0xFF) << 0 ;
    mzf_execution += (entree.read() & 0xFF) << 8 ;
   }
  else
   {
    mzf_taille     = (entree.read () & 0xFF) << 0 ;
    mzf_taille    += (entree.read () & 0xFF) << 8 ;

    mzf_adresse    = (entree.read () & 0xFF) << 0 ;
    mzf_adresse   += (entree.read () & 0xFF) << 8 ;
    
    mzf_execution  = (entree.read() & 0xFF) << 0 ;
    mzf_execution += (entree.read() & 0xFF) << 8 ;
   }
 }


/*
  =================================================================================
   Fonction de lecture de la carte SD
  =================================================================================
*/
/*
  Fonction qui renvoi l'entree courante du repertoire ouvert courant
  @arg int16_t nouvel_index Numero valide de l'entreea obtenir
*/
void obtientEntree (int16_t nouvel_index)
 {
  bool    trouve = true ;
  int16_t index = 0 ;
    
  entree.close () ;
  if (nouvel_index < 0) { nouvel_index = 0 ; }
  do
   {
    repertoire [sd_rep_profondeur].rewind () ;
    index = 0 ;
    while (index <= nouvel_index)
     {
      trouve = entree.openNext (&repertoire [sd_rep_profondeur], O_READ) ;
      if (trouve)
       {
        if ((!entree.isHidden ()) && (!entree.isSystem ()))
         {
          if (index == nouvel_index) { break ; }
          index++ ;
         }
        entree.close () ;
       }
      else { break ; }
     }
    if (!trouve) { nouvel_index = entree_index ; }
   }
  while (!trouve && index > 0) ;

  if (trouve)
   {
    entree.getSFN  (nom_fichier_court) ;
    entree.getName (nom_fichier_long, SD_TAILLE_NOM_LONG) ;
    
         if (entree.isDir ())               { entree_type = TYPE_FORMAT_REPERTOIRE ; }
    else if (testExtmzf (nom_fichier_long)) { entree_type = TYPE_FORMAT_MZF ; }
    else if (testExtmzt (nom_fichier_long)) { entree_type = TYPE_FORMAT_MZT ; }
    else if (testExtm12 (nom_fichier_long)) { entree_type = TYPE_FORMAT_M12 ; }
    else if (testExtbin (nom_fichier_long)) { entree_type = TYPE_FORMAT_BIN ; }
    else                                    { entree_type = TYPE_FORMAT_INCONNU ; }
    
    entree_taille = entree.fileSize () ;
    entree_index = nouvel_index ;
   }
  else
   {
    memset (nom_fichier_court, 0, SD_TAILLE_NOM_COURT) ;
    memset (nom_fichier_long, 0, SD_TAILLE_NOM_LONG + 1) ;
    strcpy (nom_fichier_long, "<pas de fichier>") ;
    Serial.println ("Erreur : Aucun fichier ou dossier !") ;
   }
 }

/* 
  Fonction permettant d'entrer dans un dossier
*/
void entrerRepertoire ()
 {
  if (sd_rep_profondeur < SD_REP_PROF - 2)
   {
    if (sd_rep_profondeur < 0)
     {
      if (repertoire[0].openRoot (&sd))
       {
        sd_rep_profondeur++ ;
        obtientEntree (0) ;
       }
      else
       {
        Serial.println ("Erreur : Je ne peux pas ouvrir le repertoire racine !") ;
       }
     }
    else if (entree.isOpen ())
     {
      sd_rep_profondeur++ ;
      repertoire [sd_rep_profondeur] = entree ;
      sd_rep_index [sd_rep_profondeur] = entree_index ;
      //Serial.print ("sd_rep_index [") ; Serial.print (sd_rep_profondeur) ; Serial.print ("] = ") ; Serial.println (entree_index) ;
      obtientEntree (0) ;
     }
    else
     {
      Serial.println ("Erreur : Dossier inexistant !") ;
     }
   }
  else
   {
    Serial.println ("Erreur : Trop de sous-dossier !") ;
   }
 }

/*
  Fonction qui permet de quitter un dossier pour remonter a sa racine
*/
void sortirRepertoire ()
 {
  if (sd_rep_profondeur > 0)
   {
    repertoire [sd_rep_profondeur].close () ;
    entree_index = sd_rep_index [sd_rep_profondeur] ;
    //Serial.print ("entree_index=") ; Serial.println (entree_index) ;
    sd_rep_profondeur-- ;
    //obtientEntree (entree_index) ;
   }
 }

/*
   Fonction de recherche, 'd'ajout et de classement des repertoires et fichiers
   d'un dossier courant
*/
void rechercheFichiers ()
 {
  uint8_t old_entree = 0 ;
  uint16_t nbrDossiers = 0 ;
  nombre_fichiers = 0 ;
  obtientEntree (0) ;
  // Recherche des dossiers
  do
   {
    if (entree_type == TYPE_FORMAT_REPERTOIRE)
     {
      //Serial.print ("menuNum[") ; Serial.print (nombre_fichiers) ; Serial.print ("]=") ; Serial.print (entree_index) ;
      //Serial.print (" > ") ; Serial.println (nom_fichier_long) ;
      menuNum [nombre_fichiers] = entree_index ;
      nombre_fichiers++ ;
     }
    old_entree = entree_index ;
    obtientEntree (entree_index + 1) ;
   }
  while (entree_index > old_entree) ;
  
  nbrDossiers = nombre_fichiers ;
  menu_nombre = nombre_fichiers ;
  if (menu_nombre > 1) {  ordreAlphabetique (1) ; }
  //for (int16_t l = 0 ; l < menu_nombre ; l++)
  // {
  //  Serial.print (menuNum [l]) ; Serial.print (" ") ; 
  // }
  //Serial.println () ;

  // Recherche des fichiers
  obtientEntree (0) ;
  do
   {
    if (entree_type > TYPE_FORMAT_REPERTOIRE)
     {
      //Serial.print ("menuNum[") ; Serial.print (nombre_fichiers) ; Serial.print ("]=") ; Serial.print (entree_index) ;
      //Serial.print (" > ") ; Serial.println (nom_fichier_long) ;
      menuNum [nombre_fichiers] = entree_index ;
      nombre_fichiers++ ;
     }
    old_entree = entree_index ;
    obtientEntree (entree_index + 1) ;
   }
  while (entree_index > old_entree) ;
  menu_nombre = nombre_fichiers ;
  if (menu_nombre-nbrDossiers > 1) { ordreAlphabetique (nbrDossiers+1) ; }
  //for (int16_t l = 0 ; l < menu_nombre ; l++)
  // {
  //  Serial.print (menuNum [l]) ; Serial.print (" ") ; 
  // }
  //Serial.println () ;
 }

/*
  =================================================================================
   Fonctions de lecture des fichiers + diffusion vers le MZ
  =================================================================================
*/

/*
   Fonction de calcul des durees reelles en fonction du profil machine
 */
void calculDurees (uint8_t pm)
 {
  unsigned long a = 0 ;
  if (pm == 3) { a = profil_perso [2] ; } else { a = (unsigned long)(durees [pm*4]/16) ; }
  a = a*16+16 ;
  if (a > durees [pm*4]) { duree_haut_reelle_bit_1 = a-16 ; }
  else { duree_haut_reelle_bit_1 = a ; }

  if (pm == 3) { a = profil_perso [3] ; } else { a = (unsigned long)(durees [pm*4+1]/16) ; }
  a = a*16+16 ;
  if (a > durees [pm*4+1]) { duree_periode_reelle_bit_1 = a-16 ; }
  else { duree_periode_reelle_bit_1 = a ; }

  if (pm == 3) { a = profil_perso [0] ; } else { a = (unsigned long)(durees [pm*4+2]/16) ; }
  a = a*16+16 ;
  if (a > durees [pm*4+2]) { duree_haut_reelle_bit_0 = a-16 ; }
  else { duree_haut_reelle_bit_0 = a ; }

  if (pm == 3) { a = profil_perso [1] ; } else { a = (unsigned long)(durees [pm*4+3]/16) ; }
  a = a*16+16 ;
  if (a > durees [pm*4+3]) { duree_periode_reelle_bit_0 = a-16 ; }
  else { duree_periode_reelle_bit_0 = a ; }
 }

/*
   Emission d'un Bit d'une duree en microsecondes
   et d'un temps haut d'une duree tempsHaut
 */
void emissionBit (unsigned long duree, unsigned long tempsHaut)
 {
  unsigned long depart = micros () ; // Debut d'emission
  digitalWrite (MZ_CASSETTE_READ, HIGH) ; // Mise a 1 de la sortie
  while (micros () < depart + tempsHaut) ;
  digitalWrite (MZ_CASSETTE_READ, LOW) ; // Mise a 0 de la sortie
  while (micros () < depart + duree) ;
 }

/*
   Emission du GAP : Synchronisation logiciel/materiel
    LGAP : 22000 impulsions bit 0
    SGAP : 11000 impulsions bit 0
 */
void emissionGAP (uint16_t nombre)
 {
  for (uint16_t i = 0 ; i < nombre ; i++)
   emissionBit (duree_periode_reelle_bit_0, duree_haut_reelle_bit_0) ;
 }

/*
   Emission TapeMark : Indique le debut d'un bloc de donnees
   LTM : 40 impulsions bit 1 + 40 impulsions bit 0 + 1 Bit 1
   STM : 20 impulsions bit 1 + 20 impulsions bit 0 + 1 Bit 1
 */
void emissionTapeMark (uint16_t nombre)
 {
  uint16_t i = 0 ;
  for (i = 0 ; i < nombre ; i++)
   emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ;
  for (i = 0 ; i < nombre ; i++)
   emissionBit (duree_periode_reelle_bit_0, duree_haut_reelle_bit_0) ;
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ;
 }

/*
   Emission d'un octet
*/
uint8_t emissionOctet (unsigned char valeur)
 {
  uint8_t nbr = 0 ;
  for (int i = 7 ; i >= 0 ; i--)
   {
    if (valeur & (1 << i)) { emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; nbr++ ;}
    else { emissionBit (duree_periode_reelle_bit_0, duree_haut_reelle_bit_0) ; }
   }
  // Bit 1 de fin 
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ;
  return nbr ;
 }

/*
   Focntion de lecture du fichier de type MZF/M12/BIN
*/
void lectureFichierMZF ()
 {
  //unsigned long   total = entree.fileSize () ;
  unsigned char   donnee ;
  uint32_t        checkSum ;
  unsigned int    i = 0 ;
  unsigned int    j = 0 ;
  int             front ;
  unsigned char   transfert_rapide [77] =
   {
    0x01, // Binaire
    // Nom
    0x54, 0x52, 0x41, 0x4E, 0x53, 0x46, 0x45, 0x52, 0x54, 0x20, 0x52, 0x41, 0x50, 0x49, 0x44, 0x45, 0x0D,
    // Taille
    0x03, 0x00, 
    // Adresse
    0x00, 0xD4,
    // Execution programme qui suit
    0x00, 0xD4,
                      // Org $1108 
    0x01, 0x10, 0x00, //   LD BC, Taille
    0x21, 0x00, 0x12, //   LD HL, Adresse
    0x3E, 0x02,       //   LD A, $02      Front bas
    0x32, 0x02, 0xE0, //   LD ($E002), A
                      // A:
    0xE5,             //   PUSH HL
    0x21, 0x02, 0xE0, //   LD HL, $E002
    0x11, 0x08, 0x00, //   LD DE, $0008  (D=$00 et E=$08) D=Valeur et E=Nbr bits
                      // B:
    0x7E,             //   LD A, (HL)
    0xE6, 0x10,       //   AND $10  (MOTOR ON : SENSE = 0 ?)
    0x20, 0xFB,       //   JR NZ, B:
    0xAF,             //   XOR A          Front haut
    0x77,             //   LD (HL), A     DWRITE =  1
                      // C:
    0x7E,             //   LD A, (HL)
    0xE6, 0x10,       //   AND $10  (MOTOR ON : SENSE = 1 ?)
    0x28, 0xFB,       //   JR Z, C:
    0x7E,             //   LD A, (HL)   recupere 1 bit de donnee (ordre bit 5,4,3,2,1,0,7,6)
    0xE6, 0x20,       //   AND $20      recupere le bit de donnee
    0xB2,             //   OR D
    0x07,             //   RLCA
    0x57,             //   LD D, A
    0x3E, 0x02,       //   LD A, $02      Front bas
    0x77,             //   LD (HL), A     DWRITE =  0
    0x1D,             //   DEC E
    0x20, 0xE8,       //   JR NZ, B:
    0xE1,             //   POP HL
    0x72,             //   LD (HL), D  Stockage de la valeur au bon endroit
    0x0B,             //   DEC BC
    0x79,             //   LD A, C
    0xB0,             //   OR B
    0x23,             //   INC HL
    0x20, 0xD9,       //   JR NZ, A:
    0xC3, 0xAD, 0x00  //   JP Programme   Fini on execute
   } ;
  uint8_t         ordre [8] = { 32, 16, 8, 4, 2, 1, 128, 64 } ;
  uint32_t        position = 0x80 ;
  uint8_t         entete [128] ;
  unsigned long   t1, t2 ;

  if ((mzf_type == MZF_BINAIRE) ||
      (mzf_type == MZF_KUMA_COMPILER) ||
      (mzf_type == MZF_BINAIRE_PURE) ||
      (mzf_type == MZF_BINAIRE_MZF1)) { transfert_rapide_actif = true ; }
                                 else { transfert_rapide_actif = false ; }

  // Remplissage du tableau entete
  if (transfert_rapide_actif == true)
   {
    // Entete transfert rapide
    for (i = 0 ; i < 77 ; i++) { entete [i] = transfert_rapide [i] ; }
    for (i = 77 ; i < 128 ; i++) { entete [i] = 0 ; }
   }

  entree.seekSet (0) ;
  if (mzf_type == MZF_BINAIRE_PURE)
   {
    if (transfert_rapide_actif == false)
     {
      entete [0] = (uint8_t)(entree.read () & 0xFF) ;
      for (i = 0 ; i < 128 ; i++) { entete [i] = 0x00 ; }
      // Type BINAIRE OBJ
      entete [0] = 0x01 ;
     }
    else { donnee = (uint8_t)(entree.read () & 0xFF) ; }
    
    // Nom du fichier
    for (i = 0 ; i < 12 ; i++) { entete [i+1] = strupr (nom_fichier_court [i]) ; }
    for (i = 13 ; i < 18 ; i++) { entete [i] = 0x0D ; }
    // Taille, Adresse, Execution
    if (transfert_rapide_actif == false)
     {
      // Transfert Conventionnel
      // Adresse
      entete [20] = (uint8_t)(entree.read () & 0xFF) ;
      entete [21] = (uint8_t)(entree.read () & 0xFF) ;
      // Taille
      uint16_t taille = entree_taille-7 ;
      entete [18] = (uint8_t)(taille & 0x00FF) ;
      entete [19] = (uint8_t)((taille >> 8) & 0x00FF) ;
      entree.read () ;
      entree.read () ;
      // Execution
      entete [22] = (uint8_t)(entree.read () & 0xFF) ;
      entete [23] = (uint8_t)(entree.read () & 0xFF) ;
     }
    else
     {
      // Transfert Rapide
      // Adresse
      entete [28] = (uint8_t)(entree.read () & 0xFF) ;
      entete [29] = (uint8_t)(entree.read () & 0xFF) ;
      // Taille
      uint16_t taille = entree_taille-7 ;
      entete [25] = (uint8_t)(taille & 0x00FF) ;
      entete [26] = (uint8_t)((taille >> 8) & 0x00FF) ;
      entree.read () ;
      entree.read () ;
      // Execution
      entete [75] = (uint8_t)(entree.read () & 0xFF) ;
      entete [76] = (uint8_t)(entree.read () & 0xFF) ;
     }
    
    // Position des donnees
    position = 0x07 ;
   }
  else if (mzf_type == MZF_BINAIRE_MZF1)
   {
    // Lecture des 4 premiers octets MZF1
    for (i = 0 ; i < 4 ; i++) { entree.read () ; }
    if (transfert_rapide_actif == false)
     {
      // Transfert Conventionnel
      // Chargement de toute l'entete
      for (i = 0 ; i < 128 ; i++) { entete [i] = (uint8_t)(entree.read () & 0xFF) ; }
     }
    else
     {
      // Transfert rapide
      // Lecture type de fichier
      entree.read () ;
      // Nom du fichier
      for (i = 1 ; i < 18 ; i++) { entete [i] = (uint8_t)(entree.read () & 0xFF) ; }
      // Taille
      entete [25] = (uint8_t)(entree.read () & 0xFF) ;
      entete [26] = (uint8_t)(entree.read () & 0xFF) ;
      // Adresse
      entete [28] = (uint8_t)(entree.read () & 0xFF) ;
      entete [29] = (uint8_t)(entree.read () & 0xFF) ;
      // Execution
      entete [75] = (uint8_t)(entree.read () & 0xFF) ;
      entete [76] = (uint8_t)(entree.read () & 0xFF) ;
     }
    // Position des donnees
    position = 0x84 ;
   }
  else // MZF/MZT/M12
   {
    if (transfert_rapide_actif == false)
     {
      // Transfert Conventionnel
      // Chargement de toute l'entete
      for (i = 0 ; i < 128 ; i++) { entete [i] = (uint8_t)(entree.read () & 0xFF) ; }
     }
    else
     {
      // Transfert rapide
      // Type de programme
      donnee = (uint8_t)(entree.read () & 0xFF) ;
      // Nom du fichier
      for (i = 1 ; i < 18 ; i++) { entete [i] = (uint8_t)(entree.read () & 0xFF) ; }
      // Taille
      entete [25] = (uint8_t)(entree.read () & 0xFF) ;
      entete [26] = (uint8_t)(entree.read () & 0xFF) ;
      // Adresse
      entete [28] = (uint8_t)(entree.read () & 0xFF) ;
      entete [29] = (uint8_t)(entree.read () & 0xFF) ;
      // Execution
      entete [75] = (uint8_t)(entree.read () & 0xFF) ;
      entete [76] = (uint8_t)(entree.read () & 0xFF) ;
     }
    // Position des donnees
    position = 0x80 ;
   }

  // Debut de lecture
  lcd.firstPage () ;
  do 
   {
    proprietesFichier (2) ;
    lcd.setDefaultForegroundColor () ;
    lcd.drawStr (5, 55, "LECTURE : Entete") ;
   }
  while (lcd.nextPage ()) ;
   
  // -------------------------------------------------------------------
  // Entete caracteristiques fichier
  //
  delay (2000) ; // Attente de 2s pour contrer l'attente MONITOR MOTOR
  
  emissionGAP (100) ; // 100 impulsions prises en compte par le MONITOR au lieu de 22000
  emissionTapeMark (40) ; // 40 "1" + 40 "0" + 1 "1"
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; // 1 "1"

  // Entete 1
  checkSum = 0 ;
  for (i = 0 ; i < 128 ; i++)
   {
    checkSum += emissionOctet (entete [i]) ;
   }
  emissionOctet ((unsigned char)((checkSum >> 8) & 0x00FF)) ; // Poids Fort
  emissionOctet ((unsigned char)(checkSum & 0x00FF)) ; // Poids Faible
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; // 1 "1"
  // -------------------------------------------------------------------
  
  digitalWrite (LED, LOW) ;

  lcd.firstPage () ;
  do 
   {
    proprietesFichier (2) ;
    lcd.setDefaultForegroundColor () ;
    lcd.drawStr (5, 55, "LECTURE : Programme") ;
   }
  while (lcd.nextPage ()) ;

  digitalWrite (LED, HIGH) ;

  // -------------------------------------------------------------------
  // Synchronisation entete PROGRAMME
  //
  entree.seekSet (position) ;

  delay (2000) ; // Attente de 2s pour contrer l'attente MONITOR MOTOR
  emissionGAP (100) ; // 100 impulsions prises en compte par le MONITOR au lieu de 22000
  emissionTapeMark (20) ; // 20 "1" + 20 "0" + 1 "1"
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; // 1 "1"

  // Programme
  if (transfert_rapide_actif == false)
   {
    checkSum = 0 ;
    for (i = 0 ; i < mzf_taille ; i++)
     {
      donnee = entree.read () ;
      checkSum += emissionOctet (donnee) ;
     }
    //emissionOctet ((unsigned char)((checkSum >> 8) & 0x00FF)) ; // Poids Fort
    //emissionOctet ((unsigned char)(checkSum & 0x00FF)) ; // Poids Faible
    //emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; // 1 "1"
   }
  else
   {
    // Mode Transfert Rapide
    checkSum = emissionOctet (0xC3) ;
    checkSum += emissionOctet (0x08) ;
    checkSum += emissionOctet (0x11) ;
   }
  emissionOctet ((unsigned char)((checkSum >> 8) & 0x00FF)) ; // Poids Fort
  emissionOctet ((unsigned char)(checkSum & 0x00FF)) ; // Poids Faible
  emissionBit (duree_periode_reelle_bit_1, duree_haut_reelle_bit_1) ; // 1 "1"
  // ===================================================================

  if (transfert_rapide_actif == true)
   {
    t1 = micros () ;
    // Reinitialisation
    digitalWrite (MZ_CASSETTE_SENSE, HIGH) ; // signal /SENSE a 1 (Lecteur non disponible SENSE=0)
    
    entree.seekSet (position) ;

    lcd.firstPage () ;
    do 
     {
      proprietesFichier (2) ;
      lcd.setDefaultForegroundColor () ;
      lcd.drawStr (5, 55, "LECTURE : Rapide") ;
     }
    while (lcd.nextPage ()) ;

    // REM : Malgre le positionnement de seekSet il ne veut pas aller en position $80 ou $84
    //       J'ai du faire une erreur quelque part, mais je n'ai pas le temps de chercher encore...
    //       Donc pour l'instant bricolage, on saute 104 octets !!!!
    if (position >= 0x80) { for (i = 0 ; i < 104 ; i++) { donnee = (uint8_t)(entree.read () & 0xFF) ; } }
    //       Mais c'est a revoir evidemment.
    
    for (i = 0 ; i < mzf_taille ; i++)
     {
      donnee = (uint8_t)(entree.read () & 0xFF) ;
      for (j = 0 ; j < 8 ; j++)
       {
        // Attente demande donnee
        while (digitalRead (MZ_CASSETTE_WRITE) == 0) { }
        
        // Positionne le bit
        digitalWrite (MZ_CASSETTE_READ, ((donnee & ordre [j]) != 0) ? LOW : HIGH) ;

        // Informe le positionnement du bit
        digitalWrite (MZ_CASSETTE_SENSE, LOW) ; // signal /SENSE a 0 (Lecteur disponible SENSE=1)
      
        // Attente de la lecture de la donnee
        while (digitalRead (MZ_CASSETTE_WRITE) == 1) { }

        // Reinitialisation
        digitalWrite (MZ_CASSETTE_SENSE, HIGH) ; // signal /SENSE a 1 (Lecteur non disponible SENSE=0)
       }
     }
   }
 
  digitalWrite (MZ_CASSETTE_READ, HIGH) ; // Signal de donnee a 1  -> DREAD=0
  
  t2 = micros () ;
  Serial.print (t2) ; Serial.print ("-") ; Serial.print (t1) ; Serial.print ("=") ; Serial.print (t2-t1) ; Serial.println (" us") ;
 }
 

/*
  =================================================================================
   Fonctions de gestion du lcd
  =================================================================================
*/
/*
   Trace la presentation du logiciel
*/
void presentation ()
 {
  lcd.setFont (u8g_font_6x10) ;
  lcd.setFontRefHeightText () ;
  lcd.setFontPosTop () ;
  lcd.drawStr ((lcd.getWidth ()-lcd.getStrWidth (NOM_PROGRAMME))/2, 10, NOM_PROGRAMME) ;
  lcd.drawStr (lcd.getWidth ()-lcd.getStrWidth (COPYRIGHT)-5, 20, COPYRIGHT) ;
 }


/*
   Trace la presentation lors de la lecture des fichiers sur carte SD
*/
void erreurLectureSD ()
 {
  presentation () ;
  lcd.setFont (u8g_font_5x8) ;
  lcd.drawStr (11, 50, "Erreur de lecture SD.") ;
  lcd.drawStr (16, 60,  "ARRET DU PROGRAMME.") ;
 }


/*
   Trace la presentation lors de la lecture des fichiers sur carte SD
*/
void lectureSD ()
 {
  presentation () ;
  lcd.setFont (u8g_font_5x8) ;
  lcd.drawStr (9, 50, "Lecture de la carte et") ;
  lcd.drawStr (1, 60, "classement alphabetique..") ;
 }


/*
   Trace la presentation lors de la recherche et classement de fichiers/dossiers
*/
void lectureClassementSD () 
 {
  afficheEntete () ; 
  lcd.drawStr (9, 20, "Lecture de la carte et") ;
  lcd.drawStr (1, 30, "classement alphabetique..") ;
 }


/*
   Affichage de l'entete
*/
void afficheEntete ()
 {
  /*
     u8g_font_5x7
     u8g_font_5x8
     u8g_font_6x10
  */
  lcd.setFont (u8g_font_5x8) ;
  lcd.setFontRefHeightText () ;
  lcd.setFontPosTop () ;
  lcd.setDefaultForegroundColor () ;
  lcd.drawStr (10, 0, NOM_PROGRAMME) ;
  
  switch (profil_machine)
   {
    case 0 : lcd.drawStr (93, 0, "MZ-700") ; break ;
    case 1 : lcd.drawStr (93, 0, "MZ-800") ; break ;
    case 2 : lcd.drawStr (93, 0, "MZ-80B") ; break ;
    case 3 : lcd.drawStr (93, 0, "PERSO ") ; break ;  
   }
  lcd.drawHLine (0, 8, 128) ;
  lcd.drawHLine (0, 53, 128) ;
 }


/*
   Fonction d'affichage du menu de choix du profil a utiliser 
*/
void afficheProfils (uint8_t num_profil)
 {
  lcd.firstPage () ;
  do 
   {
    lcd.setFont (u8g_font_5x8) ;
    lcd.setFontRefHeightText () ;
    lcd.setFontPosTop () ;

    lcd.drawBox (0, 12+10*num_profil, 127, 8) ;
    
    lcd.setDefaultForegroundColor () ;
    
    lcd.drawStr (26, 0, "Choix du profil") ;
    lcd.drawHLine (0, 8, 128) ;

    if (num_profil == 0) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr ( 6, 12, "MZ-700 / MZ80K / MZ-80A") ;
    if (num_profil == 1) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr (49, 22, "MZ-800") ;
    if (num_profil == 2) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr (49, 32, "MZ-80B") ;
    if (num_profil == 3) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr (34, 42, "Personnalise") ;
    lcd.setDefaultForegroundColor () ;
    lcd.drawHLine (0, 53, 128) ;

    lcd.drawStr ( 0, 55, "Bit 0=    us") ;
    lcd.drawStr (68, 55, "Bit 1=    us") ;
    if (num_profil == 3)
     {
      lcd.drawStr (30, 55, long2Char (profil_perso [1])) ; //profil_perso_periode_bit_0)) ;
      lcd.drawStr (98, 55, long2Char (profil_perso [3])) ; //profil_perso_periode_bit_1)) ;
     }
    else
     {
      lcd.drawStr (30, 55, long2Char (durees [num_profil*4+3])) ;
      lcd.drawStr (98, 55, long2Char (durees [num_profil*4+1])) ;
     }
   }
  while (lcd.nextPage ()) ; 
 }

/*
   Fonction permettant de choisir et positionner le profil perso
*/
void choixProfil ()
 {
  boolean sortie = false ;
  while (sortie == false)
   {
    lectureEncodeur () ;
    
    if (menu_retrace != 0 )
     {
      afficheProfils (menu_debut) ;
      menu_retrace = 0 ;
     }
   
    bouton_valeur = digitalRead (BOUTON) ;  
    if (!bouton_valeur)
     {
      // Attend que le bouton soit revenu a sa position initiale
      while (!digitalRead (BOUTON)) { }
      menu_retrace = 1 ;
      sortie = selectionProfil (menu_debut) ;
     }
   }
  profil_machine = menu_debut ;
 }


/*
   Fonction de selection et modification d'un profil personnalise
*/
boolean selectionProfil (uint8_t num_profil)
 {
  boolean selection = false ;
  int choix = 0 ;
  //int nombre = 2 ;
  menu_debut = 0 ;
  menu_nombre = 9999 ;
  if (num_profil == 3) { choix = 2 ; } //nombre = 6 ; }

  while (selection == false)
   {
    if (menu_retrace == 1)
     {
      lcd.firstPage () ;
      do 
       {
        lcd.setFont (u8g_font_5x8) ;
        lcd.setFontRefHeightText () ;
        lcd.setFontPosTop () ;
        lcd.setDefaultForegroundColor () ;

        switch (num_profil)
         {
          case 0 : lcd.drawStr ( 6, 0, "MZ-700 / MZ80K / MZ-80A") ; break ;
          case 1 : lcd.drawStr (49, 0, "MZ-800") ; break ;
          case 2 : lcd.drawStr (49, 0, "MZ-80B") ; break ;
          case 3 : lcd.drawStr (34, 0, "Personnalise") ; break ;
         }
        lcd.drawHLine (0, 8, 128) ;
        
        // Trace du signal carre pour explication
        lcd.drawHLine (  0, 63,  3) ;
        lcd.drawVLine (  2, 40, 23) ;
        lcd.drawHLine (  2, 40, 70) ;
        lcd.drawVLine ( 72, 40, 16) ; lcd.drawVLine ( 72, 59, 4) ;
        lcd.drawHLine ( 72, 63, 53) ;
        lcd.drawVLine (124, 40, 23) ;
        lcd.drawHLine (124, 40,  3) ;

        // Fleches et indicateurs
        lcd.drawHLine (  4, 47, 28) ; lcd.drawHLine ( 40, 47, 31) ;
        lcd.drawVLine (  5, 46,  3) ; lcd.drawVLine (  6, 45,  5) ;
        lcd.drawVLine ( 69, 46,  3) ; lcd.drawVLine ( 68, 45,  5) ;
        lcd.drawStr   ( 34, 43, "H") ;

        lcd.drawHLine (  4, 57, 75) ; lcd.drawHLine ( 86, 57, 37) ;
        lcd.drawVLine (  5, 56,  3) ; lcd.drawVLine (  6, 55,  5) ;
        lcd.drawVLine (121, 56,  3) ; lcd.drawVLine (120, 55,  5) ;
        lcd.drawStr   ( 80, 54, "T") ;

        // Donnees caracteristiques
        lcd.drawStr ( 5, 10,  "Bit 0    Bit 1") ;
        lcd.drawStr ( 0, 20, "H=    us H=    us") ;
        lcd.drawStr ( 0, 30, "T=    us T=    us") ;
        
        // Trace la boite de selection
        switch (choix)
         {
          case 0 :  lcd.drawBox (92, 20, 36, 8) ; break ;
          case 1 :  lcd.drawBox (92, 30, 36, 8) ; break ;
          case 2 :  lcd.drawBox (10, 20, 20, 8) ; break ;
          case 3 :  lcd.drawBox (10, 30, 20, 8) ; break ;
          case 4 :  lcd.drawBox (55, 20, 20, 8) ; break ;
          case 5 :  lcd.drawBox (55, 30, 20, 8) ; break ;
         }

        // Trace les caracteristiques numeriques
        if (num_profil == 3)
         {
          if (choix == 2) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
          lcd.drawStr (9, 20, long2Char (profil_perso [0])) ;
          if (choix == 3) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
          lcd.drawStr (9, 30, long2Char (profil_perso [1])) ;
          if (choix == 4) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
          lcd.drawStr (54, 20, long2Char (profil_perso [2])) ;
          if (choix == 5) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
          lcd.drawStr (54, 30, long2Char (profil_perso [3])) ;
         }
        else
         {
          lcd.setDefaultForegroundColor () ;
          lcd.drawStr (10, 20, long2Char (durees [num_profil*4+2])) ;
          lcd.drawStr (10, 30, long2Char (durees [num_profil*4+3])) ;
          lcd.drawStr (55, 20, long2Char (durees [num_profil*4+0])) ;
          lcd.drawStr (55, 30, long2Char (durees [num_profil*4+1])) ;
         }

        // Trace le menu de choix
        if (choix == 0) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
        lcd.drawStr (93, 20, "CHOISIR") ;
        if (choix == 1) { lcd.setDefaultBackgroundColor () ; } else { lcd.setDefaultForegroundColor () ; }
        lcd.drawStr (95, 30, "RETOUR") ;
       }
      while (lcd.nextPage ()) ;
      menu_retrace = 0 ;
     }

    // Gestion de l'encodeur rotatif
    switch (choix)
     {
      case 0 :
      case 1 : // Test d'abord le bouton
               bouton_valeur = digitalRead (BOUTON) ;  
               if (!bouton_valeur)
                {
                 // Attend que le bouton soit revenu a sa position initiale
                 while (!digitalRead (BOUTON)) { }
                 selection = true ;
                }
               // Test ensuite de l'encodeur
               if (selection == false)
                {
                 encoder.tick () ;
                 encodeurNouvellePosition = encoder.getPosition () ;
                 if (encodeurDernierePosition-encodeurNouvellePosition > 0)
                  {
                   choix++ ;
                   if (choix == 2) {  if (num_profil < 3) { choix = 0 ; } }
                   menu_retrace = 1 ;
                  }
                 else if (encodeurDernierePosition-encodeurNouvellePosition < 0)
                  {
                   if (choix == 0) {  if (num_profil == 3) { choix = 6 ; } else { choix = 2 ; } }
                   choix-- ;
                   menu_retrace = 1 ;
                  }
                 encodeurDernierePosition = encodeurNouvellePosition ;
                }
               break ;
      case 2 :
      case 3 :
      case 4 :
      case 5 : // Test d'abord le bouton
               bouton_valeur = digitalRead (BOUTON) ;  
               if (!bouton_valeur)
                {
                 // Attend que le bouton soit revenu a sa position initiale
                 while (!digitalRead (BOUTON)) { }
                 // On verifie que la periode ne puisse pas etres inferieure au temsp haut
                 if ((choix == 2) || (choix == 4))
                  {
                   if (profil_perso [choix+1] < profil_perso [choix]) { profil_perso [choix+1] = profil_perso [choix] ; }
                  }
                 choix++ ;
                 if (choix == 6) { choix = 0 ; }
                 menu_retrace = 1 ;
                }
               // Test ensuite de l'encodeur
               if (selection == false)
                {
                 encoder.tick () ;
                 encodeurNouvellePosition = encoder.getPosition () ;
                 if (encodeurDernierePosition-encodeurNouvellePosition > 0)
                  {
                   profil_perso [choix-2]++ ;
                   // On verifie que la periode ne puisse pas etres inferieure au temsp haut
                   if ((choix == 3) || (choix == 5))
                    {
                     if (profil_perso [choix-2] > 9999) { profil_perso [choix-2] = profil_perso [choix-3] ; }
                    }
                   else if (profil_perso [choix-2] > 9999) { profil_perso [choix-2] = 0 ; }
                   menu_retrace = 1 ;
                  }
                 else if (encodeurDernierePosition-encodeurNouvellePosition < 0)
                  {
                   // On verifie que la periode ne puisse pas etres inferieure au temsp haut
                   if ((choix == 3) || (choix == 5))
                    {
                     if (profil_perso [choix-2] <= profil_perso [choix-3]) { profil_perso [choix-2] = 10000 ; }
                    }
                   else if (profil_perso [choix-2] == 0) { profil_perso [choix-2] = 10000 ; }
                   profil_perso [choix-2]-- ;
                   menu_retrace = 1 ;
                  }
                 encodeurDernierePosition = encodeurNouvellePosition ;
                }
               break ;
     }
    
   }
  menu_debut = num_profil ;
  menu_nombre = 4 ;
  if (choix == 0) { return true ; } else { menu_retrace = 1 ; return false ; }
 }


/*
  Trace le menu de choix des fichiers
*/
void listeChoixFichiers (void)
 {
  uint8_t i, h ;
  int8_t num_menu = 0 ;

  h = lcd.getFontAscent()-lcd.getFontDescent() ;

  afficheEntete () ;
    
  if (menu_debut < -MENU_SELECTION) { menu_debut = menu_nombre-MENU_SELECTION ; }
  if (menu_debut > menu_nombre-1) { menu_debut = 0 ; }
  for (i = 0 ; i < MENU_NOMBRE_AFFICHAGE ; i++)
   {
    if ((menu_debut+i >= 0) and (menu_debut+i < menu_nombre)) { num_menu = menu_debut+i ; }
    else
     {
           if            (menu_debut+i < 0) { num_menu = menu_nombre+menu_debut+i ; }
      else if (menu_debut+i >= menu_nombre)  { num_menu = (menu_debut+i) % menu_nombre ; }
     }
    if (menuNum [num_menu] >= 0) { obtientEntree (menuNum [num_menu]) ; }
    else { entree_type = TYPE_FORMAT_RETOUR_DOSSIER ; }
    
    lcd.setDefaultForegroundColor () ;
    if (i == MENU_SELECTION)
     {
      lcd.drawBox (0, i*h+MENU_POS_Y+1, 127, h) ;
      
      menu_courant = num_menu ;

           if (entree_type == TYPE_FORMAT_REPERTOIRE)     { lcd.drawStr (46, 55, "Dossier") ; }
      else if (entree_type == TYPE_FORMAT_RETOUR_DOSSIER) { lcd.drawStr (11, 55, "Retour dossier parent") ; }
      else
       {
        lcd.drawStr (0, 55, uint32_t2Char (entree_taille)) ;
        lcd.drawStr (35, 55, "octets") ;
        
        typeFichierMZ () ;
       
             if (mzf_type == MZF_BINAIRE)             { lcd.drawStr (68, 55, "BIN OBJ AE ") ; }
        else if (mzf_type == MZF_BINAIRE_PURE)        { lcd.drawStr (68, 55, "BINAIRE SE ") ; }
        else if (mzf_type == MZF_BINAIRE_MZF1)        { lcd.drawStr (68, 55, "BIN  MZF1  ") ; }
        else if (mzf_type == MZF_KUMA_INT)            { lcd.drawStr (68, 55, "KUMA BASIC ") ; }
        else if (mzf_type == MZF_KUMA_COMPILER)       { lcd.drawStr (68, 55, "KUMA OBJ   ") ; }
        else if (mzf_type == MZF_BASIC_MZ700)         { lcd.drawStr (68, 55, "BASIC MZ700") ; }
        else if (mzf_type == MZF_DATA_MZ700)          { lcd.drawStr (68, 55, "DATA MZ700 ") ; }
        else if (mzf_type == MZF_BASIC_MZ80)          { lcd.drawStr (68, 55, "BASIC MZ80 ") ; }
        else if (mzf_type == MZF_DATA_MZ80)           { lcd.drawStr (68, 55, "DATA MZ80  ") ; }
        else                                          { lcd.drawStr (68, 55, "  ??????   ") ; }
       }
      
      lcd.setDefaultBackgroundColor () ;
     }
    
    if (entree_type == TYPE_FORMAT_REPERTOIRE)
     {
      lcd.drawBitmapP (0, i*h+MENU_POS_Y, 1, 8, icone_dossier) ;
      lcd.drawStr (10, i*h+MENU_POS_Y, nom_fichier_long) ;
     }
    else if (entree_type == TYPE_FORMAT_RETOUR_DOSSIER)
     {
      lcd.drawBitmapP (0, i*h+MENU_POS_Y, 1, 8, icone_remonte) ;
      lcd.drawStr (10, i*h+MENU_POS_Y, "RETOUR DOSSIER PARENT") ;
     }
    else
     {
      lcd.drawStr (10, i*h+MENU_POS_Y, nom_fichier_long) ;
     }
   }
 }

/*
   Affiche les proprietes du fichier selectionne
*/
void proprietesFichier (uint8_t choix) 
 {
  u8g_uint_t w, d ;
  
  w = lcd.getWidth () ;

  afficheEntete () ;
  
  obtientEntree (menuNum [menu_courant]) ;
  
  d = (w-lcd.getStrWidth (nom_fichier_long))/2 ;
  
  typeFichierMZ () ;
  lcd.drawStr (d, 10, nom_fichier_long) ;
  lcd.drawStr (0, 18, uint32_t2Char (entree_taille)) ;
  lcd.drawStr (35, 18, "octets") ;
  
       if (mzf_type == MZF_BINAIRE)             { lcd.drawStr (68, 18, "BIN OBJ AE ") ; }
  else if (mzf_type == MZF_BINAIRE_PURE)        { lcd.drawStr (68, 18, "BINAIRE SE ") ; }
  else if (mzf_type == MZF_BINAIRE_MZF1)        { lcd.drawStr (68, 18, "BIN  MZF1  ") ; }
  else if (mzf_type == MZF_KUMA_INT)            { lcd.drawStr (68, 18, "KUMA BASIC ") ; }
  else if (mzf_type == MZF_KUMA_COMPILER)       { lcd.drawStr (68, 18, "KUMA OBJ   ") ; }
  else if (mzf_type == MZF_BASIC_MZ700)         { lcd.drawStr (68, 18, "BASIC MZ700") ; }
  else if (mzf_type == MZF_DATA_MZ700)          { lcd.drawStr (68, 18, "DATA MZ700 ") ; }
  else if (mzf_type == MZF_BASIC_MZ80)          { lcd.drawStr (68, 18, "BASIC MZ80 ") ; }
  else if (mzf_type == MZF_DATA_MZ80)           { lcd.drawStr (68, 18, "DATA MZ80  ") ; }
  else                                          { lcd.drawStr (68, 18, "  ??????   ") ; }
  
  lcd.drawStr (10, 28, "Taille    = ") ;   lcd.drawStr (70, 28, uint32_t2Char (mzf_taille)) ; lcd.drawStr (105, 28, "oct") ;
  lcd.drawStr (10, 36, "Adresse   =  $") ; lcd.drawStr (80, 36, uint16_t2Hexa (mzf_adresse)) ;
  lcd.drawStr (10, 44, "Execution =  $") ; lcd.drawStr (80, 44, uint16_t2Hexa (mzf_execution)) ;

  if (choix < 2)
   {
    if (choix == 0)
     {
      lcd.setDefaultForegroundColor () ;
      lcd.drawBox (0, 54, 45, 63) ;
      lcd.setDefaultBackgroundColor () ;
     }
    else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr (5, 55, "LECTURE") ;
  
    if (choix == 1)
     {
      lcd.setDefaultForegroundColor () ;
      lcd.drawBox (88, 54, 40, 63) ;
      lcd.setDefaultBackgroundColor () ;
     }
    else { lcd.setDefaultForegroundColor () ; }
    lcd.drawStr (93, 55, "RETOUR") ;
   }
 }


/*
   Aiguillage de l'affichage
*/
void afficheEcran (uint8_t menu_type)
 {
  lcd.firstPage () ;
  do 
   {
    switch (menu_type)
     {
      case AFFICHE_PRESENTATION : presentation () ; break ;
      case AFFICHE_ERREUR_SD    : erreurLectureSD () ; break ;
      case AFFICHE_LECTURE_SD   : lectureSD () ; break ;
      case AFFICHE_LC_SD        : lectureClassementSD () ; break ;
      case AFFICHE_LISTE_CHOIX  : listeChoixFichiers () ; break ;
      case AFFICHE_PROPRIETES   : menu_debut = 0 ;
                                  proprietesFichier (menu_debut) ;
                                  // Reinitialisation encodeur
                                  encoder.setPosition (0) ;
                                  encodeurDernierePosition = 0 ;
                                  menu_lecture = 1 ;
                                  menu_nombre = 2 ;
                                  menu_retrace = 0 ;
                                  break ;
     }
   }
  while (lcd.nextPage ()) ;
 }


/*
  =================================================================================
   Fonctions de gestion de l'encodeur rotatif
  =================================================================================
*/
void lectureEncodeur (void)
 {
  encoder.tick () ;
  encodeurNouvellePosition = encoder.getPosition () ;
  if (encodeurDernierePosition-encodeurNouvellePosition > 0)
   {
    menu_debut++ ;
    if (menu_debut >= menu_nombre) { menu_debut = 0 ; }
    menu_retrace = 1+menu_lecture ;
   }
  else if (encodeurDernierePosition-encodeurNouvellePosition < 0)
   {
    if (menu_debut == 0) { menu_debut = menu_nombre ; }
    menu_debut-- ;
    menu_retrace = 1+menu_lecture ;
   }
  encodeurDernierePosition = encodeurNouvellePosition ;
 }

/*
   Fonction de selection et declenchement procedure de lecture
*/
void selection ()
 {
  switch (menu_lecture)
   {
    case 0 : if (menuNum [menu_courant] >= 0) { obtientEntree (menuNum [menu_courant]) ; }
             else { entree_type = TYPE_FORMAT_RETOUR_DOSSIER ; }
             if ((entree_type == TYPE_FORMAT_REPERTOIRE) || (entree_type == TYPE_FORMAT_RETOUR_DOSSIER))
              {
               entree_index = menuNum [menu_courant] ;
               //Serial.print ( " Avant > ") ; Serial.print (menu_debut) ; Serial.print ( " > ") ; Serial.println (menu_courant) ;
               if (entree_type == TYPE_FORMAT_REPERTOIRE)
                {
                 // On descent dans un sous-dossier
                 entrerRepertoire () ;
                 menu_debut = -1 ;
                }
               else
                {
                 // On remonte a un sous-dossier ou dossier parent
                 sortirRepertoire () ;
                 menu_debut = entree_index ;
                }

               afficheEcran (AFFICHE_LC_SD) ;
               rechercheFichiers () ;
               
               if (sd_rep_profondeur > 0)
                {
                 menuNum [nombre_fichiers] = -1 ;
                 nombre_fichiers++ ;
                }

               menu_nombre = nombre_fichiers ;
               if (menu_debut == -1) { menu_debut = menu_nombre-MENU_SELECTION ; }

               //Serial.print ( " Apres > ") ; Serial.print (menu_debut) ; Serial.print ( " > ") ; Serial.println (menu_courant) ;
               //Serial.println () ;
               
               menu_retrace = 1 ;
              }
             else
              {
               menu_debut_sauve = menu_debut ;
               afficheEcran (AFFICHE_PROPRIETES) ;
              }
             break ;
    case 1 : switch (menu_debut)
              {
               case 0 : // Lecture du fichier
                        digitalWrite (LED, HIGH) ;
                        digitalWrite (MZ_CASSETTE_SENSE, LOW) ; // signal /SENSE a 0 (Lecteur disponible)
                        lectureFichierMZF () ;
                        digitalWrite (MZ_CASSETTE_SENSE, HIGH) ; // signal /SENSE a 1 (Lecteur non disponible)
                        digitalWrite (LED, LOW) ;
                        menu_debut = menu_debut_sauve ;
                        menu_nombre = nombre_fichiers ;
                        encoder.setPosition (0) ;
                        encodeurDernierePosition = 0 ;
                        menu_retrace = 1 ;
                        menu_lecture = 0 ;
                        break ;
               case 1 : // retour menu
                        menu_debut = menu_debut_sauve ;
                        menu_nombre = nombre_fichiers ;
                        encoder.setPosition (0) ;
                        encodeurDernierePosition = 0 ;
                        menu_lecture = 0 ;
                        menu_retrace = 1 ;
                        break ; 
              }
   }
 }

/*
  =================================================================================
    Fonction d'initialisation generale
  =================================================================================
*/
void setup()
 {
  afficheEcran (AFFICHE_PRESENTATION) ;
  
  Serial.begin (INTERFACE_SERIE_BAUD) ;
  Serial.println (NOM_PROGRAMME) ;
  Serial.println (COPYRIGHT) ;

  // Attente pour pouvoir appuyer sur le bouton si besoin
  delay (500) ;

  // Initialisation bouton encodeur rotatif
  pinMode (BOUTON, INPUT) ;
  digitalWrite (BOUTON, HIGH) ;

  // Initialisation interface lecteur SD
  pinMode (SD0_SS, OUTPUT) ;

  // Initialisation interface Cassette MZ
  pinMode (MZ_CASSETTE_REMOTE, INPUT_PULLUP) ;
  
  pinMode (MZ_CASSETTE_SENSE, OUTPUT) ;
  digitalWrite (MZ_CASSETTE_SENSE, HIGH) ; // signal /SENSE a 1 (Lecteur non disponible)
    
  pinMode (MZ_CASSETTE_WRITE, INPUT_PULLUP) ;
  
  pinMode (MZ_CASSETTE_READ, OUTPUT) ;
  digitalWrite (MZ_CASSETTE_READ, HIGH) ;  // Signal de donnee a 1

  // Initialisation LED : Pas d'activite donc eteinte
  pinMode (LED, OUTPUT) ;
  digitalWrite (LED, LOW) ;

  menu_retrace = 1 ;
  encoder.setPosition (0) ;

  // ==================================== BOOT ========================================
  //  Si bouton appuye pendant plus de DUREE_BOOT s ou premier boot alors choix profil
  //
  int bouton_boot = 0 ;
  if (!digitalRead (BOUTON))
   {
    date_debut = millis () ;
    while ((!digitalRead (BOUTON)) && (millis () <= date_debut + DUREE_BOOT*1000)) { }
    date_fin = millis () ;
    if (date_fin-date_debut >= 5*1000) { bouton_boot = 1 ; }
   }
  if ((EEPROM.read (EEPROM_CHOIX_PROFIL) == 255) || (bouton_boot == 1))
   {
    // On choisi le profil machine au premier boot ou a la demande
    afficheProfils (profil_machine) ;
    // Attente relachement bouton
    while (!digitalRead (BOUTON)) { }
    menu_nombre = 4 ;
    menu_lecture = 0 ;
    menu_debut = profil_machine ;
    choixProfil () ;
    ecritureProfil () ;
    asm volatile ("jmp 0") ; // Reboot
   }
  else { lectureProfil () ; }
  // ==================================================================================

  // Lecture sur carte SD
  sd_pret = sd.begin (SD0_SS, SPI_FULL_SPEED) ;
  if (!sd_pret) // accès au SD en full-speed
   {
    sd.initErrorHalt () ;
    Serial.println ("Pas de carte SD !") ;
    afficheEcran (AFFICHE_ERREUR_SD) ;
    // Boucle infinie
    while (true) { }
   }

  // Lecture des fichiers et stockage numero
  afficheEcran (AFFICHE_LECTURE_SD) ;
  entrerRepertoire () ;
  rechercheFichiers () ;
  
  // Initialisations pour affichage
  menu_debut = menu_nombre-MENU_SELECTION ;
  calculDurees (profil_machine) ;
 }


/*
  =================================================================================
    Programme principal
  =================================================================================
*/
void loop ()
 {
  // Gestion affichage
  if (menu_retrace != 0 )
   {
    switch (menu_retrace)
     {
      case 1 : afficheEcran (AFFICHE_LISTE_CHOIX) ;
               break ;
      case 2 : lcd.firstPage () ;
               do { proprietesFichier (menu_debut) ; } while (lcd.nextPage ()) ;
               break ;
      case 3 : lcd.firstPage () ;
               do { proprietesFichier (2) ; } while (lcd.nextPage ()) ;
               break ;
     }
    menu_retrace = 0 ;
   }

  // Gestion encodeur rotatif
  lectureEncodeur () ;
  bouton_valeur = digitalRead (BOUTON) ;  
  if (!bouton_valeur)
   {
    // Attend que le bouton soit revenu a sa position initiale
    date_debut = millis () ;
    while ((!digitalRead (BOUTON)) && (millis () <= date_debut + DUREE_INTERFACE*1000)) { }
    date_fin = millis () ;
    
    if (date_fin-date_debut >= DUREE_INTERFACE*1000)
     {
      // Revient au choix du profil machine si appuie long (> 3s)
      afficheProfils (profil_machine) ;
      // Attente relachement bouton
      while (!digitalRead (BOUTON)) { }
      menu_nombre = 4 ;
      menu_lecture = 0 ;
      menu_debut = profil_machine ;
      choixProfil () ;
      ecritureProfil () ;
      // On recommence l'intialisation apres selection du nouveau profil
      calculDurees (profil_machine) ;
      menu_nombre = nombre_fichiers ;
      menu_debut = menu_nombre-MENU_SELECTION ;
      menu_retrace = 1 ;
     }
    else { selection () ; }
   }
 }
