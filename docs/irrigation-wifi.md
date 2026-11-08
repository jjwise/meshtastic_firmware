# Plusieurs reseaux Wi-Fi pour la passerelle

Dans le fichier local ignore par Git
`src/modules/irrigation/IrrigationGatewaySecrets.generated.h`, ajouter une liste
ordonnee. Les noms et mots de passe ci-dessous sont des exemples a remplacer :

```cpp
#define IRRIGATION_WIFI_NETWORKS { \
  {IRRIGATION_WIFI_SSID, IRRIGATION_WIFI_PASSWORD}, \
  {"SSID_SECONDAIRE", "MOT_DE_PASSE_SECONDAIRE"}, \
  {"SSID_TERTIAIRE", "MOT_DE_PASSE_TERTIAIRE"} \
}
```

La premiere entree conserve les identifiants existants. La liste peut aussi
contenir uniquement des couples de chaines, sans les anciennes macros.
Sans IRRIGATION_WIFI_NETWORKS, la paire IRRIGATION_WIFI_SSID/PASSWORD reste
utilisee comme auparavant. Ne jamais commiter ce fichier ni coller ses valeurs
dans les logs de diagnostic.

Au demarrage, les entrees sont essayees dans l'ordre de la liste, sans choisir
le signal le plus fort. Chaque reseau dispose de 20 secondes, incluant l'attente
native de 5 secondes avant l'association. Une connexion reussie est conservee.
Apres une perte, la passerelle retente ce reseau, puis passe au suivant apres
20 secondes sans connexion. Si aucun ne fonctionne, elle reprend la liste en
boucle. Le traitement utilise le gestionnaire Wi-Fi Meshtastic existant et ne
bloque pas la boucle radio par une attente ou un scan synchrone.

Une connexion Wi-Fi avec adresse IP suffit a retenir le reseau. Cette selection
ne teste pas l'acces Internet, le DNS ou la disponibilite MQTT.
Les SSID sont compares tels quels (UTF-8 inclus). Les entrees vides, nulles ou
trop longues sont ignorees : maximum 32 octets pour le SSID et 64 pour le mot de
passe. Un mot de passe vide signifie un reseau ouvert. Deux entrees peuvent
avoir le meme SSID avec des mots de passe differents. Les nouveaux logs affichent
le numero d'entree, jamais ses identifiants.

Recompiler et flasher le profil `hwt_gw` pour appliquer une modification de liste.
La liste compilee remplace la paire Wi-Fi pour ce profil, meme si l'ancienne
configuration irrigation a ete enregistree. Aucune suppression de configuration
ou de memoire anti-rejeu n'est necessaire.

Validation logicielle : `python3 bin/test-irrigation-wifi.py` compile le selecteur,
le callback natif de reconnexion et Throttle contre un transport simule. Essai
sur banc requis : premier reseau absent ou mot de passe incorrect, second reseau
disponible, maintien de connexion puis bascule apres disparition du reseau.
