# noether_gui

Interface de préparation des chemins d'outil : on ouvre une pièce, on y choisit des
surfaces, on trace un chemin à la main sur chacune, on génère les passes, on les retouche,
puis on exporte le chemin.

L'application est `noether_gui_app`. La fenêtre est la classe `TPPWidget`.

---

## 1. Disposition de la fenêtre

```
+--------------------------------------------------------------------------+
| Fichier  Affichage  Génération                                  (menus)  |
| [1 Fichier] [2 Sélection] [3 Traitement] [4 Trajectoire] [Affichage]     |
|  outils de l'onglet courant, icône et libellé                            |
+-------------+------------------------------------------+-----------------+
| Arbre       |                                          | Options         |
|  pièce      |              vue 3D (VTK)                |  titre          |
|  Surfaces   |                                          |  aide           |
|   Surface 1 |                                          |  champs de      |
|    tracé    |                                          |  l'outil        |
|  Trajectoire|                                          |                 |
|   Passe 1   |                                          |                 |
| [Reprendre] |                                          |                 |
| [Supprimer] |                                          |                 |
+-------------+------------------------------------------+-----------------+
| message de l'action en cours                   étendue de la pièce chargée |
+--------------------------------------------------------------------------+
```

| zone | classe | rôle |
|---|---|---|
| barre du process | `ProcessRibbon` | un onglet par étape, une rangée d'outils par onglet |
| arbre | `QTreeWidget` du dock « Arbre » | pièce, surfaces et leur tracé, passes générées |
| options | `QStackedWidget` du dock « Options » | une page par outil : titre, aide, réglages |
| barre d'état | étiquette permanente | étendue du maillage, et unité retenue pour un STEP |

Le code de la disposition est dans `src/widgets/tpp_widget_layout.cpp`. La sélection, le
tracé et la génération sont dans `src/widgets/tpp_widget.cpp`.

La barre d'outils et le dock de configuration du pipeline du fichier `.ui` sont cachés.
Leurs actions restent accessibles par les menus.

---

## 2. Les onglets

| onglet | outils | page d'options |
|---|---|---|
| 1 Fichier | Ouvrir une pièce, Reprendre un chemin | formats acceptés, finesse STEP du projet |
| 2 Sélection | Région, Brosse, Polygone, Gomme, puis Valider la surface | angle de planéité, diamètre de la brosse |
| 3 Traitement | Tracé manuel, Générer, puis Déplacer des points, Densifier | pas du tracé, hauteur d'approche, rayon de l'outil ; pas cible de la densification |
| 4 Trajectoire | Ordre des passes, Exporter | liste des passes, monter, descendre, supprimer, annuler |
| Affichage | Calques, Vues standard, Langue | calques dessinés, disques (pleins, surélévation, zones non couvertes), vues face, dessus, côté, isométrique |

Les outils qui prennent la souris dans la vue s'excluent : Région, Brosse, Polygone,
Gomme, Tracé manuel, Déplacer des points et Densifier. En choisir un relâche le précédent.

Un outil de sélection est toujours armé. Tracé manuel, Déplacer des points et Densifier
sont des modes temporaires : ils rendent la souris au dernier outil de sélection quand on choisit
un autre outil, quand on valide la disposition des points déplacés, ou quand on entre
dans les onglets Fichier, Trajectoire ou Affichage.

Changer d'onglet affiche la page d'options de l'onglet. Aucune action n'est lancée, à
part ce changement d'outil de souris :
- l'onglet Sélection reprend le dernier outil de sélection ;
- l'onglet Traitement active le Tracé manuel.

---

## 3. L'arbre

```
pièce                         nom du fichier chargé
Surfaces (n)
  [x] Surface k · f faces     case cochée : surface dessinée en couleur
        Tracé, s sommets      ou « sans tracé »
Trajectoire (p passes)
  Passe i · m poses           clic : la passe est choisie dans la liste des passes
```

- **Valider la surface** (onglet 2) : la sélection en cours, en orange, devient une
  surface. Elle est refusée si elle est faite de plusieurs morceaux.
- **Reprendre** : les faces de la surface courante repassent dans la sélection en cours,
  pour être retouchées puis validées de nouveau. Son tracé est perdu.
- **Supprimer** : la surface et son tracé disparaissent, ses faces redeviennent libres.
- **Case décochée** : la surface est dessinée comme une face libre. Elle est toujours
  générée.

Le tracé manuel s'applique à la surface courante de l'arbre. Choisir l'élément du tracé
revient à choisir sa surface.

---

## 4. Disques et couverture

Le disque de l'outil est dessiné autour de chaque point du tracé en cours, et autour de
chaque point de contact du chemin livré. Réglages, page Affichage, Calques :

| réglage | effet |
|---|---|
| Disques pleins | coché : disques pleins semi-transparents ; décoché : seulement leur bord |
| Surélévation | décalage des disques le long de l'axe de l'outil, pour l'affichage seulement |
| Zones non couvertes | montre ou cache la couche des zones que le disque n'atteint pas |

L'analyseur de couverture mesure ce que le disque atteint :
- pendant le tracé manuel : le tracé de la surface courante, contre cette surface ;
- sinon, après génération : le chemin livré, retouches comprises, contre toutes les surfaces
  tracées. L'approche et le retrait ne comptent pas.

Une zone est couverte quand elle est à moins du rayon d'outil d'un tronçon entre deux
points voisins (`computeCoverage` de noether_tpp). Le disque balaie le tronçon : ce n'est
pas seulement l'union des cercles dessinés. Les faces sont découpées en triangles d'au
plus un sixième du rayon d'outil de côté, pour que le résultat ne dépende pas de la taille
des triangles du maillage. Les zones non couvertes sont dessinées en vert citron, juste
au-dessus de la pièce. La barre d'état donne la part couverte de l'aire et l'aire non
couverte.

La mesure est refaite à la fin de chaque tirage, pas pendant : sur une grande surface, elle
ralentirait le geste. Elle n'est refaite que si le chemin mesuré, les faces ou le rayon
changent : le style et la surélévation des disques ne la relancent pas. Au-delà de
200 000 triangles de mesure, ils sont agrandis : la résolution baisse au lieu du temps de
calcul.

---

## 5. Densifier

Outil de l'onglet Traitement, après une génération :

1. cliquer un premier point de contact du chemin (A), puis un second (B) sur la même passe
   générée ;
2. régler le pas cible dans les options ;
3. cliquer « Densifier ».

Chaque écart entre deux points voisins de A à B, plus long que le pas cible, reçoit des
points régulièrement répartis. Les points existants ne bougent pas. Les nouveaux points sont
posés sur la pièce (projection sur la face la plus proche du maillage entier ; près d'un
angle vif ou sur une pièce mince, cette face peut appartenir à une surface voisine) et
orientés comme ceux du tracé :
axe z sur la normale, axe x dans le sens de marche. « Annuler » de la page Ordre retire
toute la densification d'un coup. Densifier de nouveau le même écart remplace les points
ajoutés ; un écart déjà sous le nouveau pas perd ses points ajoutés.

Densifier demande la pièce ouverte : un chemin repris sans sa pièce est refusé. Pendant
Densifier, le glisser tourne la vue.

Les points ajoutés sont enregistrés tels quels dans la retouche (fichier de version 3, champ
`insertions`). Ils ne suivent pas un déplacement ultérieur de leurs voisins, et ne se
saisissent pas avec Déplacer des points : pour les changer, il faut redensifier.

---

## 6. Langue

L'interface existe en français et en anglais. La langue se choisit dans Affichage, Langue.
Elle est gardée dans les réglages de l'application (`QSettings`, clé `language`, valeurs
`fr` ou `en`). Le français est la langue par défaut.

La classe `Translator` lie un texte bilingue (`Bilingual{french, english}`) à un bouton,
une étiquette, une action, un menu ou un dock. Elle rejoue toutes les liaisons au
changement de langue. Les textes calculés (arbre, étendue) sont recalculés.

Ne changent pas de langue :
- le titre de la fenêtre, fixé par l'application ;
- un message déjà affiché dans la barre d'état : le message suivant est dans la nouvelle
  langue ;
- les messages d'erreur qui viennent des bibliothèques (noether_cad, noether_tpp, lecture
  YAML) ;
- les widgets de configuration du pipeline, cachés dans cette interface.

---

## 7. Ouvrir une pièce

Formats acceptés : STEP (`.step`, `.stp`), PLY, STL.

Un STEP est converti en maillage par `noether_cad` (voir son README), à la finesse lue
dans un fichier YAML passé au lancement :

```bash
noether_gui_app --mesh <piece.step|ply|stl> --step-tessellation <finesse.yaml>
```

```yaml
linear_deflection_m: <metres>
angular_deflection_rad: <radians>
solids: all          # ou one : un fichier a plusieurs solides est alors refuse
```

Sans ce fichier, un STEP est refusé : la finesse n'a pas de valeur par défaut. Le fichier
est relu à chaque ouverture. Le PLY et sa fiche YAML s'écrivent à côté du STEP.

Un PLY ou un STL est chargé tel quel, sans conversion d'unité : il est supposé en mètres.
L'étendue affichée dans la barre d'état est le contrôle d'une unité fausse.

---

## 8. Tests

`-DNOETHER_ENABLE_TESTING=ON`, puis `ctest` dans le dossier de build.

| exécutable | ce qui est vérifié |
|---|---|
| `noether_gui_translator_utest` | choix du texte selon la langue, liaisons rejouées, langue gardée ; onglets et outils de `ProcessRibbon`, leur traduction |
| `noether_gui_step_import_utest` | lecture de la finesse STEP, refus des valeurs invalides, résumé de conversion dans les deux langues |
| `noether_gui_path_edit_widget_utest` | retouches du chemin : ordre, suppression, points déplacés, points ajoutés par densification, annulation, relecture |
| `noether_gui_trace_tool_utest` | points du tracé manuel, approche et retrait |
| `noether_gui_outline_tool_utest` | contour du polygone |
| `noether_gui_pipeline_direction_utest` | direction donnée au planificateur raster |

La disposition de la fenêtre, le dessin des disques et la couche de couverture n'ont pas de
test automatique : ils se vérifient à l'écran. L'application des points ajoutés et leur
lecture et écriture dans le fichier de retouche sont testées dans noether_tpp
(`tool_path_recipe_utest`).
