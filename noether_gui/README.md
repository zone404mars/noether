# noether_gui

> La spécification du fork est `SPECIFICATIONS.md`, à la racine du dépôt. Ce README est le mode d'emploi détaillé ; en cas de désaccord, la spécification fait foi.

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
| 3 Traitement | Tracé manuel, Générer, Éditer les points | pas du tracé, hauteur d'approche, rayon de l'outil ; points choisis, Supprimer, densification par pas ou par nombre |
| 4 Trajectoire | Ordre des passes, Exporter | liste des passes, monter, descendre, supprimer, annuler, rétablir |
| Affichage | Calques, Vues standard, Langue | calques dessinés, disques (pleins, surélévation, zones non couvertes), vues face, dessus, côté, isométrique |

Les outils qui prennent la souris dans la vue s'excluent : Région, Brosse, Polygone,
Gomme, Tracé manuel et Éditer les points. En choisir un relâche le précédent.

Un outil de sélection est toujours armé. Tracé manuel et Éditer les points sont des modes
temporaires : ils rendent la souris au dernier outil de sélection quand on choisit un autre
outil, ou quand on entre dans les onglets Fichier, Trajectoire ou Affichage.

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

## 5. Éditer les points

Outil de l'onglet Traitement. Avant génération, il édite les sommets du tracé de la surface
courante ; après génération, les poses de contact du chemin livré. L'approche et le retrait
ne se saisissent pas : ils suivent la première et la dernière pose de contact.

| geste | effet |
|---|---|
| survol | le point à moins de 10 px du curseur, visible, est éclairé en blanc |
| clic | choisit ce point seul ; clic dans le vide : vide la sélection |
| Ctrl + clic | ajoute ou retire le point |
| glisser depuis un point | déplace les points choisis, posés sur leur surface ; un seul Annuler pour tout le glisser |
| Suppr | supprime les points choisis |
| Échap | vide la sélection |
| Ctrl+Z, Ctrl+Y | annuler, rétablir |

Après un clic qui laisse une sélection, une barre apparaît près du curseur (Supprimer,
Densifier). Une action indisponible est grisée ; sa bulle d'aide dit pourquoi.

**Densifier** demande deux points A et B d'une même passe générée. Chaque écart entre deux
poses voisines de A à B reçoit des points, en mode Pas (pas cible, 10 mm par défaut) ou en
mode Nombre (N points par écart, 1 par défaut). Un aperçu les montre avant de valider. Un
point est posé sur la surface qui a donné la passe, sur la face la plus proche tournée du
même côté que les deux poses voisines (spec §21.1) : il ne tombe jamais sur la face opposée
d'une tôle mince. Un écart sans place de ce côté est sauté, et la barre d'état le dit.

Un chemin repris n'a pas de surface : ses points se suppriment, mais ne se densifient ni
ne se déplacent.

Le code est dans `src/widgets/tpp_widget_edit_points.cpp` (gestes),
`tpp_widget_point_actions.cpp` (actions) et `tpp_widget_point_view.cpp` (panneau, barre,
marques). La logique sans Qt est dans `point_edit`, `point_picking`, `pass_surface` et
`edited_path`.

## 5 bis. Cube de navigation

En haut à droite de la vue. Clic sur une face, une arête ou un coin : la caméra se place
sur l'axe, la bissectrice ou la diagonale, et la vue est recadrée. Glisser sur le cube :
rotation libre. Vue alignée sur une face à moins de 1° : quatre flèches vers les faces
voisines et deux flèches courbes (quart de tour). Le bouton maison donne la vue
isométrique. Les libellés sont en majuscules, sans accent : le texte des faces est dessiné
par `vtkVectorText`, qui ne connaît que l'ASCII.

Code : `src/widgets/view_cube.cpp` (calcul), `src/widgets/tpp_widget_view_cube.cpp` (dessin
et boutons).

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
| `noether_gui_path_edit_widget_utest` | panneau Ordre : monter, descendre, supprimer, annuler, rétablir, état modifié |
| `noether_gui_pass_surface_utest` | pose d'un point sur la surface de la passe, test du même côté, tôle cintrée |
| `noether_gui_edited_path_utest` | chemin livré édité : actions, annuler, rétablir, suppression de poses |
| `noether_gui_point_edit_utest` | sélection, densification par pas ou par nombre, glisser, saisie à l'écran |
| `noether_gui_view_cube_utest` | zones du cube, haut de l'écran, flèches, quart de tour, alignement à 1° |
| `noether_gui_tpp_widget_utest` | la fenêtre complète se construit |
| `noether_gui_trace_tool_utest` | points du tracé manuel, approche et retrait |
| `noether_gui_outline_tool_utest` | contour du polygone |
| `noether_gui_pipeline_direction_utest` | direction donnée au planificateur raster |

La disposition de la fenêtre, le dessin des disques, la couche de couverture, les gestes
dans la vue et le dessin du cube n'ont pas de test automatique : ils se vérifient à
l'écran.
