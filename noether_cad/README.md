# noether_cad

> La spécification du fork est `SPECIFICATIONS.md`, à la racine du dépôt. Ce README est le mode d'emploi détaillé ; en cas de désaccord, la spécification fait foi.

Conversion d'un STEP en maillage pour Noether.

Le paquet lit le **STEP d'une piece** et en tire un maillage **en metres**, dans un
**repere maitrise**. Chaque triangle reste rattache a la **face CAO exacte** dont il
provient.

C'est le seul maillon qui detient la geometrie exacte. Tout ce qui vient apres travaille
sur des triangles.

---

## 1. Perimetre

Ce que le paquet fait :

- lire un STEP et appliquer l'unite qui y est declaree, pour sortir des metres
- mailler a une finesse imposee par l'appelant
- construire la table des faces : chaque triangle sait de quelle face STEP il vient
- recentrer la piece et **ecrire la transformation appliquee**
- produire un maillage PLY et sa fiche YAML

Ce que le paquet ne fait pas :

| tache | responsable |
|---|---|
| choisir les surfaces a traiter, tracer et generer le chemin | `noether_gui`, `noether_tpp` |
| recaler la piece reelle, planifier en articulaire | hors de noether |

`noether_gui` appelle directement cette bibliotheque quand on lui ouvre un STEP. La
ligne de commande `noether_cad` fait la meme chose sans interface.

---

## 2. Entree

Un fichier STEP (`.step`, `.stp`). Un fichier que OpenCASCADE ne lit pas comme un STEP est
refuse a la lecture.

Ne pas accepter de PLY en entree est un choix. Un PLY est un ensemble de triangles : il
n'a ni face nommee, ni topologie, ni unite. Sans B-Rep :

- la normale disponible est celle d'un triangle, pas celle de la surface
- la table des faces n'existe pas, donc rien a selectionner
- l'unite devient une devinette

**Un fichier a plusieurs solides est refuse tant qu'on n'a pas choisi**, et le refus
liste les solides. Le paquet ne choisit pas a la place de l'operateur. Trois choix
possibles :

- tous les solides (`--solids all`) : la geometrie de collision est complete, c'est le
  choix normal pour une piece traitee en place
- une liste (`--solids <i,j,...>`), avec les indices que `info` affiche
- une exclusion (`--excluded-solids <i,j,...>`), avec `all` ou avec une liste : ce qui
  est ecarte n'existe plus dans le maillage, ni a l'ecran ni pour la collision

---

## 3. Traitement

```
  1. lecture OCCT        unite declaree lue et appliquee  ->  metres
  2. tessellation        deflexion lineaire + angulaire, imposees par l'appelant
  3. table des faces     chaque triangle porte l'id de sa face STEP
  4. fusion              sommets confondus entre faces voisines fusionnes a 0,1 micron
  5. recentrage          translation vers le centre de la boite englobante
  6. ecriture            <prefixe>.ply  +  <prefixe>.yaml
```

### L'unite est lue, jamais devinee

L'unite est portee par une entite `LENGTH_UNIT` de la section DATA, pas par le HEADER :

```
#15538 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
```

Elle est lue dans le texte du fichier plutot que par OpenCASCADE, dont l'API change
entre versions alors que le format de l'entite est fige par la norme. Un fichier sans
unite, ou avec plusieurs unites differentes, est refuse. L'appelant peut imposer
l'unite (`--unit mm|cm|m|in`) pour reinterpreter les nombres d'un fichier dont l'unite
declaree est fausse ; le fichier doit tout de meme en declarer une, lisible. La fiche
note alors `unit_forced: true`. `noether_gui` n'impose jamais l'unite.

### La conversion en metres precede la tessellation

Les deflexions s'expriment dans l'unite du modele. Mailler une forme restee en
millimetres avec une deflexion donnee en metres la rend mille fois trop fine, sans
aucune erreur. La mise a l'echelle est donc faite avant de mailler.

### La verification vient apres la tessellation

Sur une piece riche en NURBS, la boite englobante calculee sur les points de controle
n'a pas de rapport avec l'encombrement reel. Seuls les triangles donnent la vraie
etendue. Elle est affichee a chaque conversion : c'est le seul controle qui detecte une
unite fausse, et il demande un oeil humain.

### Aucune valeur par defaut pour la tessellation

Deflexion lineaire et deflexion angulaire sont **obligatoires** et strictement
positives. Une finesse de maillage est un choix d'ingenierie. Trop grossier, la corde
des triangles passe sous la surface et l'etendue retrecit. Trop fin, le maillage
alourdit tout ce qui suit.

### Le recentrage est une translation pure

Vers le centre de la boite englobante des triangles. Pas de rotation ni d'alignement
sur des axes d'inertie : ce serait imprevisible et impossible a refaire a la main.

**La transformation appliquee est ecrite dans la fiche.** Une piece est souvent exportee
a sa place dans l'assemblage du produit, donc loin de l'origine. Sans cette
transformation, ce qui a ete palpe sur la piece reelle ne peut plus etre rattache au
maillage recentre, et rien ne le signale.

---

## 4. Sorties

Un `.ply` ou un `.yaml` deja present sous le meme prefixe est remplace sans
confirmation.

### `<prefixe>.ply`

PLY binaire little-endian, sommets en **metres** sur 32 bits, triangles orientes vers
l'exterieur de la matiere. Le maillage est **etanche** : les sommets des faces voisines
sont fusionnes, chaque arete partagee appartient a deux triangles.

### `<prefixe>.yaml`

```yaml
cad:
  source: <chemin du STEP>
  declared_unit: mm
  unit_forced: false
  solid_count: 1
  solid_total: 1
  retained_solids: [0]
  excluded_solids: []

  # output_frame = T_source_output * source_frame_in_meters
  recentering: bounding_box        # ou none
  T_source_output:
    translation: [<tx>, <ty>, <tz>]
    rotation_quat: [0.0, 0.0, 0.0, 1.0]

  tessellation:
    linear_deflection_m: <valeur>
    angular_deflection_rad: <valeur>
    vertices: <n>
    triangles: <n>

  bounding_box_m:                  # mesuree sur les triangles, apres recentrage
    min: [<x>, <y>, <z>]
    max: [<x>, <y>, <z>]
    extent: [<dx>, <dy>, <dz>]

  # triangles : l'intervalle [debut, fin[ dans l'ordre du maillage
  faces:
    - id: 0
      type: plane
      triangles: [0, <n>]
      area_m2: <a>
      centroid: [<x>, <y>, <z>]
      normal: [<nx>, <ny>, <nz>]
    - id: 1
      type: cylinder
      triangles: [<n>, <m>]
      area_m2: <a>
      centroid: [<x>, <y>, <z>]
      axis_point: [<x>, <y>, <z>]
      axis_direction: [<x>, <y>, <z>]
      radius_m: <r>
```

Les parametres analytiques ne sont ecrits que pour les types qui en portent :

| type | parametres |
|---|---|
| `plane` | `normal`, sortante |
| `cylinder` | `axis_point`, `axis_direction`, `radius_m` |
| `cone` | `axis_point`, `axis_direction` |
| `sphere` | `center`, `radius_m` |
| `torus` | `axis_point`, `axis_direction`, `radius_m` (grand rayon) |
| `bspline`, `other` | aucun |

Le champ `triangles` permet de selectionner une face sans recherche : il suffit
d'extraire un intervalle.

---

## 5. Utilisation

Diagnostic, n'ecrit rien :

```bash
ros2 run noether_cad noether_cad info <piece.step>
```

Rapporte l'unite declaree, le nombre de solides et la repartition des faces par type.

Production du maillage :

```bash
ros2 run noether_cad noether_cad mesh <piece.step> \
  --linear-deflection <metres> \
  --angular-deflection <radians>
```

| option | defaut | role |
|---|---|---|
| `--linear-deflection` | **aucun** | ecart maximal surface / triangles, en metres |
| `--angular-deflection` | **aucun** | ecart angulaire maximal entre triangles voisins, en radians |
| `--output <prefixe>` | le STEP sans son extension | prefixe du `.ply` et du `.yaml` |
| `--unit mm\|cm\|m\|in` | lue dans le fichier | impose l'unite |
| `--recentering bounding_box\|none` | `bounding_box` | mode de recentrage |
| `--solids all\|one\|<i,j,...>` | `one` | solides a mailler ; `one` refuse un fichier a plusieurs solides |
| `--excluded-solids <i,j,...>` | aucun | solides ecartes, avec `all` ou une liste |

Codes de retour : 0 succes, 2 ligne de commande incomplete, 3 STEP inexploitable,
4 autre erreur.

**Toujours relire l'etendue affichee.**

---

## 6. Decoupage logiciel

```
  libnoether_cad      C++ sans ROS. Lit, maille, tabule, recentre, ecrit.
        |
        +-- noether_cad       la ligne de commande, sous-commandes info et mesh
        +-- noether_gui       ouvre un STEP en appelant la bibliotheque
```

L'API publique (`cad_model.h`, `step_reader.h`, `mesh_writer.h`) **n'expose aucun type
OpenCASCADE** : le B-Rep ne sort pas de la bibliotheque, OCCT n'est donc pas une
dependance de ceux qui l'utilisent.

---

## 7. Dependances

OpenCASCADE (`libocct-foundation-dev`, `libocct-modeling-data-dev`,
`libocct-modeling-algorithms-dev`, `libocct-data-exchange-dev`), sans cle rosdep : il
est trouve par son `OpenCASCADEConfig.cmake`. Eigen. yaml-cpp pour les tests seulement :
la fiche est ecrite a la main, avec ses commentaires.

---

## 8. Tests

`-DNOETHER_ENABLE_TESTING=ON`. Les tests fabriquent leurs STEP avec OpenCASCADE, sans
fichier de piece :

- unite lue (mm, m), unite imposee, entite `LENGTH_UNIT` absente, multiple ou en pouces
- recentrage par translation pure, ecrit dans `T_source_output`
- table des faces contigue, aires exactes, normales et triangles sortants
- fusion des sommets
- faces courbes : axe et rayon d'un cylindre, centre et rayon d'une sphere, recentres
- selection de solides : refus sans choix, `all`, liste, doublons, exclusion, indice
  inexistant, selection vide
- deflexions nulles, negatives, infinies ou NaN refusees
- unite imposee sur un fichier sans unite refusee
- taille du PLY, fiche YAML relue

---

## 9. Hors perimetre

- reparation de geometrie
- decimation : on regle la deflexion au lieu de creer puis detruire des triangles.
  Decimer detruirait la correspondance triangle vers face.
- aretes et sommets CAO : la table ne contient que des faces
- selection de features sur le B-Rep : elle demanderait de garder la forme chargee en
  memoire, ce que `buildModel` ne fait pas
