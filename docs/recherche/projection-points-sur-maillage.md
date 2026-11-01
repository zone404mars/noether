# Poser des points sur un maillage entre deux points sans changer de nappe

Recherche bibliographique du 31/10/2026 pour l'outil **Densifier** du fork Noether.

Code concerné :
`noether_gui/src/widgets/tpp_widget_densify.cpp`, fonctions `gapPoses` (l. 239) et
`projectOnMesh` (l. 268). Le localisateur est construit sur `selection_poly_`, c'est-à-dire
la pièce entière (`tpp_widget_densify.cpp:228`, `tpp_widget.cpp:688-689`). La
spécification décrit ce comportement tel quel (`SPECIFICATIONS.md` §9.2 et §21) : pas
d'écart entre spec et code.

Conventions de ce document :
- chaque affirmation porte sa source (URL, DOI, ou `fichier:ligne`) ;
- **Déduction** marque un raisonnement géométrique fait ici, sans source externe ;
- **Estimation** marque un coût non mesuré ;
- **Incertain** marque ce que la recherche n'a pas pu vérifier sur une source primaire.

Vocabulaire :
- **couple A, B** : deux poses voisines d'une passe, sur la surface ; Densifier ajoute des
  points entre elles ;
- **corde** : segment droit A-B ; **L** sa longueur ; **écart** : l'intervalle entre A et B ;
- **peau** : une des deux grandes faces d'une tôle ; **nappe** : morceau de surface
  connexe que l'on veut garder (ici la peau de A et B) ;
- **chant** : la paroi étroite d'épaisseur t au bord libre d'une tôle ;
- **congé** : raccord arrondi entre deux faces ; **pli** : tôle courbée, de rayon
  extérieur **R** ;
- **ourlet** : bord de tôle replié à 180° sur lui-même ; **épingle** : pli à 180° serré ;
- **flèche s** : distance maximale entre la corde et l'arc qu'elle sous-tend ;
- **slerp** : interpolation sphérique entre deux vecteurs unitaires ;
- **variété** (manifold) : maillage où chaque arête touche au plus deux triangles ;
- **B-rep** : représentation CAO exacte par faces et arêtes (le STEP) ;
- **ICP** : recalage itératif par appariement de points ; **OLP** : programmation hors
  ligne de robots ; **capot clim** : pièce réelle de test du projet.

---

## 1. Résumé

1. Le point le plus proche est garanti unique et sur la bonne surface tant que le point projeté est à moins de la portée (reach) de Federer ([DOI 10.1090/S0002-9947-1959-0110078-1](https://doi.org/10.1090/S0002-9947-1959-0110078-1)) ; pour une tôle d'épaisseur t, cette portée vaut au plus t/2 (déduction).
2. La corde A-B (longueur L) s'enfonce sous la peau convexe d'un pli de rayon extérieur R de sa flèche s = R - sqrt(R² - L²/4) ; dès que s > t/2, le plus proche est sur l'autre peau. Pour L = 20 mm et t = 3 mm : R < 34 mm (déduction, §3.1).
3. Une correction documentée dans plusieurs domaines est le « point le plus proche compatible » : on écarte les faces dont la normale s'éloigne trop d'une normale de référence (ICP de Pulli, seuil 45°, [DOI 10.1109/IM.2001.924423](https://doi.org/10.1109/IM.2001.924423) ; option Cull Faces de Blender).
4. La projection selon une direction (CATIA, OpenCASCADE, RoboDK, libigl ; NX et SolidWorks incertains) peut rendre plusieurs solutions ; les logiciels gardent la plus proche ou tirent dans les deux sens. Elle échoue près des bords quand le rayon manque la pièce.
5. La coupe par plan (Noether, `vtkCutter`) donne exactement la courbe d'une passe raster, mais il faut choisir la composante connexe et elle s'interrompt sur un trou.
6. Les géodésiques (MMP, chaleur, FlipOut) et la marche face par face restent sur la nappe connexe par construction ; elles demandent un maillage soudé, souvent variété, et contournent un trou au lieu de le survoler.
7. VTK 9.1 fournit `vtkDijkstraGraphGeodesicPath` (chemin par arêtes), que VTK utilise lui-même pour tracer un contour entre deux points d'une surface ; geometry-central (MIT) fournit FlipOut et `traceGeodesic`.
8. En FAO (PowerMill, Robotmaster), le nombre de points est piloté par une tolérance de corde ou un écart maximal ; la documentation publique ne dit pas où les points ajoutés sont posés (incertain).
9. Recommandation principale : plus proche compatible, normale de référence interpolée de A à B, recherche bornée autour du point, contrôles de continuité et refus explicite en cas d'échec.
10. Alternative si la courbe voulue est la section plane de la passe : coupe locale par le plan (A, B, n) et marche connexe de A à B ; les cas de test communs sont en §6.

---

## 2. Tableau comparatif

Coûts donnés pour quelques centaines de points sur 0,1 à 1 M triangles. Sauf mention
contraire, ce sont des **estimations** non mesurées.

| Approche | Nappe garantie | Modes d'échec | Coût | Complexité avec VTK 9.1 / Eigen | Dépendances |
| --- | --- | --- | --- | --- | --- |
| 3.1 Plus proche, pièce entière (actuel) | Non. Seulement si flèche < t/2 | Corde dans la matière (congé serré), angle vif, paroi voisine | Requêtes locales, < 1 ms au total (estimation) ; localisateur O(n log n) déjà construit | Existant | VTK |
| 3.1 Plus proche compatible (filtre de normale) | Oui contre la peau opposée d'une tôle seule. Non contre une paroi parallèle de même orientation | Normale de référence indéfinie (épingle 180°), surface qui tourne plus que le seuil entre A et B, pas de face compatible proche | Comme ci-dessus si la recherche est bornée | Faible : `FindCellsWithinBounds` + boucle sur les cellules | VTK |
| 3.2 Rayon selon la normale ou une direction | Non. Dépend du sens du tir et du choix parmi les impacts | Rayon qui manque la pièce près d'un bord (76/3000 au prototype), impact sur une paroi en surplomb | Très faible (estimation) | Faible : `IntersectWithLine` ; impacts multiples seulement dans `vtkOBBTree` | VTK |
| 3.3 Coupe par plan (A, B, n) + marche connexe | Oui entre A et B tant qu'aucune paroi ne coupe le chemin | Trou entre A et B, A-B parallèle à n, plusieurs composantes dans la zone | Faible si la coupe est locale ; O(n) par coupe sur la pièce entière | Moyenne : `vtkCutter`, `vtkStripper`, `vtkPolyDataConnectivityFilter` | VTK |
| 3.4 Géodésique par arêtes (Dijkstra) | Oui sur un maillage connexe soudé (le passage à l'autre peau est plus long) | Contourne les trous, chemin en zigzag sur les arêtes, A et B à ramener sur des sommets | Faible sur un sous-maillage local avec arrêt à la cible | Faible : `vtkDijkstraGraphGeodesicPath` | VTK |
| 3.4 Géodésique exacte ou redressée (MMP, FlipOut, chaleur) | Comme ci-dessus | Maillage non variété refusé (FlipOut), mémoire (MMP), trou contourné | MMP point à point : quelques s sur 1 M triangles (mesuré par les auteurs) ; FlipOut : ms (annoncé) | Moyenne à forte : conversion de maillage | geometry-central (MIT), libigl (MPL 2.0), CGAL (GPL) |
| 3.5 Marche face par face (dans le plan ou géodésique la plus droite) | Oui jusqu'au premier bord ou à la première paroi | Bord ou trou (arrêt), arête non variété, passage par un sommet | Proportionnel au nombre de faces traversées, négligeable | Moyenne : `GetCellEdgeNeighbors` et intersection plan-triangle à écrire | VTK, ou geometry-central |
| 3.7 Plus proche restreint à la région | Oui si la région ne contient qu'une peau | Point projeté sur le bord de la région sans erreur signalée, région qui contient les deux peaux | Faible ; localisateur par région | Existant (`projectOnSurface`) | VTK |

---

## 3. Approches

### 3.1 Projection au point le plus proche

**Principe.** Chaque point de la corde est remplacé par le point de la surface le plus
proche. `vtkCellLocator::FindClosestPoint` explore les cases autour du point par anneaux
successifs, puis vérifie les cases voisines ; il classe les candidats par distance seule,
sans aucun filtre d'orientation ([vtkCellLocator.cxx, VTK 9.1.0](https://github.com/Kitware/VTK/blob/v9.1.0/Common/DataModel/vtkCellLocator.cxx)).
Le même schéma se retrouve en remaillage : `pmp-library` crée le milieu d'une arête trop
longue puis le reprojette sur la face la plus proche du maillage de référence, trouvée par
un kd-tree, sans filtre ([remeshing.cpp, `split_long_edges` et `project_to_reference`](https://github.com/pmp-library/pmp-library/blob/main/src/pmp/algorithms/remeshing.cpp)).

**Pourquoi il change de peau.**
- La projection au plus proche est garantie unique pour tout point à une distance de la
  surface inférieure à sa portée (reach), notion introduite par Federer ; c'est une
  condition suffisante : au-delà, le plus proche peut rester unique mais se trouver sur
  une autre partie de la surface
  ([DOI 10.1090/S0002-9947-1959-0110078-1](https://doi.org/10.1090/S0002-9947-1959-0110078-1)) ;
  sa version locale est la « local feature size », distance à l'axe médian
  ([Amenta et Bern 1999, DOI 10.1007/PL00009475](https://doi.org/10.1007/PL00009475)).
  La méthode « closest point » des EDP sur surfaces repose sur la même condition
  ([Ruuth et Merriman 2008, DOI 10.1016/j.jcp.2007.10.009](https://doi.org/10.1016/j.jcp.2007.10.009), contenu non relu ici).
- **Déduction.** Dans une tôle d'épaisseur t, l'axe médian est la surface à mi-épaisseur :
  la portée vaut au plus t/2. Un point de la corde plus enfoncé que t/2 a pour plus proche
  la peau opposée, dont la normale est inversée.
- **Déduction.** Sur la face convexe d'un pli de rayon extérieur R, le milieu d'une corde
  de longueur L est enfoncé de la flèche s = R - sqrt(R² - L²/4) ≈ L²/(8R). La mauvaise
  peau apparaît quand s > t/2. Ce calcul suppose que l'arc A-B reste dans le pli
  (R >= L/2) et que R >= t (rayon intérieur positif). Sur la face concave, la corde
  passe dans l'air : le risque y vient d'une autre paroi proche, pas de la flèche.

```text
coupe d'un pli, face convexe en haut (épaisseur t)

               _ - - - _                 peau extérieure, rayon R
          A  /           \  B
            o------M------o              corde A-B, longueur L ; M est à s sous la peau extérieure
           /    _ - - _    \
          /   /         \   \            peau intérieure, rayon R - t, à t sous la peau extérieure

si s > t/2, M est plus près de la peau intérieure : le plus proche y tombe, normale inversée
```

| t | L | R critique (mauvaise peau en dessous) |
| --- | --- | --- |
| 3 mm | 20 mm | 34 mm |
| 3 mm | 10 mm | 9 mm |
| 1 mm | 20 mm | 100 mm |

| t | R | L maximal sans risque |
| --- | --- | --- |
| 3 mm | 5 mm | 7 mm |
| 3 mm | 10 mm | 10,5 mm |
| 1 mm | 5 mm | 4,4 mm |

Ces valeurs sont cohérentes avec les 3 % de couples touchés dans les congés du capot
clim, à L ≈ 20 mm. Elles montrent aussi qu'aucun pas fixe ne suffit pour toute pièce :
plus la tôle est fine et le congé serré, plus le pas sûr est court.

**Comment la littérature traite le cas biface.**

1. *Filtre d'orientation.* L'ICP de référence de Rusinkiewicz et Levoy reprend celui de
   Pulli : chaque point est apparié « au plus proche échantillon de l'autre maillage dont
   la normale est à moins de 45° de la normale source », et les paires qui touchent un
   sommet de bord sont rejetées
   ([Rusinkiewicz et Levoy 2001, §2, DOI 10.1109/IM.2001.924423](https://doi.org/10.1109/IM.2001.924423),
   [PDF auteur](https://www.cs.princeton.edu/~smr/papers/fasticp/fasticp_paper.pdf) ;
   [Pulli 1999, DOI 10.1109/IM.1999.805346](https://doi.org/10.1109/IM.1999.805346)).
   Le même article nomme cette variante « closest compatible point ». Le modificateur
   Shrinkwrap de Blender propose « Cull Faces » (front ou back) pour interdire la
   projection sur l'un des côtés des faces, selon leur normale
   ([manuel Blender, Shrinkwrap](https://docs.blender.org/manual/en/latest/modeling/modifiers/deform/shrinkwrap.html)).
   Le prototype « plus proche parmi les faces orientées comme la normale moyenne » est
   cette méthode : 0 mauvaise peau, 0 échec sur 3000 couples.
2. *Continuité et historique.* En simulation de tissu, Bridson et al. notent que
   « le tissu étant très fin, même de petites interpénétrations le font ressortir du
   mauvais côté », et que des forces de répulsion seules ne suffisent pas : il faut
   considérer le trajet entre deux pas de temps
   ([Bridson, Fedkiw, Anderson 2002, DOI 10.1145/566654.566623](https://doi.org/10.1145/566654.566623),
   [PDF auteur](https://www.cs.ubc.ca/~rbridson/docs/cloth2002.pdf), §1 et §7). Le côté se
   décide depuis un état précédent valide, pas depuis la position courante seule.
3. *Recherche locale depuis le point précédent.* Rusinkiewicz et Levoy listent aussi
   « project and walk » : projeter, puis chercher dans le voisinage de l'impact
   ([§3.2 du PDF](https://www.cs.princeton.edu/~smr/papers/fasticp/fasticp_paper.pdf)).
   En remaillage, `pmp-library` ne déplace un sommet que d'un pas de lissage tangentiel
   avant de le reprojeter ([remeshing.cpp, `tangential_smoothing`](https://github.com/pmp-library/pmp-library/blob/main/src/pmp/algorithms/remeshing.cpp)) ;
   **déduction** : c'est la petitesse du déplacement, pas la projection, qui évite le saut.
4. *Rayon limité.* VTK 9.1 offre `FindClosestPointWithinRadius`, qui renvoie 0 si aucune
   cellule n'est dans le rayon (`/usr/include/vtk-9.1/vtkAbstractCellLocator.h:181-217`).
   Le rayon borne la recherche et le coût, mais ne distingue pas les peaux.

**Garantie.**
- Plus proche simple : aucune au-delà de s < t/2.
- Plus proche compatible (**déduction**) : la peau opposée et les parois de bord libre
  (normales à 90°) sont exclues si le seuil est sous 90°. Il reste le risque d'une autre
  surface de même orientation : pour un ourlet ou deux parois parallèles de même
  orientation séparées d'un jeu g, le saut se produit dès que s > (t + g)/2. Le gain est
  donc grand pour une tôle seule, faible pour un ourlet serré.

**Modes d'échec du filtre.**
- Normale de référence indéfinie : la moyenne de n_A et n_B s'annule quand A et B sont de
  part et d'autre d'une épingle à 180°.
- La surface tourne plus que le seuil entre A et B : avec A sur une face et B sur l'autre
  d'un angle vif de 90°, la normale moyenne est à 45° des deux faces : il faut un seuil
supérieur à 45°.
- Aucune face compatible dans le voisinage : sans borne, le plus proche compatible peut
  être loin, sur une autre partie de la pièce.

**Coût.** Estimation : une requête bornée touche quelques dizaines de cellules ; quelques
centaines de points restent sous la milliseconde une fois le localisateur construit.

**Implémentation.** Faible. `FindCellsWithinBounds` (`vtkAbstractCellLocator.h:225`)
donne les cellules d'une boîte autour du point ; une boucle garde celles dont la normale
passe le seuil et calcule le point le plus proche sur chaque triangle
(`vtkCell::EvaluatePosition`). Les normales de face sont déjà dans `face_normals_`.

### 3.2 Projection selon une direction ou selon la normale

**Principe.** On tire un rayon depuis le point de la corde, selon la normale de référence
ou une direction donnée, et on garde un impact.

**Pratique documentée en CAO et en OLP.**
- OpenCASCADE `GeomAPI_ProjectPointOnSurf` calcule **toutes** les projections
  orthogonales d'un point sur une surface (`NbPoints`, `Point(i)`), donne la plus proche
  par `NearestPoint` et `LowerDistance`, cherche par défaut les extrema min et max, et
  accepte un domaine (u, v) restreint dans `Init`
  ([référence OCCT](https://occt3d.com/dev/doc/refman/html/class_geom_a_p_i___project_point_on_surf.html)).
- OpenCASCADE `BRepProj_Projection` projette une arête ou un fil sur une forme, en
  projection cylindrique (une direction) ou conique (un point) ; le résultat est un
  composé de fils parcouru par `More`/`Next`/`Current`, donc potentiellement plusieurs
  ([référence OCCT](https://occt3d.com/dev/doc/refman/html/class_b_rep_proj___projection.html)).
- CATIA V5 Projection : normale ou selon une direction ; type de solution « Nearest
  solution » ou « All solutions », la première ne gardant que la projection la plus
  proche quand plusieurs sont possibles
  ([API V5 via pycatia, `HybridShapeProject`](https://pycatia.readthedocs.io/en/latest/api/pycatia/hybrid_shape_interfaces/hybrid_shape_project.html)).
- SolidWorks Projected Curve : option « Bi-directional » qui projette des deux côtés de
  l'esquisse, et « Reverse projection »
  ([aide SolidWorks 2024](https://help.solidworks.com/2024/English/SolidWorks/Sldworks/r_projected_curves_pm.htm)).
  **Incertain** : page non lisible par l'outil, contenu connu par l'extrait du moteur de
  recherche seulement.
- NX Project Curve : directions « Along Face Normal », « Along Vector », « Towards
  Point », et pour « Along Vector » l'option « Project Both Sides »
  ([NX Open, ProjectCurveBuilder](https://docs.plm.automation.siemens.com/data_services/resources/nx/12/nx_api/custom/en_US/nxopen_python_ref/NXOpen.Features.ProjectCurveBuilder.html)).
  **Incertain** : même réserve, page redirigée vers un portail authentifié.
- RoboDK `ProjectPoints` : `PROJECTION_CLOSEST`, `PROJECTION_ALONG_NORMAL`,
  `PROJECTION_ALONG_NORMAL_RECALC` (défaut, recalcule la normale sur la surface),
  `PROJECTION_CLOSEST_RECALC` ; liste vide si échec
  ([API Python RoboDK](https://robodk.com/doc/en/PythonAPI/robodk.html)).
- libigl `line_mesh_intersection` : un rayon est tiré pour chaque sommet selon N **et son
  opposé** ([en-tête libigl](https://github.com/libigl/libigl/blob/main/include/igl/embree/line_mesh_intersection.h), dépend d'Embree).
- Blender Shrinkwrap, mode Project : sens « Negative » et « Positive » ; si les deux sont
  actifs, l'impact le plus proche gagne ; « Limit » ignore les sommets trop loin de la
  surface ; mode « Target Normal Project » plus lisse mais plus lent
  ([manuel Blender](https://docs.blender.org/manual/en/latest/modeling/modifiers/deform/shrinkwrap.html)).

**Garantie.** Aucune en soi. **Déduction** : un tir dans les deux sens qui garde l'impact
le plus proche a le même seuil t/2 que le plus proche (l'autre peau est à t - s). Une
variante plus sûre part d'un point décalé vers l'extérieur de plus que la flèche
maximale et garde le premier impact sur une face orientée vers le tireur ; ce n'est
documenté dans aucune des sources ci-dessus.

**Modes d'échec.** Rayon qui manque la pièce près d'un bord libre (76 échecs sur 3000 au
prototype) ; impact sur une paroi en surplomb ; normale de référence fausse dans un congé
serré, qui fait dévier le point latéralement.

**Coût.** Très faible (estimation). **Implémentation.** Faible :
`vtkCellLocator::IntersectWithLine` rend le premier impact. La variante qui rend tous les
impacts n'est implémentée que dans `vtkOBBTree` en VTK 9.1
(`/usr/include/vtk-9.1/vtkAbstractCellLocator.h:136-149`).

**Variante B-rep.** Le STEP est disponible : projeter sur la face CAO avec
`GeomAPI_ProjectPointOnSurf` donne toutes les solutions et la normale exacte. **Incertain** :
le lien entre triangle du maillage et face CAO d'origine n'a pas été vérifié dans
`noether_cad` ; A et B sur deux faces CAO posent le choix de la face pour chaque point.

### 3.3 Coupe par plan (plane slicing)

**Principe.** On coupe le maillage par le plan qui contient A, B et la normale de
référence, on garde la polyligne qui passe par A, et on la parcourt jusqu'à B.

**Dans Noether.** `PlaneSlicerRasterPlanner` coupe chaque plan avec `vtkCutter`
(`SetSortByToSortByCell`), assemble avec `vtkStripper` en désactivant
`JoinContiguousSegments` parce qu'il n'ordonne pas les segments, puis réordonne et joint
lui-même (`processPolyLines`) ; les normales viennent des données de points interpolées
par la coupe (`noether_tpp/src/tool_path_planners/raster/plane_slicer_raster_planner.cpp:604-623`,
`:367-379`, `:423`). Le plan y est global : sa normale est
`cut_direction × normale PCA de la pièce` (`:557`), pas un plan par couple A, B.

**FAO.** La méthode iso-planaire (passes = sections planes) est l'une des familles
classiques de l'usinage de formes gauches
([Lasemi, Xue, Gu 2010, revue, DOI 10.1016/j.cad.2010.04.002](https://doi.org/10.1016/j.cad.2010.04.002), contenu non relu ici).

**Garantie (déduction).** La polyligne de coupe suit la connexité du maillage. Depuis A,
elle reste sur la peau de A jusqu'à rencontrer une paroi (bord libre, bord de trou) ; le
passage sur l'autre peau passe forcément par cette paroi, qui se détecte par un saut de
normale. Cela suppose une extraction locale : sur la pièce entière, la coupe d'un solide
fermé est une boucle qui relie les deux peaux par le chant, et c'est le parcours de A
vers B, arrêté à la première paroi, qui garde la bonne peau. B est dans le plan par
construction et sur la surface : si la marche atteint B
sans paroi, tous les points sont sur la bonne nappe.

**Modes d'échec.**
- Trou entre A et B : la polyligne descend dans la paroi du trou. Il faut s'arrêter et
  laisser l'écart sans point, ou reprendre depuis B.
- A-B presque parallèle à la normale de référence : le plan est mal défini.
- Plusieurs composantes dans la zone (autre partie de la pièce, autre peau) : il faut
  garder celle qui contient A, par exemple avec
  `vtkPolyDataConnectivityFilter::SetExtractionModeToClosestPointRegion`
  (`/usr/include/vtk-9.1/vtkPolyDataConnectivityFilter.h:140`).
- Plan qui passe exactement par un sommet ou une arête : segments dégénérés à fusionner.

**Coût.** Estimation : une coupe sur la pièce entière est O(n), soit de l'ordre de
dizaines de millisecondes pour 1 M triangles, à répéter par écart. Restreinte aux
cellules d'une boîte autour de A-B (`FindCellsWithinBounds`), elle ne touche que
quelques centaines de cellules.

**Implémentation.** Moyenne, VTK seul : extraction locale, `vtkCutter`, `vtkStripper`,
choix de la composante, parcours de A vers B, rééchantillonnage par abscisse curviligne.

### 3.4 Géodésiques sur maillage

**Principe.** On calcule le plus court chemin sur la surface de A à B, puis on le
rééchantillonne au pas voulu.

**Méthodes.**
- MMP, exact : [Mitchell, Mount, Papadimitriou 1987, DOI 10.1137/0216045](https://doi.org/10.1137/0216045).
  Surazhsky et al. l'implémentent, rappellent le pire cas O(n² log n), constatent un
  comportement sous-quadratique, calculent un chemin point à point exact sur 1 M
  triangles en « quelques secondes », des chemins entre sommets proches à cadence
  interactive, et identifient la mémoire comme facteur limitant
  ([Surazhsky et al. 2005, DOI 10.1145/1073204.1073228](https://doi.org/10.1145/1073204.1073228),
  [PDF](https://hhoppe.com/geodesics.pdf), §1 et §6).
  Implémentation de Kirsanov reprise par libigl `exact_geodesic` (distances seulement,
  MPL 2.0, [en-tête](https://github.com/libigl/libigl/blob/main/include/igl/exact_geodesic.h))
  et par geometry-central `GeodesicAlgorithmExact`, qui accepte une distance maximale de
  propagation, des points d'arrêt, et rend le chemin par `traceBack`
  ([doc geometry-central](https://github.com/nmwsharp/geometry-central/blob/master/docs/docs/surface/algorithms/geodesic_distance.md)).
  CGAL `Surface_mesh_shortest_path` implémente Xin et Wang
  ([DOI 10.1145/1559755.1559761](https://doi.org/10.1145/1559755.1559761),
  [doc CGAL](https://doc.cgal.org/latest/Surface_mesh_shortest_path/index.html)) ; la
  plupart des paquets CGAL sont sous GPL ([licence CGAL](https://www.cgal.org/license.html)).
- Méthode de la chaleur : deux systèmes linéaires creux, factorisables une fois ; résultat
  = un champ de distance, le chemin demande un tracé supplémentaire le long du gradient
  ([Crane, Weischedel, Wardetzky 2013, DOI 10.1145/2516971.2516977](https://doi.org/10.1145/2516971.2516977),
  [page projet](https://www.cs.cmu.edu/~kmcrane/Projects/HeatMethod/)). libigl
  `heat_geodesics` (MPL 2.0). Appliquée à la génération de trajectoires sur maillages
  scannés de mauvaise qualité par [Zou 2020, arXiv:2004.09136](https://arxiv.org/abs/2004.09136).
  **Estimation** : la factorisation d'un laplacien à 0,5 M sommets coûte plusieurs
  secondes et beaucoup de mémoire, hors de proportion pour un écart de 20 mm.
- FlipOut (edge flips) : part d'un chemin de Dijkstra sur les arêtes et le redresse en
  géodésique localement la plus courte par retournements d'arêtes intrinsèques ;
  « s'exécute en millisecondes » ; exige un `ManifoldSurfaceMesh`
  ([Sharp et Crane 2020, DOI 10.1145/3414685.3417839](https://doi.org/10.1145/3414685.3417839),
  [doc geometry-central](https://github.com/nmwsharp/geometry-central/blob/master/docs/docs/surface/algorithms/flip_geodesics.md)),
  licence MIT ([LICENSE](https://github.com/nmwsharp/geometry-central/blob/master/LICENSE)).
- Géodésique de graphe, VTK 9.1 : `vtkDijkstraGraphGeodesicPath` suit les arêtes, coût
  = longueur euclidienne des arêtes, triangles seulement, option `StopWhenEndReached` ;
  l'adjacence n'est reconstruite que si l'entrée change
  (`/usr/include/vtk-9.1/vtkDijkstraGraphGeodesicPath.h:20-31,76`,
  [source v9.1.0](https://github.com/Kitware/VTK/blob/v9.1.0/Filters/Modeling/vtkDijkstraGraphGeodesicPath.cxx)).
  VTK l'utilise dans `vtkPolygonalSurfaceContourLineInterpolator` pour placer un contour
  entre deux points cliqués sur une surface
  (`/usr/include/vtk-9.1/vtkPolygonalSurfaceContourLineInterpolator.h:20-26`) : c'est la
  réponse de VTK au même besoin d'interface.

**Garantie (déduction).** Le chemin reste sur la surface connexe. Pour A et B sur la même
peau, passer par l'autre peau impose de franchir deux fois une paroi (coût 2t) : ce
chemin est en général plus long. Exception : près d'un bord libre, sur un pli d'angle θ,
la peau intérieure raccourcit le trajet d'environ θ·t ; le passage par l'autre peau
devient possible si θ > 2 rad et si le détour jusqu'au bord est court. Cela suppose un maillage soudé entre
faces CAO, ce que fait `noether_cad` (tolérance de fusion 0,1 µm,
`noether_cad/src/step_reader.cpp:50-56,193`).

**Modes d'échec.**
- Trou entre A et B : la géodésique le contourne. Pour une passe de ponçage, le tracé
  voulu survole le trou ; le contournement change l'intention de la passe.
- La géodésique n'est pas plane : sur une surface en selle, elle s'écarte latéralement
  de la section plane de la passe raster (déduction).
- Dijkstra : extrémités sur des sommets, zigzag le long des arêtes ; l'écart au vrai
  chemin dépend de la taille des triangles (incertain : pas de borne sourcée ici).
- FlipOut et `traceGeodesic` refusent un maillage non variété. **Incertain** : le
  maillage soudé d'OpenCASCADE n'a pas été contrôlé pour les arêtes non variété.

**Coût.** Dijkstra sur un sous-maillage local avec arrêt à la cible : estimation de
l'ordre de la milliseconde. Sur la pièce entière, la construction de l'adjacence est
O(n) une fois.

**Implémentation.** Dijkstra VTK : faible. FlipOut ou MMP : conversion `vtkPolyData` vers
geometry-central ou libigl (Eigen), dépendance supplémentaire.

### 3.5 Marche sur la surface (surface walking, tracing)

**Principe.** On part de la face de A et on avance face par face en traversant les
arêtes partagées, soit dans le plan (A, B, n), soit en ligne « la plus droite ».

**Sources.**
- Polthier et Schmies définissent les géodésiques les plus droites, qui donnent une
  solution unique au problème de la valeur initiale, donc un déplacement unique dans une
  direction tangente donnée, propriété que n'ont pas les géodésiques localement les plus
  courtes ([résumé auteur](https://page.mi.fu-berlin.de/polthier/articles/straightest/straightest_abstract.html),
  [DOI 10.1007/978-3-662-03567-2_11](https://doi.org/10.1007/978-3-662-03567-2_11)).
- geometry-central `traceGeodesic` trace ce chemin depuis un point et un vecteur
  tangent ; il s'arrête sur une arête de bord (`hitBoundary`) ou une arête barrière,
  accepte un nombre maximal de faces, et exige un `ManifoldSurfaceMesh`
  ([doc](https://github.com/nmwsharp/geometry-central/blob/master/docs/docs/surface/algorithms/geodesic_paths.md)).
- Polissage robotisé : Basañez et Rosell construisent un graphe dont les nœuds sont les
  milieux des triangles et dont chaque arc passe par le milieu de l'arête commune ;
  entre deux points cliqués, Dijkstra donne le chemin, puis un lissage déplace chaque
  point le long de son arête jusqu'à convergence
  ([« Robotic polishing systems », IEEE RAM 2005, DOI 10.1109/MRA.2005.1511867](https://doi.org/10.1109/MRA.2005.1511867),
  [PDF auteur](https://iocnet.upc.edu/usuaris/janrosell/PDFs/2005_RAM_Robotic_Polishhing_Systems_BasanezRosell.pdf)).
  C'est une combinaison de 3.4 et 3.5 appliquée exactement à notre besoin.
- Les méthodes de marche servent aussi en CAO pour l'intersection surface/surface
  ([Barnhill et Kersey 1990, DOI 10.1016/0167-8396(90)90035-P](https://doi.org/10.1016/0167-8396(90)90035-P),
  titre vérifié, contenu non relu).

**Garantie (déduction).** Le chemin ne quitte la peau de A qu'en traversant une arête
vers une face de paroi, détectable par le saut de normale, ou en atteignant un bord.

**Modes d'échec.**
- Tir géodésique depuis A vers B : il n'arrive pas exactement sur B dès que la surface
  est courbée ; il faut corriger la direction (problème de tir) ou marcher dans le plan.
- Bord ou trou : arrêt (`hitBoundary`), à traiter comme un écart sans point.
- Passage exactement par un sommet, arête non variété : cas dégénérés à gérer.

**Coût.** Proportionnel au nombre de faces traversées (L / taille d'arête), négligeable.

**Implémentation.** Marche dans le plan avec VTK seul : `vtkPolyData::GetCellEdgeNeighbors`
(`/usr/include/vtk-9.1/vtkPolyData.h:411`) et intersection plan-triangle à écrire,
environ une centaine de lignes (estimation). Marche géodésique : geometry-central.

### 3.6 Pratique en FAO et en programmation hors ligne

- **RoboDK** : projection explicite au choix, plus proche ou selon la normale, avec ou
  sans recalcul de la normale ([API Python](https://robodk.com/doc/en/PythonAPI/robodk.html)).
  La page « Robot Machining » ne décrit pas de rééchantillonnage des courbes importées
  ([doc](https://robodk.com/doc/en/Robot-Machining.html)).
- **PowerMill** : « Redistribute » permet d'insérer des points le long des passes à
  espacement constant, « Point separation distance » limite l'écart entre points
  consécutifs ; la page ne dit pas si les points ajoutés sont posés sur le modèle ou
  interpolés sur la passe
  ([aide PowerMill 2018](https://help.autodesk.com/cloudhelp/2018/ENU/PWRM-ReferenceHelp/files/GUID-0A68D751-66AA-4189-91E5-6739FFFD05CB.htm)).
- **Robotmaster** : selon un article de sa base de connaissances, « Max Deviation »
  (écart maximal entre la surface CAO et le segment droit) et « Max Angle » fixent le
  nombre de points ([KB Robotmaster V7](https://robotmaster.atlassian.net/wiki/x/A4CiU)).
  **Incertain** : page non lisible par l'outil, contenu connu par l'extrait du moteur de
  recherche.
- **Delmia, SprutCAM** : aucune documentation publique trouvée sur l'insertion de points.
  **Incertain.**
- **Noether amont** : `UniformSpacingModifier` rééchantillonne une passe par une spline
  Eigen dans l'espace 3D, sans reprojection sur le maillage
  (`noether_tpp/src/tool_path_modifiers/uniform_spacing_modifier.cpp:18-45`).
- **Littérature robot** : polissage par graphe et Dijkstra sur le maillage (Basañez et
  Rosell 2005, §3.5) ; balayage uniforme sur maillage triangulé pour le polissage par
  jet d'eau abrasif ([Khakpour, Birglen, Tahan 2014, DOI 10.1007/s00170-014-6504-1](https://doi.org/10.1007/s00170-014-6504-1)) ;
  trajectoires robotisées sur surface ([Lyu et al. 2019, DOI 10.1007/s40436-018-00246-x](https://doi.org/10.1007/s40436-018-00246-x)) ;
  peinture guidée par CAO ([Sheng et al. 2000, DOI 10.1109/IROS.2000.895251](https://doi.org/10.1109/IROS.2000.895251)).
  **Incertain** : pour ces trois derniers, seuls titre et revue sont vérifiés, le texte
  n'a pas été lu.

**Constat.** Les logiciels de FAO pilotent la densité par une tolérance de corde et
partent en général de la surface CAO, pas d'un maillage ; aucune des documentations
publiques consultées ne dit comment le logiciel évite l'autre peau d'une tôle.

### 3.7 Projection sur la sous-surface sélectionnée

**Principe.** Le localisateur ne contient que les faces de la région de travail.

**Dans le fork.** `projectOnSurface` le fait déjà pour le tracé manuel : il extrait les
faces de la région, construit un `vtkCellLocator` sur ce sous-maillage et projette au plus
proche (`noether_gui/src/widgets/tpp_widget.cpp:1957-1989`). Les régions grandissent de
face en face tant que l'angle entre faces voisines reste sous le seuil de planéité
(`tpp_widget.cpp:727-731`). En CAO, l'équivalent est le domaine (u, v) restreint de
`GeomAPI_ProjectPointOnSurf::Init` ([OCCT](https://occt3d.com/dev/doc/refman/html/class_geom_a_p_i___project_point_on_surf.html)).

**Garantie (déduction).** Si la région ne contient qu'une peau, l'autre peau est absente
et ne peut pas être choisie. Une paroi de bord libre à 90° arrête la croissance tant que
le seuil de planéité reste sous 90°.

**Modes d'échec.**
- Le vrai point est hors de la région : la projection tombe sur le bord de la région,
  sans erreur.
- La région contient les deux peaux (bord arrondi franchi par la croissance) ou deux
  parois parallèles.
- A et B dans deux régions différentes, ou passe générée hors de toute région conservée.

**Coût.** Faible ; un localisateur par région. **Implémentation.** Existante.

---

## 4. Combinaisons utilisées en pratique

| Domaine | Combinaison | Source |
| --- | --- | --- |
| Recalage ICP | Plus proche + normale compatible à 45° + rejet des paires sur un bord | [Rusinkiewicz et Levoy 2001](https://www.cs.princeton.edu/~smr/papers/fasticp/fasticp_paper.pdf), §2 |
| Recalage ICP | Projection puis recherche locale (« project and walk ») | idem, §3.2 |
| Modélisation (Blender) | Tir selon un axe, sens positif et/ou négatif, faces avant ou arrière exclues, distance limite | [Shrinkwrap](https://docs.blender.org/manual/en/latest/modeling/modifiers/deform/shrinkwrap.html) |
| CAO (CATIA, OCCT) | Toutes les solutions calculées, puis la plus proche gardée, éventuellement dans un domaine restreint | [pycatia](https://pycatia.readthedocs.io/en/latest/api/pycatia/hybrid_shape_interfaces/hybrid_shape_project.html), [OCCT](https://occt3d.com/dev/doc/refman/html/class_geom_a_p_i___project_point_on_surf.html) |
| Remaillage (pmp) | Petit déplacement tangentiel, puis plus proche global ; sommets d'arêtes vives non reprojetés | [remeshing.cpp](https://github.com/pmp-library/pmp-library/blob/main/src/pmp/algorithms/remeshing.cpp) |
| Simulation de tissu | Épaisseur explicite + contrôle du trajet depuis l'état précédent | [Bridson et al. 2002](https://www.cs.ubc.ca/~rbridson/docs/cloth2002.pdf) |
| Polissage robotisé | Chemin de graphe sur le maillage, puis lissage des points sur leurs arêtes | [Basañez et Rosell 2005](https://iocnet.upc.edu/usuaris/janrosell/PDFs/2005_RAM_Robotic_Polishhing_Systems_BasanezRosell.pdf) |
| Interface VTK | Géodésique de graphe entre deux points cliqués | `vtkPolygonalSurfaceContourLineInterpolator.h:20-26` |
| Géodésiques | Dijkstra sur arêtes, puis redressement (FlipOut) | [geometry-central](https://github.com/nmwsharp/geometry-central/blob/master/docs/docs/surface/algorithms/flip_geodesics.md) |

Le motif commun : une **recherche bornée** (petit pas, voisinage, domaine), un **critère
de côté** (normale compatible, face avant, état précédent) et un **rejet explicite** des
cas douteux (bord, pas de solution, limite de distance).

---

## 5. Recommandation

Le choix dépend de la courbe que l'on veut entre A et B :
- « des points sur la peau de A et B, près de la corde » : recommandation 1 ;
- « la section plane de la passe, comme le planificateur raster » : recommandation 2.

### Recommandation 1 : plus proche compatible, local et contrôlé

Évolution de `projectOnMesh`, VTK seul, sans nouvelle structure.

```text
pour chaque point P_k de la corde, k = 1..N, u = k / (N + 1) :
# pas = |P_(k+1) - P_k|, écart régulier entre points de la corde ; Q_0 = A
si angle(n_A, n_B) > angle_max_ab : refuser l'écart              # épingle
pour chaque point P_k de la corde, k = 1..N, u = k / (N + 1) :
    n_ref = slerp(n_A, n_B, u)                  # interpolation sphérique des normales
    cellules = FindCellsWithinBounds(boîte centrée en P_k, demi-côté h)
    candidates = { c : n_face(c) · n_ref >= cos(seuil) }
    Q_k = point le plus proche de P_k parmi candidates
    refuser l'écart si :
        candidates vide
        Q_k sur une arête de bord                                # rejet ICP des bords
        |Q_k - Q_(k-1)| > 2 · pas                               # saut de position
```

- Le prototype décrit dans la demande (filtre sur la normale moyenne) a donné 0 mauvaise
  peau et 0 échec sur 3000 couples du capot clim. Ces chiffres ne sont pas reproduits
  ici et le cas 8 du §6 doit les rejouer.
- Ajouts par rapport au prototype : `slerp` au lieu de la moyenne, voisinage borné,
  contrôle de saut de position, refus signalé à l'opérateur au lieu d'un point faux.
- Aucun contrôle de saut de normale entre points consécutifs : il refuserait un angle vif
  légitime entre A et B. Le côté est déjà tenu par le filtre sur `n_ref`.
- Lien entre réglages (déduction) : sur un angle vif entre A et B, n_ref passe à
  angle(n_A, n_B)/2 de chaque face au milieu ; il faut donc `seuil` > angle_max_ab / 2.
- **Valeurs à valider avant tout code** : `seuil` (Pulli utilise 45°), `h`,
  `angle_max_ab`. Ce sont des propositions, pas des réglages.
- Limite connue (déduction) : ourlet ou parois parallèles de même orientation avec
  flèche > (t + g)/2. Le contrôle de saut de position peut la détecter, sans garantie.

### Recommandation 2 : section plane locale et marche connexe

```text
si angle(n_A, n_B) > angle_max_ab : refuser l'écart              # épingle
plan = plan passant par A et B, contenant slerp(n_A, n_B, 1/2)
cellules = FindCellsWithinBounds(boîte de A-B, gonflée de L/2)  # extraction locale
polylignes = vtkStripper(vtkCutter(cellules, plan))
garder la composante qui contient A
marcher de A vers B ; arrêt si une face coupée a n_face · n_ref(u) < cos(seuil)
    (paroi de bord libre ou de trou), ou si B n'est pas atteint
rééchantillonner au pas par abscisse curviligne ; normale = normale de la face coupée
```

- Même courbe que `PlaneSlicerRasterPlanner` pour une passe raster : cohérence entre
  passe générée et passe densifiée.
- Trou : arrêt explicite, l'écart reste sans point ou est complété depuis B.
- Coût plus élevé à écrire (estimation : 100 à 150 lignes), VTK seul.

Les géodésiques (3.4) ne sont pas recommandées comme solution principale : elles
contournent les trous, ce qui change l'intention de la passe, et les variantes exactes
ajoutent une dépendance. `vtkDijkstraGraphGeodesicPath` reste un bon **oracle de test**
pour vérifier qu'un point densifié est sur la nappe connexe de A.

La spécification (`SPECIFICATIONS.md` §9.2 et §21) devra être amendée avec la règle
retenue : elle décrit aujourd'hui « la face la plus proche du maillage entier ».

---

## 6. Cas de test pour valider sur toute forme

Invariant de succès : pour chaque couple (A, B), chaque point ajouté est sur la même
peau que A et B, ou l'écart est refusé avec un message ; jamais de point sur une autre
nappe. Oracle : surface analytique pour les pièces synthétiques ; nappe connexe de A
par `vtkDijkstraGraphGeodesicPath` pour les pièces réelles.

| # | Cas | Géométrie (STEP synthétique) | Attendu | Mesure |
| --- | --- | --- | --- | --- |
| 1 | Congé serré | Tôle pliée à 90°, t ∈ {1, 3} mm, R extérieur ∈ {t + 1, 5, 10, 34, 50} mm, L ∈ {5, 10, 20, 40} mm ; A, B sur la face convexe, puis sur la face concave avec une paroi voisine | Points sur la bonne peau ; écart au rayon analytique < flèche de maillage | Nombre de points sur la mauvaise peau (n · n_ref < 0, ou distance à la peau attendue supérieure à un seuil à fixer, par exemple t/4) |
| 2 | Tôle mince | Plaque et cylindre, t ∈ {0,5 ; 1 ; 3} mm | Comme 1 | Comme 1 |
| 3 | Trou entre A et B | Perçage Ø {5, 10, 30} mm centré sur A-B | Aucun point dans le trou ni sur sa paroi ; écart refusé ou points seulement sur la peau | Points sur la paroi du trou, refus signalés |
| 4 | Bord libre | A-B parallèle à un bord libre à {0,5 ; 1 ; 3} mm ; A-B qui franchit un bord de bride convexe | Aucun point sur le chant ni sur l'autre peau | Points sur le chant |
| 5 | Parois parallèles proches | Ourlet (même orientation), jeu g ∈ {0 ; 1 ; 3 ; 10} mm ; profil en U (orientations opposées) | Bonne peau, ou refus | Mauvaise peau, refus |
| 6 | A et B sur deux faces CAO | Jonction tangente (congé-plan) ; jonction vive à 90° et 135° | Points sur les deux faces, sans refus, si seuil > angle(n_A, n_B)/2 | Continuité de position, connexité du maillage soudé |
| 7 | Épingle 180° | Pli en U serré, A et B de part et d'autre | Refus explicite (recommandation 1) | Refus signalé |
| 8 | Pièces réelles | Capot clim (3000 couples) et deux pièces d'autres familles | 0 mauvaise peau | Tableau par couple : verdict mauvaise peau, échec, refus |

Le contrôle final énumère tous les couples de tous les cas avec leur verdict.

---

## 7. Reproduction des chiffres de flèche

Les tableaux du §3.1 se recalculent ainsi :

```bash
python3 -c "
import math
for L, t in [(20, 3), (10, 3), (20, 1)]:
    s = t / 2
    print('t', t, 'L', L, 'R critique', round((L * L / 4 + s * s) / (2 * s), 1))
for R, t in [(5, 3), (10, 3), (5, 1)]:
    s = t / 2
    print('t', t, 'R', R, 'L max', round(2 * math.sqrt(2 * R * s - s * s), 1))
"
```

Sortie attendue : R critique 34.1, 9.1, 100.2 ; L max 7.1, 10.5, 4.4.

## 8. Points non vérifiés

- Chiffres du prototype (3 %, 0/3000, 76/3000) : repris de la demande, non reproduits.

- Contenu des pages SolidWorks, NX et Robotmaster : connu par extraits de moteur de
  recherche seulement.
- Méthodes d'insertion de points dans Delmia et SprutCAM : pas de source trouvée.
- Coûts marqués « estimation » : non mesurés sur nos pièces.
- Présence d'arêtes non variété dans le maillage soudé de `noether_cad` : non contrôlée.
- Lien triangle vers face CAO dans `noether_cad` : non vérifié.
- Textes de Lasemi 2010, Ruuth et Merriman 2008, Barnhill et Kersey 1990, Khakpour 2014,
  Lyu 2019, Sheng 2000 : DOI et titres vérifiés, contenu non lu.
