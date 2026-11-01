#!/bin/bash
# Captures d'écran du guide : joue le scénario de la grande girouette dans l'image Noether, sur un
# écran virtuel de 1600 x 1000, et écrit les captures brutes dans shots/. annotate.py les recadre
# et les annote ensuite dans img/.
#
# Usage : capture.sh <image noether> <step de la pièce> <dossier de config>
#   <dossier de config> contient step_tessellation.yaml et noether_gui_param.yaml
#   (ceux de z404-docker-images, x86/noether/desktop/config).
# Les coordonnées de clic valent pour cette fenêtre et cette pièce : un changement de disposition de
# l'IHM demande de les reprendre.
set -euo pipefail

IMAGE=$1
STEP=$2
CONFIG=$3
GUIDE=$(cd "$(dirname "$0")" && pwd)
# Dossier de travail : la pièce, sa conversion et le chemin exporté
WORK=$(mktemp -d)
mkdir -p "$GUIDE/shots"
cp "$STEP" "$WORK/grande_girouette.step"
cp "$CONFIG"/step_tessellation.yaml "$CONFIG"/noether_gui_param.yaml "$WORK/"

# Image de capture : l'image Noether, plus un serveur X virtuel et de quoi le piloter
docker build -q -t z404-noether:guide - > /dev/null <<EOF
FROM $IMAGE
USER root
RUN apt-get update && apt-get install -y --no-install-recommends xvfb xdotool imagemagick x11-utils openbox \
    && rm -rf /var/lib/apt/lists/*
EOF

docker rm -f noether_guide > /dev/null 2>&1 || true
docker run -d --name noether_guide --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY=:99 \
    -e LIBGL_ALWAYS_SOFTWARE=1 -v "$WORK":/work -v "$GUIDE/shots":/shots -w /work z404-noether:guide \
    sh -c "Xvfb :99 -screen 0 1600x1000x24 & sleep 1; openbox & sleep 1; \
           noether_gui_app -s step_tessellation.yaml -c noether_gui_param.yaml > /work/app.log 2>&1; sleep infinity" \
    > /dev/null

# x : commande xdotool dans le conteneur ; shot : capture de tout l'écran
x() { docker exec noether_guide xdotool "$@"; }
shot() { docker exec noether_guide import -window root "/shots/$1.png"; }
# Un sommet du tracé : appui, petit mouvement, relâchement au même endroit
vertex() { x mousemove "$1" "$2" mousedown 1 mousemove $(($1 + 1)) "$2" mousemove "$1" "$2" mouseup 1; sleep 1.5; }

sleep 6
WINDOW=$(x search --name "Tool Path Planner" | head -1)
x windowsize "$WINDOW" 1600 1000 windowmove "$WINDOW" 0 0
sleep 2
shot 00_start

# Étape 1 : ouvrir la pièce, attendre la conversion du STEP
x mousemove 76 110 click 1; sleep 2; shot 01_dialog
x mousemove 683 335 click --repeat 2 1
until ls "$WORK"/*.ply > /dev/null 2>&1; do sleep 5; done
sleep 8; shot 02_loaded

# Vues standard : Côté, Isométrique, Face
x mousemove 462 64 click 1; sleep 1; shot 03_affichage
x mousemove 156 110 click 1; sleep 1; shot 04_vues
for view in "1377 263 cote" "1521 263 iso" "1377 232 face"; do
    set -- $view
    x mousemove "$1" "$2" click 1; sleep 3; shot "05_$3"
done
# Bandeau des trois vues, assemblé par ImageMagick dans le conteneur
docker exec noether_guide convert /shots/05_cote.png /shots/05_iso.png /shots/05_face.png \
    -crop 1010x830+282+146 +repage +append -resize 1600x /shots/views_strip.png

# Étape 2 : vue Côté, zoom, Région sur la zone centrale, Valider la surface
x mousemove 1377 263 click 1; sleep 2; x mousemove 787 560 click 4 click 4 click 4; sleep 2
x mousemove 154 64 click 1; sleep 2; shot 06_selection
x mousemove 787 560 click 5 click 5; sleep 2
x mousemove 650 500 click 1; sleep 15; shot 07_region
x mousemove 437 110 click 1; sleep 4; shot 08_validated

# Étape 3 : tracé en serpentin de 8 sommets, Générer
x mousemove 260 64 click 1; sleep 2; shot 09_traitement
vertex 410 475; vertex 1165 475; vertex 1165 520; vertex 410 520
vertex 440 565; vertex 1135 565; vertex 1100 598; vertex 490 598
sleep 2; shot 10_trace
x mousemove 164 110 click 1; sleep 15; shot 11_generated

# Éditer les points : A au haut de la descente en biais, Ctrl + clic sur B en bas, Densifier au pas de 10 mm
x mousemove 277 110 click 1; sleep 2; shot 12_edit
x mousemove 412 523 click 1; sleep 2
x keydown ctrl; x mousemove 443 565 click 1; x keyup ctrl; sleep 3
x mousemove 700 760; sleep 2; shot 15_densify_pts
x mousemove 1449 394 click 1; sleep 4; shot 16_densified

# Étape 4 : ordre des passes, export
x mousemove 364 64 click 1; sleep 1; x mousemove 78 110 click 1; sleep 2; shot 14_trajectoire
x mousemove 189 110 click 1; sleep 2; shot 17_export
x type --delay 30 "chemin_grande_girouette.yaml"; x key Return; sleep 4; shot 18_exported

# Couverture : vue isométrique, zoom
x mousemove 462 64 click 1; sleep 1; x mousemove 156 110 click 1; sleep 1
x mousemove 1521 263 click 1; sleep 3; x mousemove 787 560 click 4; sleep 3; shot 19_iso

cp "$WORK/chemin_grande_girouette.yaml" "$GUIDE/shots/" 2> /dev/null || true
docker rm -f noether_guide > /dev/null
echo "captures dans $GUIDE/shots, travail dans $WORK"
