"""Annote les captures du guide Noether : cadres de zones et pastilles numérotées."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

# dossier des captures brutes et dossier des images du guide
SHOTS = Path(__file__).resolve().parent / "shots"
IMAGES = Path(__file__).resolve().parent / "img"
# couleur des annotations
RED = (220, 30, 60)
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)


def badge(draw, x, y, label):
    """Pastille rouge numérotée centrée sur (x, y)."""
    draw.ellipse((x - 17, y - 17, x + 17, y + 17), fill=RED, outline="white", width=3)
    draw.text((x, y), label, fill="white", font=FONT, anchor="mm")


def box(draw, rect, label=None, width=4):
    """Cadre rouge, avec une pastille à son coin haut gauche si un libellé est donné."""
    draw.rectangle(rect, outline=RED, width=width)
    if label:
        badge(draw, rect[0] + 18, rect[1] + 18, label)


def annotate(source, target, boxes=(), badges=(), crop=None):
    """Écrit target depuis source avec les cadres et pastilles, puis le recadrage éventuel."""
    image = Image.open(SHOTS / f"{source}.png").convert("RGB")
    draw = ImageDraw.Draw(image)
    for rect, label in boxes:
        box(draw, rect, label)
    for x, y, label in badges:
        badge(draw, x, y, label)
    if crop:
        image = image.crop(crop)
    image.save(IMAGES / f"{target}.png")


def main():
    """Produit toutes les images du guide, recadrées sur la zone utile de chaque étape."""
    IMAGES.mkdir(exist_ok=True)
    # zones de la fenêtre 1600 x 1000 : ruban, vue 3D, panneau Options, barre d'état
    ribbon = (0, 40, 620, 145)
    annotate("00_start", "window", boxes=[((4, 44, 1596, 142), "A"), ((4, 148, 272, 976), "B"),
                                          ((282, 148, 1292, 976), "C"), ((1298, 148, 1596, 976), "D"),
                                          ((4, 979, 1596, 999), None)],
             badges=[(800, 960, "E")])
    annotate("01_dialog", "open_dialog", badges=[(30, 95, "1"), (598, 335, "2")], crop=(0, 40, 1110, 690))
    annotate("02_loaded", "loaded_status", crop=(740, 978, 1600, 1000))
    annotate("04_vues", "views_ribbon", badges=[(108, 95, "1")], crop=ribbon)
    annotate("04_vues", "views_options", badges=[(1318, 263, "2")], crop=(1296, 146, 1600, 290))
    annotate("07_region", "region", badges=[(18, 95, "1"), (650, 500, "3"), (380, 95, "4")],
             crop=(0, 40, 1294, 760))
    annotate("07_region", "region_options", badges=[(1440, 263, "2")], crop=(1296, 146, 1600, 290))
    annotate("08_validated", "validated", boxes=[((10, 192, 264, 226), None)], badges=[(250, 245, "1")],
             crop=(0, 146, 280, 290))
    annotate("09_traitement", "trace_ribbon", badges=[(20, 95, "1")], crop=ribbon)
    annotate("09_traitement", "trace_tree", badges=[(250, 217, "2")], crop=(0, 146, 280, 290))
    annotate("09_traitement", "trace_options", badges=[(1565, 293, "3"), (1565, 325, "4"), (1565, 357, "5")],
             crop=(1296, 146, 1600, 410))
    annotate("10_trace", "trace", badges=[(410, 452, "1"), (487, 625, "2"), (392, 545, "3")],
             crop=(282, 380, 1294, 660))
    annotate("10_trace", "trace_status", crop=(1210, 978, 1600, 1000))
    annotate("11_generated", "generated", badges=[(128, 95, "1"), (225, 268, "2")], crop=(0, 40, 1294, 700))
    annotate("12_edit", "edit_ribbon", badges=[(277, 95, "1")], crop=ribbon)
    # les lettres A et B sont dessinées par l'IHM elle-même
    annotate("15_densify_pts", "densify_points", crop=(282, 380, 1294, 660))
    annotate("15_densify_pts", "densify_options", badges=[(1440, 331, "1"), (1440, 363, "2"), (1330, 394, "3")],
             crop=(1296, 146, 1600, 410))
    annotate("02_loaded", "cube", badges=[(1196, 196, "1"), (1276, 252, "2"), (1274, 187, "3"), (1144, 273, "4")],
             crop=(1110, 146, 1296, 320))
    annotate("16_densified", "densified", crop=(282, 420, 800, 640))
    annotate("14_trajectoire", "order_ribbon", badges=[(24, 95, "1")], crop=ribbon)
    annotate("14_trajectoire", "order_options", badges=[(1562, 276, "2"), (1322, 477, "3"), (1458, 508, "4"),
                                                        (1322, 539, "5"), (1458, 539, "6")],
             crop=(1296, 146, 1600, 560))
    annotate("17_export", "export", badges=[(150, 95, "1"), (790, 622, "2")], crop=(0, 40, 1110, 690))
    annotate("19_iso", "result_iso", crop=(282, 230, 1292, 900))


if __name__ == "__main__":
    main()
