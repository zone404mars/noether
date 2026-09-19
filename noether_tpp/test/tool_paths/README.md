# Chemins outil de reference

Paire reelle, prise sur la piece 5803024014 du registre z404, programme
`9521e303-0eee-4c86-bd22-2ebbf2d77695`.

| Fichier | Ce que c'est |
|---|---|
| `generated_tool_path.yaml` | ce que la generation Noether a produit, 7 passes |
| `hand_edited_tool_path.yaml` | ce que le mainteneur en a fait a la main, 4 passes |

La retouche manuelle a reordonne, inverse et soude des passes. Aucune pose n'y a ete
creee ni deplacee : chaque pose du fichier edite existe telle quelle dans le fichier
genere. C'est ce qui en fait un oracle utilisable pour `applyRecipe`, et c'est la
raison pour laquelle cette paire est versionnee ici plutot que regeneree.
