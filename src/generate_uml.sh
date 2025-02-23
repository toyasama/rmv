#!/bin/bash

DIRECTORIES=(
    "rmv_chore/rmv_chore/"
    "rmv_chore/visualization/"
    "library/markers_management/"
    "library/parameters/"
    "library/tf_management/"
    "library/topic_management/"
    "library/utils/"
)

pyreverse -o puml  --filter-mode ALL --colorized --no-standalone "${DIRECTORIES[@]}"

if [ $? -eq 0 ]; then
    echo "Diagramme UML généré avec succès !"
else
    echo " Erreur lors de la génération du diagramme UML."
fi
